#!/usr/bin/env python3

import math
import socket
import struct
import time
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def float_to_uint(
    value: float,
    minimum: float,
    maximum: float,
    bits: int,
) -> int:
    """물리값을 MIT 프로토콜용 unsigned integer로 변환."""
    value = clamp(value, minimum, maximum)
    span = maximum - minimum
    return int((value - minimum) * ((1 << bits) - 1) / span)


def uint_to_float(
    value: int,
    minimum: float,
    maximum: float,
    bits: int,
) -> float:
    """MIT 프로토콜 unsigned integer를 물리값으로 변환."""
    span = maximum - minimum
    return value * span / ((1 << bits) - 1) + minimum


class SocketCan:
    """Linux SocketCAN 송신 클래스."""

    def __init__(self, interface: str) -> None:
        self.socket = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW,
        )
        self.socket.bind((interface,))
        self.socket.setblocking(False)

    def send(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("Classic CAN payload cannot exceed 8 bytes")

        payload = data.ljust(8, b"\x00")

        # struct can_frame:
        # can_id: uint32
        # can_dlc: uint8
        # padding: 3 bytes
        # data: 8 bytes
        frame = struct.pack(
            "=IB3x8s",
            can_id,
            len(data),
            payload,
        )
        # ENOBUFS(105)를 여기서 삼키면 "명령은 보냈는데 모터는 안 도는"
        # 상황이 되므로, 호출부가 판단할 수 있게 그대로 올린다.
        self.socket.send(frame)

    def recv_feedback(self) -> tuple:
        """대기 중인 MIT 피드백 프레임을 논블로킹으로 하나 읽는다."""
        try:
            frame = self.socket.recv(16)
        except BlockingIOError:
            return ()

        data = struct.unpack("=IB3x8s", frame)[2]
        motor_id = data[0]
        p_int = (data[1] << 8) | data[2]
        v_int = (data[3] << 4) | (data[4] >> 4)
        i_int = ((data[4] & 0x0F) << 8) | data[5]
        return (motor_id, p_int, v_int, i_int)

    def close(self) -> None:
        self.socket.close()


class Ak60MitNode(Node):
    # AK70-10 MIT 범위 (CubeMars 매뉴얼 기준).
    # 참고: AK60-6은 V ±45, T ±15로 다르다.
    # 펌웨어 버전에 따라 범위가 다르면 반드시 매뉴얼과 대조해 수정해야 한다.
    P_MIN = -12.5
    P_MAX = 12.5

    V_MIN = -50.0
    V_MAX = 50.0

    KP_MIN = 0.0
    KP_MAX = 500.0

    KD_MIN = 0.0
    KD_MAX = 5.0

    T_MIN = -25.0
    T_MAX = 25.0

    def __init__(self) -> None:
        super().__init__("ak60_mit_node")

        self.declare_parameter("can_interface", "can1")
        self.declare_parameter("motor_id", 1)

        # 단일 모터 시험에서는 /cmd_vel linear.x를
        # 모터 목표속도 rad/s로 변환하기 위한 배율이다.
        self.declare_parameter("velocity_scale", 5.0)
        self.declare_parameter("max_motor_velocity", 3.0)
        self.declare_parameter("kd", 0.5)
        self.declare_parameter("command_timeout", 0.5)

        self.can_interface = (
            self.get_parameter("can_interface")
            .get_parameter_value()
            .string_value
        )
        self.motor_id = (
            self.get_parameter("motor_id")
            .get_parameter_value()
            .integer_value
        )
        self.velocity_scale = (
            self.get_parameter("velocity_scale")
            .get_parameter_value()
            .double_value
        )
        self.max_motor_velocity = (
            self.get_parameter("max_motor_velocity")
            .get_parameter_value()
            .double_value
        )
        self.kd = (
            self.get_parameter("kd")
            .get_parameter_value()
            .double_value
        )
        self.command_timeout = (
            self.get_parameter("command_timeout")
            .get_parameter_value()
            .double_value
        )

        if not 0 <= self.motor_id <= 0x7FF:
            raise ValueError("motor_id must be a standard 11-bit CAN ID")

        self.bus = SocketCan(self.can_interface)

        self.target_velocity = 0.0
        self.last_command_time = self.get_clock().now()

        self.feedback_count = 0
        self.last_feedback = ()

        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        # 10 Hz로 MIT 명령 반복 전송
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"AK60 MIT node started: "
            f"interface={self.can_interface}, ID={self.motor_id}"
        )

        self.enable_motor()
        time.sleep(0.1)

    def enable_motor(self) -> None:
        """
        일반적인 CubeMars MIT Enable 프레임.

        일부 V3 펌웨어/설정에서는 전용 진입 절차가 다를 수 있으므로,
        반응이 없으면 현재 펌웨어용 공식 Demo와 대조해야 한다.
        """
        try:
            self.bus.send(
                self.motor_id,
                bytes.fromhex("FFFFFFFFFFFFFFFC"),
            )
        except OSError as error:
            self.get_logger().error(
                f"MIT enable 송신 실패: {error} "
                f"(모터 전원·배선·종단저항 확인 필요)"
            )
            return

        self.get_logger().info("MIT enable frame sent")

    def disable_motor(self) -> None:
        self.bus.send(
            self.motor_id,
            bytes.fromhex("FFFFFFFFFFFFFFFD"),
        )
        self.get_logger().info("MIT disable frame sent")

    def cmd_vel_callback(self, msg: Twist) -> None:
        # 단일 모터 시험:
        # linear.x를 모터 rad/s로 변환
        requested = msg.linear.x * self.velocity_scale

        self.target_velocity = clamp(
            requested,
            -self.max_motor_velocity,
            self.max_motor_velocity,
        )
        self.last_command_time = self.get_clock().now()

    def make_mit_command(
        self,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
        torque: float,
    ) -> bytes:
        p_int = float_to_uint(
            position,
            self.P_MIN,
            self.P_MAX,
            16,
        )
        v_int = float_to_uint(
            velocity,
            self.V_MIN,
            self.V_MAX,
            12,
        )
        kp_int = float_to_uint(
            kp,
            self.KP_MIN,
            self.KP_MAX,
            12,
        )
        kd_int = float_to_uint(
            kd,
            self.KD_MIN,
            self.KD_MAX,
            12,
        )
        t_int = float_to_uint(
            torque,
            self.T_MIN,
            self.T_MAX,
            12,
        )

        data: List[int] = [0] * 8

        data[0] = (p_int >> 8) & 0xFF
        data[1] = p_int & 0xFF

        data[2] = (v_int >> 4) & 0xFF
        data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
        data[4] = kp_int & 0xFF

        data[5] = (kd_int >> 4) & 0xFF
        data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
        data[7] = t_int & 0xFF

        return bytes(data)

    def control_loop(self) -> None:
        elapsed = (
            self.get_clock().now() - self.last_command_time
        ).nanoseconds / 1e9

        # 일정 시간 /cmd_vel이 없으면 자동 정지
        velocity = (
            self.target_velocity
            if elapsed <= self.command_timeout
            else 0.0
        )

        command = self.make_mit_command(
            position=0.0,
            velocity=velocity,
            kp=0.0,
            kd=self.kd,
            torque=0.0,
        )

        try:
            self.bus.send(self.motor_id, command)
        except OSError as error:
            # ENOBUFS(105)면 버스에 ACK을 주는 노드가 없다는 뜻이다.
            # (모터 전원 / CANH-CANL 배선 / 120옴 종단 확인)
            self.get_logger().error(
                f"CAN 송신 실패: {error} "
                f"(모터 전원·배선·종단저항 확인 필요)",
                throttle_duration_sec=2.0,
            )
            return

        self.drain_feedback()

    def drain_feedback(self) -> None:
        """모터 피드백을 모두 읽어 상태를 갱신한다."""
        got = False
        while True:
            feedback = self.bus.recv_feedback()
            if not feedback:
                break
            got = True
            self.feedback_count += 1
            motor_id, p_int, v_int, i_int = feedback
            self.last_feedback = (
                motor_id,
                uint_to_float(p_int, self.P_MIN, self.P_MAX, 16),
                uint_to_float(v_int, self.V_MIN, self.V_MAX, 12),
                uint_to_float(i_int, self.T_MIN, self.T_MAX, 12),
            )

        if got:
            motor_id, pos, vel, cur = self.last_feedback
            self.get_logger().info(
                f"피드백 ID={motor_id} "
                f"pos={pos:.3f} rad vel={vel:.3f} rad/s cur={cur:.2f} A",
                throttle_duration_sec=1.0,
            )
        elif self.feedback_count == 0:
            self.get_logger().warn(
                "모터 피드백이 전혀 없다. "
                "모터가 버스에 응답하지 않는 상태다.",
                throttle_duration_sec=3.0,
            )

    def destroy_node(self) -> bool:
        try:
            # 먼저 속도 0을 여러 번 전송
            stop_command = self.make_mit_command(
                position=0.0,
                velocity=0.0,
                kp=0.0,
                kd=self.kd,
                torque=0.0,
            )

            for _ in range(5):
                self.bus.send(self.motor_id, stop_command)
                time.sleep(0.01)

            self.disable_motor()
            self.bus.close()

        except OSError as error:
            self.get_logger().error(
                f"CAN shutdown error: {error}"
            )

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ak60MitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
