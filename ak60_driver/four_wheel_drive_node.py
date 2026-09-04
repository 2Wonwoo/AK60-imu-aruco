#!/usr/bin/env python3
"""ROS 2 /cmd_vel to four AK60-6 motors through SocketCAN."""

import socket
import struct
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .protocol import Ak60V3Protocol, CAN_EFF_FLAG, clamp


class SocketCanSender:
    def __init__(self, interface: str) -> None:
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((interface,))

    def send_extended(self, can_id: int, payload: bytes) -> None:
        if len(payload) != 8:
            raise ValueError("AK60 command payload must contain exactly 8 bytes")
        frame = struct.pack("=IB3x8s", can_id | CAN_EFF_FLAG, 8, payload)
        self.socket.send(frame)

    def close(self) -> None:
        self.socket.close()


class FourWheelDriveNode(Node):
    """Convert differential-drive velocity commands into four motor commands."""

    MOTOR_DIRECTION = {
        1: 1.0,   # front right
        2: -1.0,  # front left
        3: 1.0,   # rear right
        4: -1.0,  # rear left
    }

    def __init__(self) -> None:
        super().__init__("four_wheel_drive_node")

        self.declare_parameter("can_interface", "can1")
        self.declare_parameter("wheel_radius", 0.10)
        self.declare_parameter("track_width", 0.50)
        self.declare_parameter("max_wheel_velocity", 2.0)
        self.declare_parameter("kd", 1.0)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("command_timeout", 0.5)

        interface = self.get_parameter("can_interface").value
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.track_width = float(self.get_parameter("track_width").value)
        self.max_wheel_velocity = float(
            self.get_parameter("max_wheel_velocity").value
        )
        self.kd = float(self.get_parameter("kd").value)
        control_rate = float(self.get_parameter("control_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius must be positive")
        if self.track_width <= 0.0:
            raise ValueError("track_width must be positive")
        if control_rate <= 0.0:
            raise ValueError("control_rate must be positive")

        self.protocol = Ak60V3Protocol()
        self.bus = SocketCanSender(str(interface))
        self.targets: Dict[int, float] = {motor_id: 0.0 for motor_id in range(1, 5)}
        self.last_command_time = self.get_clock().now()
        self.timed_out = True
        self.closed = False

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        # 바퀴를 개별로 돌려야 하는 동작(예: IMU 자세 복원)을 위한 입력.
        # 배열 순서는 모터 ID 1,2,3,4 이고 단위는 전진 양수인 rad/s.
        self.wheel_subscription = self.create_subscription(
            Float64MultiArray, "/wheel_velocities", self.wheel_velocities_callback, 10
        )
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        try:
            self.send_stop(repetitions=10)
        except OSError as error:
            # 인터페이스가 내려가 있으면 여기서 처음 드러난다. 트레이스백만
            # 남기면 원인을 알기 어려우므로 조치 방법까지 알려준다.
            raise RuntimeError(
                f"CAN 인터페이스 '{interface}' 로 보낼 수 없습니다 ({error}). "
                f"먼저 인터페이스를 올리세요:  ./can_check.sh {interface} 1000000"
            ) from error

        self.get_logger().info(
            "Four-wheel driver ready: ID 1/3=right, ID 2/4=left, "
            f"interface={interface}, max={self.max_wheel_velocity:.1f} rad/s"
        )

    def cmd_vel_callback(self, message: Twist) -> None:
        linear = float(message.linear.x)
        angular = float(message.angular.z)

        left_linear = linear - angular * self.track_width / 2.0
        right_linear = linear + angular * self.track_width / 2.0
        left_wheel = left_linear / self.wheel_radius
        right_wheel = right_linear / self.wheel_radius

        left_wheel = clamp(
            left_wheel, -self.max_wheel_velocity, self.max_wheel_velocity
        )
        right_wheel = clamp(
            right_wheel, -self.max_wheel_velocity, self.max_wheel_velocity
        )

        self.targets = {
            1: self.MOTOR_DIRECTION[1] * right_wheel,
            2: self.MOTOR_DIRECTION[2] * left_wheel,
            3: self.MOTOR_DIRECTION[3] * right_wheel,
            4: self.MOTOR_DIRECTION[4] * left_wheel,
        }
        self.last_command_time = self.get_clock().now()
        self.timed_out = False

    def wheel_velocities_callback(self, message: Float64MultiArray) -> None:
        """바퀴별 속도 명령. data[0..3] 이 모터 ID 1..4, 전진 양수인 rad/s."""
        if len(message.data) != 4:
            self.get_logger().warn(
                f"/wheel_velocities expects 4 values, got {len(message.data)}",
                throttle_duration_sec=2.0,
            )
            return

        self.targets = {
            motor_id: self.MOTOR_DIRECTION[motor_id]
            * clamp(
                float(message.data[motor_id - 1]),
                -self.max_wheel_velocity,
                self.max_wheel_velocity,
            )
            for motor_id in (1, 2, 3, 4)
        }
        self.last_command_time = self.get_clock().now()
        self.timed_out = False

    def send_targets(self, targets: Dict[int, float]) -> None:
        for motor_id in (1, 2, 3, 4):
            payload = self.protocol.velocity_command(targets[motor_id], self.kd)
            self.bus.send_extended(self.protocol.control_id(motor_id), payload)

    def send_stop(self, repetitions: int = 1) -> None:
        stopped = {motor_id: 0.0 for motor_id in range(1, 5)}
        for _ in range(repetitions):
            self.send_targets(stopped)
            if repetitions > 1:
                time.sleep(0.02)

    def control_loop(self) -> None:
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        targets = self.targets
        if age > self.command_timeout:
            targets = {motor_id: 0.0 for motor_id in range(1, 5)}
            if not self.timed_out:
                self.get_logger().warn("command timeout: stopping all motors")
                self.timed_out = True

        try:
            self.send_targets(targets)
        except OSError as error:
            self.get_logger().error(
                f"CAN send failed: {error}", throttle_duration_sec=2.0
            )

    def destroy_node(self) -> bool:
        if not self.closed:
            try:
                self.send_stop(repetitions=10)
            except OSError as error:
                self.get_logger().error(f"CAN stop failed during shutdown: {error}")
            finally:
                self.bus.close()
                self.closed = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FourWheelDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
