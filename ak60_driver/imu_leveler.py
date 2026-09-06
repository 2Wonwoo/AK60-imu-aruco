#!/usr/bin/env python3
"""Level the robot body using an EBIMU-9DOFV6 attached over USB serial.

When a wheel climbs an obstacle the body tilts.  This node reads roll/pitch from
the IMU, works out which corner is raised, and drives that wheel backwards until
the body is parallel to the ground again.

The wheel commands go out on ``/wheel_velocities`` (Float64MultiArray, motor IDs
1..4, forward-positive rad/s) which ``four_wheel_drive_node`` turns into CAN
frames.  ``dry_run`` is on by default so the logic can be checked by hand-tilting
the robot before anything moves.
"""

import glob
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from .imu_level_control import LevelController, LevelSupervisor, WHEEL_NAMES

try:
    import serial
except ImportError:  # pragma: no cover - 런타임 환경에서만 의미가 있다
    serial = None


class ImuLeveler(Node):
    def __init__(self) -> None:
        super().__init__("imu_leveler")

        self.declare_parameter("port", "")            # 비우면 자동 탐색
        self.declare_parameter("baud", 115200)
        self.declare_parameter("publish_rate", 20.0)

        self.declare_parameter("level_threshold", 5.0)
        self.declare_parameter("gain", 0.08)
        self.declare_parameter("max_velocity", 0.6)
        self.declare_parameter("min_velocity", 0.05)
        self.declare_parameter("reverse_only", True)
        self.declare_parameter("max_tilt", 35.0)

        self.declare_parameter("roll_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        # 이 로봇에 IMU 를 장착한 상태에서 평지에 두고 측정한 값(500 샘플 평균).
        # 다른 곳에 옮겨 달았다면 tare_on_start:=true 로 다시 잡을 것.
        self.declare_parameter("roll_offset", -8.72)
        self.declare_parameter("pitch_offset", -4.82)
        # 이 로봇은 IMU 가 바디에 수직축으로 90도 돌아간 채 장착되어 있다.
        # 네 모서리를 하나씩 들어올려 확인한 값 (test_imu_level_control.py 참조).
        self.declare_parameter("mount_yaw_deg", 90.0)
        # 시작 시점의 자세를 0점으로 잡는다. 이후 기울기는 모두 그 기준 대비다.
        # 껐다 켤 때마다 자동으로 맞춰지므로 offset 을 손으로 넣을 필요가 없다.
        self.declare_parameter("tare_on_start", True)
        self.declare_parameter("tare_samples", 50)

        self.declare_parameter("dry_run", True)       # 기본은 모터를 움직이지 않는다
        # 수평이 이 시간만큼 유지되어야 마커 추종에 주도권을 돌려준다.
        # 임계값 근처에서 두 동작이 번갈아 튀는 것을 막는다.
        self.declare_parameter("level_hold_sec", 0.3)
        # True 면 수평일 때 명령을 내보내지 않아 다른 노드(마커 추종)가 로봇을
        # 몬다. False 면 예전처럼 매 주기 명령을 내보낸다 (단독 사용).
        self.declare_parameter("yield_when_level", True)

        self.dry_run = bool(self.parameter("dry_run"))
        self.yield_when_level = bool(self.parameter("yield_when_level"))
        self.controller = LevelController(
            level_threshold=float(self.parameter("level_threshold")),
            gain=float(self.parameter("gain")),
            max_velocity=float(self.parameter("max_velocity")),
            min_velocity=float(self.parameter("min_velocity")),
            reverse_only=bool(self.parameter("reverse_only")),
            roll_sign=float(self.parameter("roll_sign")),
            pitch_sign=float(self.parameter("pitch_sign")),
            roll_offset=float(self.parameter("roll_offset")),
            pitch_offset=float(self.parameter("pitch_offset")),
            mount_yaw_deg=float(self.parameter("mount_yaw_deg")),
            max_tilt=float(self.parameter("max_tilt")),
        )
        self.supervisor = LevelSupervisor(
            controller=self.controller,
            level_hold_sec=float(self.parameter("level_hold_sec")),
        )

        self.publisher = self.create_publisher(Float64MultiArray, "/wheel_velocities", 10)
        self.serial = self.open_imu()

        if bool(self.parameter("tare_on_start")):
            self.get_logger().info(
                "시작 자세를 0점으로 잡는다 - 로봇이 평평한 바닥에 서 있어야 한다"
            )
            self.tare(int(self.parameter("tare_samples")))

        rate = float(self.parameter("publish_rate"))
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        self.last_reason = ""

        self.get_logger().info(
            f"IMU leveler ready: dry_run={self.dry_run}, "
            f"zero=(roll {self.controller.roll_offset:+.2f}, "
            f"pitch {self.controller.pitch_offset:+.2f}), "
            f"deadzone=+-{self.controller.level_threshold:.1f} deg, "
            f"gain={self.controller.gain:.3f}, max={self.controller.max_velocity:.2f} rad/s"
        )
        if self.dry_run:
            self.get_logger().warn(
                "dry_run=True 이므로 명령을 발행하지 않는다. "
                "로봇을 손으로 기울여 판정이 맞는지 먼저 확인할 것"
            )

    def parameter(self, name: str):
        return self.get_parameter(name).value

    def open_imu(self):
        if serial is None:
            raise RuntimeError("pyserial 이 없습니다:  pip3 install --user pyserial")

        port = str(self.parameter("port")).strip()
        if not port:
            candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
            if not candidates:
                raise RuntimeError(
                    "IMU 시리얼 포트를 찾을 수 없습니다 (/dev/ttyUSB*, /dev/ttyACM*)"
                )
            port = candidates[0]

        baud = int(self.parameter("baud"))
        connection = serial.Serial(port, baud, timeout=0.2)
        connection.reset_input_buffer()
        self.get_logger().info(f"IMU connected: {port} @ {baud} bps")
        return connection

    def read_attitude(self) -> Optional[tuple]:
        """가장 최근 자세를 (roll, pitch) 로 돌려준다. 없으면 None."""
        latest = None
        # 센서는 100Hz 로 보내는데 제어는 그보다 느리므로, 밀린 줄을 모두
        # 비우고 마지막 값만 쓴다. 그렇지 않으면 오래된 자세로 제어하게 된다.
        while self.serial.in_waiting > 0:
            line = self.serial.readline()
            if not line:
                break
            text = line.decode("ascii", "replace").strip()
            if not text.startswith("*"):
                continue
            try:
                values = [float(x) for x in text[1:].split(",")]
            except ValueError:
                continue
            if len(values) >= 2:
                latest = (values[0], values[1])
        return latest

    def tare(self, samples: int) -> None:
        """시작 시점의 자세를 0점으로 삼는다.

        이후의 기울기는 모두 이 기준 대비로 판정하므로, 로봇이 **평평한 바닥에
        네 바퀴로 서 있을 때** 실행해야 한다. 장애물에 올라간 상태로 잡으면
        그 기울어진 자세를 수평으로 학습해 버린다.
        """
        rolls, pitches = [], []
        deadline = self.get_clock().now().nanoseconds + int(3e9)
        while len(rolls) < samples and self.get_clock().now().nanoseconds < deadline:
            attitude = self.read_attitude()
            if attitude is not None:
                rolls.append(attitude[0])
                pitches.append(attitude[1])

        if not rolls:
            self.get_logger().warn(
                "tare 실패: IMU 데이터를 받지 못했다. 파라미터의 offset 값을 그대로 쓴다"
            )
            return

        # 기준을 잡는 동안 흔들렸다면 그 기준을 믿을 수 없다.
        spread = max(
            max(rolls) - min(rolls),
            max(pitches) - min(pitches),
        )
        if spread > 1.0:
            self.get_logger().warn(
                f"tare 중 자세가 {spread:.1f}도 흔들렸다. 로봇이 정지해 있는지 "
                f"확인하고 다시 시작할 것"
            )

        self.controller.roll_offset = sum(rolls) / len(rolls)
        self.controller.pitch_offset = sum(pitches) / len(pitches)
        self.get_logger().info(
            f"tare 완료 ({len(rolls)} 샘플): "
            f"roll_offset={self.controller.roll_offset:+.2f}, "
            f"pitch_offset={self.controller.pitch_offset:+.2f}"
        )

    def publish(self, velocities) -> None:
        message = Float64MultiArray()
        message.data = [float(velocities[motor_id]) for motor_id in (1, 2, 3, 4)]
        self.publisher.publish(message)

    def control_loop(self) -> None:
        attitude = self.read_attitude()
        if attitude is None:
            return

        roll, pitch = attitude

        if not self.yield_when_level:
            # 단독 사용: 매 주기 명령을 내보낸다 (수평이면 정지 명령).
            command = self.controller.update(roll, pitch)
            self.report(command.reason, command.velocities, command.active())
            if not self.dry_run:
                self.publish(command.velocities)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        decision = self.supervisor.update(roll, pitch, now)

        self.report(
            f"[{decision.state}] {decision.reason}",
            decision.velocities,
            decision.leveling(),
        )
        if self.dry_run:
            return
        # 수평일 때는 아무것도 내보내지 않아 마커 추종이 로봇을 몰게 둔다.
        if decision.publish:
            self.publish(decision.velocities)

    def report(self, reason, velocities, active) -> None:
        if reason != self.last_reason:
            self.get_logger().info(reason)
            self.last_reason = reason
        if self.dry_run and active and any(velocities.values()):
            detail = "  ".join(
                f"{WHEEL_NAMES[w]}({w}) {velocities[w]:+.2f}" for w in (1, 2, 3, 4)
            )
            self.get_logger().info(f"[dry_run] {detail}", throttle_duration_sec=0.5)

    def destroy_node(self) -> bool:
        try:
            if not self.dry_run:
                self.publish(self.controller.stop().velocities)
        finally:
            if getattr(self, "serial", None) is not None:
                self.serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuLeveler()
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
