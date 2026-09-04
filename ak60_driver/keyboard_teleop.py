#!/usr/bin/env python3
"""Small terminal keyboard teleop node for the AK60 robot."""

import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
AK60 keyboard drive
-------------------
        W: forward
   A: left   D: right
        S: reverse

SPACE or X: stop
Q: stop and quit
"""


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("ak60_keyboard_teleop")
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard_teleop must be run in an interactive terminal")

        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("publish_rate", 20.0)

        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear = 0.0
        self.angular = 0.0
        self.old_terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.terminal_restored = False

        self.timer = self.create_timer(1.0 / publish_rate, self.update)
        print(HELP, flush=True)

    def read_key(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1).lower() if ready else None

    def publish_command(self) -> None:
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.publisher.publish(message)

    def stop(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self.publish_command()

    def update(self) -> None:
        key = self.read_key()
        if key == "w":
            self.linear, self.angular = self.linear_speed, 0.0
        elif key == "s":
            self.linear, self.angular = -self.linear_speed, 0.0
        elif key == "a":
            self.linear, self.angular = 0.0, self.angular_speed
        elif key == "d":
            self.linear, self.angular = 0.0, -self.angular_speed
        elif key in (" ", "x"):
            self.linear, self.angular = 0.0, 0.0
        elif key == "q":
            self.stop()
            rclpy.shutdown()
            return

        self.publish_command()

    def restore_terminal(self) -> None:
        if not self.terminal_restored:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal)
            self.terminal_restored = True

    def destroy_node(self) -> bool:
        self.stop()
        self.restore_terminal()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
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

