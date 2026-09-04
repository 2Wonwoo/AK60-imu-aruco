from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ak60_driver"), "config", "drive.yaml"
    )
    return LaunchDescription([
        Node(
            package="ak60_driver",
            executable="four_wheel_drive_node",
            name="four_wheel_drive_node",
            output="screen",
            parameters=[config],
        )
    ])

