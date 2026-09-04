"""Launch the AK60 four-wheel driver and the ArUco follower."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ak60_driver"), "config", "drive.yaml"
    )

    arguments = [
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("target_marker_id", default_value="1"),
        DeclareLaunchArgument("marker_size", default_value="0.15"),
        DeclareLaunchArgument("focal_px", default_value="1357.0"),
        DeclareLaunchArgument("stop_distance", default_value="0.60"),
        DeclareLaunchArgument("resume_distance", default_value="0.75"),
        DeclareLaunchArgument("sensor_id", default_value="0"),
    ]

    driver = Node(
        package="ak60_driver",
        executable="four_wheel_drive_node",
        name="four_wheel_drive_node",
        output="screen",
        parameters=[config],
    )
    follower = Node(
        package="ak60_driver",
        executable="aruco_follower",
        name="aruco_follower",
        output="screen",
        parameters=[{
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
            "headless": ParameterValue(LaunchConfiguration("headless"), value_type=bool),
            "target_marker_id": ParameterValue(
                LaunchConfiguration("target_marker_id"), value_type=int
            ),
            "marker_size": ParameterValue(
                LaunchConfiguration("marker_size"), value_type=float
            ),
            "focal_px": ParameterValue(
                LaunchConfiguration("focal_px"), value_type=float
            ),
            "stop_distance": ParameterValue(
                LaunchConfiguration("stop_distance"), value_type=float
            ),
            "resume_distance": ParameterValue(
                LaunchConfiguration("resume_distance"), value_type=float
            ),
            "sensor_id": ParameterValue(
                LaunchConfiguration("sensor_id"), value_type=int
            ),
        }],
    )

    return LaunchDescription(arguments + [driver, follower])
