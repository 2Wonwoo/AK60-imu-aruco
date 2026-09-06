"""Launch the AK60 four-wheel driver and the IMU body leveller."""

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
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("port", default_value=""),
        DeclareLaunchArgument("level_threshold", default_value="2.5"),
        DeclareLaunchArgument("roll_offset", default_value="-2.31"),
        DeclareLaunchArgument("pitch_offset", default_value="1.83"),
        DeclareLaunchArgument("gain", default_value="0.08"),
        DeclareLaunchArgument("max_velocity", default_value="0.6"),
        DeclareLaunchArgument("reverse_only", default_value="true"),
        DeclareLaunchArgument("roll_sign", default_value="1.0"),
        DeclareLaunchArgument("pitch_sign", default_value="1.0"),
        DeclareLaunchArgument("mount_yaw_deg", default_value="90.0"),
        # 시작 시점의 자세를 0점으로 잡는다 (평평한 바닥에서 실행할 것).
        # false 로 두면 위의 roll_offset / pitch_offset 을 그대로 쓴다.
        DeclareLaunchArgument("tare_on_start", default_value="true"),
    ]

    driver = Node(
        package="ak60_driver",
        executable="four_wheel_drive_node",
        name="four_wheel_drive_node",
        output="screen",
        parameters=[config],
    )
    leveler = Node(
        package="ak60_driver",
        executable="imu_leveler",
        name="imu_leveler",
        output="screen",
        parameters=[{
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
            "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
            "level_threshold": ParameterValue(
                LaunchConfiguration("level_threshold"), value_type=float
            ),
            "roll_offset": ParameterValue(
                LaunchConfiguration("roll_offset"), value_type=float
            ),
            "pitch_offset": ParameterValue(
                LaunchConfiguration("pitch_offset"), value_type=float
            ),
            "gain": ParameterValue(LaunchConfiguration("gain"), value_type=float),
            "max_velocity": ParameterValue(
                LaunchConfiguration("max_velocity"), value_type=float
            ),
            "reverse_only": ParameterValue(
                LaunchConfiguration("reverse_only"), value_type=bool
            ),
            "roll_sign": ParameterValue(
                LaunchConfiguration("roll_sign"), value_type=float
            ),
            "pitch_sign": ParameterValue(
                LaunchConfiguration("pitch_sign"), value_type=float
            ),
            "mount_yaw_deg": ParameterValue(
                LaunchConfiguration("mount_yaw_deg"), value_type=float
            ),
            "tare_on_start": ParameterValue(
                LaunchConfiguration("tare_on_start"), value_type=bool
            ),
        }],
    )

    return LaunchDescription(arguments + [driver, leveler])
