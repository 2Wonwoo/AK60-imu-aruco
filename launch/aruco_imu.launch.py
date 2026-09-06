"""ArUco 마커를 추종하되, 바디가 기울면 자세 복원을 우선하는 통합 실행.

세 노드가 함께 뜬다::

    aruco_follower  --/cmd_vel----------+
                                        +--> four_wheel_drive_node --> CAN
    imu_leveler  --/wheel_velocities----+

평소에는 imu_leveler 가 아무것도 내보내지 않아 마커 추종이 로봇을 몬다.
바디가 기울면 imu_leveler 가 전체를 정지시킨 뒤 들린 바퀴를 후진시키고,
그동안 four_wheel_drive_node 는 /cmd_vel 을 무시한다. 수평이 돌아오면
발행을 멈춰 주도권이 마커 추종으로 돌아간다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ak60_driver"), "config", "drive.yaml"
    )

    arguments = [
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("headless", default_value="true"),
        # 마커 추종
        DeclareLaunchArgument("target_marker_id", default_value="1"),
        DeclareLaunchArgument("marker_size", default_value="0.15"),
        DeclareLaunchArgument("focal_px", default_value="1357.0"),
        DeclareLaunchArgument("stop_distance", default_value="0.60"),
        DeclareLaunchArgument("resume_distance", default_value="0.75"),
        DeclareLaunchArgument("sensor_id", default_value="0"),
        # 자세 복원
        DeclareLaunchArgument("level_threshold", default_value="2.5"),
        DeclareLaunchArgument("gain", default_value="0.08"),
        DeclareLaunchArgument("max_velocity", default_value="0.6"),
        DeclareLaunchArgument("roll_offset", default_value="-8.72"),
        DeclareLaunchArgument("pitch_offset", default_value="-4.82"),
        DeclareLaunchArgument("mount_yaw_deg", default_value="90.0"),
        DeclareLaunchArgument("level_hold_sec", default_value="0.3"),
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
    follower = Node(
        package="ak60_driver",
        executable="aruco_follower",
        name="aruco_follower",
        output="screen",
        parameters=[{
            "dry_run": bool_param("dry_run"),
            "headless": bool_param("headless"),
            "target_marker_id": int_param("target_marker_id"),
            "marker_size": float_param("marker_size"),
            "focal_px": float_param("focal_px"),
            "stop_distance": float_param("stop_distance"),
            "resume_distance": float_param("resume_distance"),
            "sensor_id": int_param("sensor_id"),
        }],
    )
    leveler = Node(
        package="ak60_driver",
        executable="imu_leveler",
        name="imu_leveler",
        output="screen",
        parameters=[{
            "dry_run": bool_param("dry_run"),
            "level_threshold": float_param("level_threshold"),
            "gain": float_param("gain"),
            "max_velocity": float_param("max_velocity"),
            "roll_offset": float_param("roll_offset"),
            "pitch_offset": float_param("pitch_offset"),
            "mount_yaw_deg": float_param("mount_yaw_deg"),
            "level_hold_sec": float_param("level_hold_sec"),
            "tare_on_start": bool_param("tare_on_start"),
            # 수평일 때는 발행하지 않아 마커 추종에 주도권을 넘긴다
            "yield_when_level": True,
        }],
    )

    return LaunchDescription(arguments + [driver, follower, leveler])
