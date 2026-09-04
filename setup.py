import os
from glob import glob

from setuptools import find_packages, setup


package_name = "ak60_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@example.com",
    description="ROS 2 SocketCAN drive and ArUco following for four AK60-6 motors",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "ak60_mit_node = ak60_driver.ak60_mit_node:main",
            "four_wheel_drive_node = ak60_driver.four_wheel_drive_node:main",
            "keyboard_teleop = ak60_driver.keyboard_teleop:main",
            "aruco_follower = ak60_driver.aruco_follower:main",
            "imu_leveler = ak60_driver.imu_leveler:main",
        ],
    },
)
