# ArUco ID 1 autonomous following

The follower uses the CAM1 IMX219 camera (`sensor_id=0`) and publishes ROS 2
`geometry_msgs/Twist` commands on `/cmd_vel`. Do not run `keyboard_teleop` at
the same time because both nodes publish to the same topic.

## Control regions

The image is horizontally segmented by normalized marker-center error:

- center (`abs(error) <= 0.15`): move forward and make small steering corrections
- side (`0.15 < abs(error) < 0.60`): slow down and turn toward the marker
- extreme side (`abs(error) >= 0.60`): stop forward motion and rotate in place
- temporarily lost at a side: rotate toward the last side for at most 1.5 seconds
- lost after the search timeout: stop

Only ArUco dictionary `4x4_50`, marker ID `1` controls the robot by default.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ak60_driver --symlink-install
source install/setup.bash
```

## Camera-only dry run (mandatory first test)

This publishes only zero velocity. The four-wheel motor node does not need to
run for this test.

```bash
ros2 run ak60_driver aruco_follower --ros-args \
  -p dry_run:=true \
  -p headless:=true \
  -p target_marker_id:=1 \
  -p marker_size:=0.15
```

In a NoMachine desktop terminal, set `headless:=false` to display the segmented
camera view. Yellow lines bound the center region and red lines bound the
extreme-side regions.

## Live drive

Lift the wheels for the first live direction test and keep an emergency power
cutoff within reach. Confirm that left/right turns are correct before placing
the robot on the floor.

The launch defaults are live drive, headless camera, marker ID 1, 15 cm marker,
1357 px focal length, 0.60 m stop distance, 0.75 m resume distance, and camera
sensor ID 0. After `source ~/.bashrc`, start it with:

```bash
ros2 launch ak60_driver aruco_follow.launch.py
```

Because `dry_run` defaults to `false`, this command can move the robot as soon
as marker ID 1 is visible.

`marker_size` is the measured black-square side length in metres. Incorrect
marker size or focal length directly produces an incorrect stop distance. If
`~/aruco_tracking/measure_distance.py --calibrate 1.0` reports a calibrated
focal length, pass that number as `focal_px` instead of the estimated 1357.0.

Press `Ctrl+C` to stop. The follower publishes zero on a normal shutdown, and
the four-wheel driver also stops independently if `/cmd_vel` is absent for 0.5
seconds.
