#!/usr/bin/env python3
"""ROS 2 ArUco ID follower publishing safe differential-drive /cmd_vel."""

import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .aruco_follow_control import FollowCommand, FollowController


ARUCO_DICTIONARIES = {
    "4x4_50": cv2.aruco.DICT_4X4_50,
    "4x4_100": cv2.aruco.DICT_4X4_100,
    "5x5_100": cv2.aruco.DICT_5X5_100,
    "6x6_250": cv2.aruco.DICT_6X6_250,
    "original": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def gstreamer_pipeline(
    sensor_id: int,
    width: int,
    height: int,
    framerate: int,
    flip_method: int,
) -> str:
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
        "framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! "
        "appsink drop=1 max-buffers=1 sync=false"
        % (sensor_id, width, height, framerate, flip_method, width, height)
    )


class ArucoFollower(Node):
    """Track ArUco ID 1 and publish velocity commands to the AK60 driver."""

    def __init__(self) -> None:
        super().__init__("aruco_follower")

        self.declare_parameter("target_marker_id", 1)
        self.declare_parameter("dictionary", "4x4_50")
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)
        self.declare_parameter("focal_px", 1357.0)

        self.declare_parameter("stop_distance", 0.60)
        self.declare_parameter("resume_distance", 0.75)
        self.declare_parameter("slow_distance", 1.50)
        self.declare_parameter("forward_speed", 0.18)
        self.declare_parameter("min_forward_ratio", 0.35)
        self.declare_parameter("steering_gain", 0.90)
        self.declare_parameter("max_turn_speed", 0.65)
        self.declare_parameter("search_turn_speed", 0.30)
        self.declare_parameter("center_threshold", 0.15)
        self.declare_parameter("edge_threshold", 0.60)
        self.declare_parameter("search_timeout", 1.50)
        self.declare_parameter("distance_filter_alpha", 0.35)

        # dry_run is deliberately true by default. The first camera test cannot
        # move the robot until the operator explicitly opts in.
        self.declare_parameter("dry_run", True)
        self.declare_parameter("headless", True)

        self.target_marker_id = int(self.parameter("target_marker_id"))
        dictionary_name = str(self.parameter("dictionary"))
        if dictionary_name not in ARUCO_DICTIONARIES:
            raise ValueError(f"unsupported ArUco dictionary: {dictionary_name}")

        self.marker_size = float(self.parameter("marker_size"))
        self.sensor_id = int(self.parameter("sensor_id"))
        self.width = int(self.parameter("width"))
        self.height = int(self.parameter("height"))
        self.framerate = int(self.parameter("framerate"))
        self.flip_method = int(self.parameter("flip_method"))
        self.focal_px = float(self.parameter("focal_px"))
        self.dry_run = bool(self.parameter("dry_run"))
        self.headless = bool(self.parameter("headless"))
        self.filter_alpha = float(self.parameter("distance_filter_alpha"))

        if self.marker_size <= 0.0 or self.focal_px <= 0.0:
            raise ValueError("marker_size and focal_px must be positive")
        if self.width <= 0 or self.height <= 0 or self.framerate <= 0:
            raise ValueError("camera dimensions and framerate must be positive")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("distance_filter_alpha must be in (0, 1]")

        self.controller = FollowController(
            stop_distance=float(self.parameter("stop_distance")),
            resume_distance=float(self.parameter("resume_distance")),
            slow_distance=float(self.parameter("slow_distance")),
            forward_speed=float(self.parameter("forward_speed")),
            min_forward_ratio=float(self.parameter("min_forward_ratio")),
            steering_gain=float(self.parameter("steering_gain")),
            max_turn_speed=float(self.parameter("max_turn_speed")),
            search_turn_speed=float(self.parameter("search_turn_speed")),
            center_threshold=float(self.parameter("center_threshold")),
            edge_threshold=float(self.parameter("edge_threshold")),
            search_timeout=float(self.parameter("search_timeout")),
        )

        self.aruco_dictionary = cv2.aruco.Dictionary_get(
            ARUCO_DICTIONARIES[dictionary_name]
        )
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.camera_matrix = np.array(
            [
                [self.focal_px, 0.0, self.width / 2.0],
                [0.0, self.focal_px, self.height / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        self.distortion = np.zeros((5, 1), dtype=np.float64)

        pipeline = gstreamer_pipeline(
            self.sensor_id,
            self.width,
            self.height,
            self.framerate,
            self.flip_method,
        )
        self.capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            raise RuntimeError(
                "camera open failed; check CAM1 connection and sensor_id=0"
            )

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_seen_at: Optional[float] = None
        self.filtered_distance: Optional[float] = None
        self.last_state = "STARTING"
        self.closed = False
        self.timer = self.create_timer(1.0 / self.framerate, self.update)

        mode = "DRY RUN (motors will not move)" if self.dry_run else "LIVE DRIVE"
        self.get_logger().warn(
            f"ArUco follower ready: ID={self.target_marker_id}, {mode}, "
            f"stop={self.controller.stop_distance:.2f}m, "
            f"resume={self.controller.resume_distance:.2f}m"
        )

    def parameter(self, name: str):
        return self.get_parameter(name).value

    def publish(self, command: FollowCommand) -> None:
        message = Twist()
        if not self.dry_run:
            message.linear.x = command.linear
            message.angular.z = command.angular
        self.publisher.publish(message)

        if command.state != self.last_state:
            self.get_logger().info(
                f"state={command.state} linear={message.linear.x:+.3f} "
                f"angular={message.angular.z:+.3f}"
            )
            self.last_state = command.state

    def target_from_frame(self, frame) -> Optional[Tuple[np.ndarray, float, float]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dictionary, parameters=self.detector_parameters
        )
        if ids is None:
            return None

        matching = np.flatnonzero(ids.flatten() == self.target_marker_id)
        if matching.size == 0:
            return None

        marker_corners = corners[int(matching[0])]
        _, translations, _ = cv2.aruco.estimatePoseSingleMarkers(
            [marker_corners], self.marker_size, self.camera_matrix, self.distortion
        )
        distance = float(np.linalg.norm(translations[0][0]))
        center_x = float(marker_corners[0][:, 0].mean())
        horizontal_error = (center_x - self.width / 2.0) / (self.width / 2.0)
        return marker_corners, distance, horizontal_error

    def update(self) -> None:
        ok, frame = self.capture.read()
        now = time.monotonic()
        if not ok:
            self.publish(self.controller.stop("CAMERA_FAILURE"))
            self.get_logger().error(
                "camera frame read failed; commanding stop",
                throttle_duration_sec=2.0,
            )
            return

        target = self.target_from_frame(frame)
        distance = None
        error = None
        marker_corners = None

        if target is not None:
            marker_corners, raw_distance, error = target
            if self.filtered_distance is None:
                self.filtered_distance = raw_distance
            else:
                alpha = self.filter_alpha
                self.filtered_distance = (
                    alpha * raw_distance + (1.0 - alpha) * self.filtered_distance
                )
            distance = self.filtered_distance
            self.last_seen_at = now
            # Never let smoothing delay a stop when the latest raw observation
            # says the marker is closer than the filtered estimate.
            control_distance = min(raw_distance, self.filtered_distance)
            command = self.controller.update_detection(control_distance, error)
        else:
            age = float("inf") if self.last_seen_at is None else now - self.last_seen_at
            command = self.controller.update_missing(age)
            if age > self.controller.search_timeout:
                self.filtered_distance = None

        self.publish(command)
        if not self.headless:
            self.draw_debug(frame, marker_corners, distance, error, command)
            cv2.imshow("AK60 ArUco Follower", frame)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                self.publish(self.controller.stop("USER_STOP"))
                rclpy.shutdown()

    def draw_debug(
        self,
        frame,
        marker_corners,
        distance: Optional[float],
        error: Optional[float],
        command: FollowCommand,
    ) -> None:
        center = self.width // 2
        center_offset = int(self.controller.center_threshold * center)
        edge_offset = int(self.controller.edge_threshold * center)
        for x, color in (
            (center - edge_offset, (0, 0, 255)),
            (center - center_offset, (0, 255, 255)),
            (center + center_offset, (0, 255, 255)),
            (center + edge_offset, (0, 0, 255)),
        ):
            cv2.line(frame, (x, 0), (x, self.height), color, 1)

        if marker_corners is not None:
            cv2.aruco.drawDetectedMarkers(
                frame, [marker_corners], np.array([[self.target_marker_id]])
            )
        text = f"ID {self.target_marker_id} {command.state}"
        if distance is not None and error is not None:
            text += f" d={distance:.2f}m err={error:+.2f}"
        if self.dry_run:
            text += " [DRY RUN]"
        cv2.putText(
            frame, text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )

    def stop(self) -> None:
        # rclpy's SIGINT handler may invalidate the context before finally runs.
        # In that case the motor driver independently stops on its 0.5 s command
        # timeout, and publishing through the invalid context would raise RCLError.
        if not rclpy.ok():
            return
        for _ in range(5):
            self.publish(self.controller.stop("SHUTDOWN"))

    def destroy_node(self) -> bool:
        if not self.closed:
            self.stop()
            self.capture.release()
            if not self.headless:
                cv2.destroyAllWindows()
            self.closed = True
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
