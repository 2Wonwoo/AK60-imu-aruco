"""Pure control policy for following one ArUco marker.

The camera image is segmented horizontally into center, side, and extreme-side
zones.  Keeping this module independent from ROS and OpenCV makes the safety
logic easy to unit test without opening the camera or CAN interface.
"""

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class FollowCommand:
    linear: float
    angular: float
    state: str


class FollowController:
    """Convert marker distance/position into differential-drive commands."""

    def __init__(
        self,
        *,
        stop_distance: float,
        resume_distance: float,
        slow_distance: float,
        forward_speed: float,
        min_forward_ratio: float,
        steering_gain: float,
        max_turn_speed: float,
        search_turn_speed: float,
        center_threshold: float,
        edge_threshold: float,
        search_timeout: float,
    ) -> None:
        if stop_distance <= 0.0:
            raise ValueError("stop_distance must be positive")
        if resume_distance <= stop_distance:
            raise ValueError("resume_distance must be greater than stop_distance")
        if slow_distance <= resume_distance:
            raise ValueError("slow_distance must be greater than resume_distance")
        if not 0.0 < center_threshold < edge_threshold < 1.0:
            raise ValueError(
                "zone thresholds must satisfy 0 < center < edge < 1"
            )
        if forward_speed <= 0.0 or max_turn_speed <= 0.0:
            raise ValueError("drive speeds must be positive")
        if search_turn_speed < 0.0 or search_timeout < 0.0:
            raise ValueError("search settings cannot be negative")

        self.stop_distance = stop_distance
        self.resume_distance = resume_distance
        self.slow_distance = slow_distance
        self.forward_speed = forward_speed
        self.min_forward_ratio = clamp(min_forward_ratio, 0.0, 1.0)
        self.steering_gain = steering_gain
        self.max_turn_speed = max_turn_speed
        self.search_turn_speed = search_turn_speed
        self.center_threshold = center_threshold
        self.edge_threshold = edge_threshold
        self.search_timeout = search_timeout

        self.holding_distance = False
        self.last_error = 0.0

    @staticmethod
    def stop(state: str) -> FollowCommand:
        return FollowCommand(0.0, 0.0, state)

    def update_detection(self, distance: float, horizontal_error: float) -> FollowCommand:
        """Return a command for a visible marker.

        ``horizontal_error`` is normalized to [-1, 1]: negative is left and
        positive is right. ROS angular.z is positive for a left turn, hence the
        negative sign in the steering command.
        """
        if distance <= 0.0:
            return self.stop("INVALID_DISTANCE")

        error = clamp(horizontal_error, -1.0, 1.0)
        self.last_error = error

        if distance <= self.stop_distance:
            self.holding_distance = True
        elif self.holding_distance and distance >= self.resume_distance:
            self.holding_distance = False

        if self.holding_distance:
            return self.stop("DISTANCE_HOLD")

        angular = clamp(
            -self.steering_gain * error,
            -self.max_turn_speed,
            self.max_turn_speed,
        )
        absolute_error = abs(error)

        # The outer image segments rotate in place. This keeps a marker near an
        # image edge from leaving the field of view while the robot moves ahead.
        if absolute_error >= self.edge_threshold:
            if abs(angular) < self.search_turn_speed:
                angular = -self.search_turn_speed if error > 0.0 else self.search_turn_speed
            return FollowCommand(0.0, angular, "EDGE_RIGHT" if error > 0.0 else "EDGE_LEFT")

        distance_ratio = (distance - self.stop_distance) / (
            self.slow_distance - self.stop_distance
        )
        distance_ratio = clamp(
            distance_ratio, self.min_forward_ratio, 1.0
        )
        steering_ratio = clamp(1.0 - 0.65 * absolute_error, 0.35, 1.0)
        linear = self.forward_speed * distance_ratio * steering_ratio

        if absolute_error <= self.center_threshold:
            state = "FOLLOW_CENTER"
        elif error < 0.0:
            state = "FOLLOW_LEFT"
        else:
            state = "FOLLOW_RIGHT"
        return FollowCommand(linear, angular, state)

    def update_missing(self, seconds_since_seen: float) -> FollowCommand:
        """Rotate briefly toward the last edge position, then stop safely."""
        if self.holding_distance:
            return self.stop("LOST_DISTANCE_HOLD")
        if (
            0.0 <= seconds_since_seen <= self.search_timeout
            and abs(self.last_error) > self.center_threshold
            and self.search_turn_speed > 0.0
        ):
            angular = (
                -self.search_turn_speed
                if self.last_error > 0.0
                else self.search_turn_speed
            )
            return FollowCommand(0.0, angular, "SEARCH_LAST_SIDE")
        return self.stop("MARKER_LOST")
