from ak60_driver.aruco_follow_control import FollowController


def controller():
    return FollowController(
        stop_distance=0.60,
        resume_distance=0.75,
        slow_distance=1.50,
        forward_speed=0.18,
        min_forward_ratio=0.35,
        steering_gain=0.90,
        max_turn_speed=0.65,
        search_turn_speed=0.30,
        center_threshold=0.15,
        edge_threshold=0.60,
        search_timeout=1.50,
    )


def test_close_marker_stops_and_uses_hysteresis():
    policy = controller()
    assert policy.update_detection(0.55, 0.0).state == "DISTANCE_HOLD"
    assert policy.update_detection(0.70, 0.0).state == "DISTANCE_HOLD"
    resumed = policy.update_detection(0.80, 0.0)
    assert resumed.state == "FOLLOW_CENTER"
    assert resumed.linear > 0.0


def test_center_marker_drives_forward():
    command = controller().update_detection(1.50, 0.0)
    assert command.state == "FOLLOW_CENTER"
    assert command.linear > 0.0
    assert command.angular == 0.0


def test_left_and_right_segments_steer_toward_marker():
    policy = controller()
    left = policy.update_detection(1.50, -0.40)
    right = policy.update_detection(1.50, 0.40)
    assert left.state == "FOLLOW_LEFT" and left.angular > 0.0
    assert right.state == "FOLLOW_RIGHT" and right.angular < 0.0


def test_extreme_side_rotates_without_forward_motion():
    policy = controller()
    left = policy.update_detection(1.50, -0.85)
    right = policy.update_detection(1.50, 0.85)
    assert left.state == "EDGE_LEFT" and left.linear == 0.0 and left.angular > 0.0
    assert right.state == "EDGE_RIGHT" and right.linear == 0.0 and right.angular < 0.0


def test_lost_marker_search_is_bounded_then_stops():
    policy = controller()
    policy.update_detection(1.50, 0.80)
    search = policy.update_missing(0.5)
    stopped = policy.update_missing(2.0)
    assert search.state == "SEARCH_LAST_SIDE" and search.angular < 0.0
    assert stopped.state == "MARKER_LOST"
    assert stopped.linear == 0.0 and stopped.angular == 0.0
