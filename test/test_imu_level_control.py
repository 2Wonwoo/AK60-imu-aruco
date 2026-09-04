from ak60_driver.imu_level_control import LevelController, WHEEL_NAMES


# 이 로봇의 실측 장착 오차 (평지 500 샘플 평균)
ROBOT_ROLL_OFFSET = -2.31
ROBOT_PITCH_OFFSET = 1.83


def controller(**overrides):
    settings = dict(
        level_threshold=5.0,
        gain=0.08,
        max_velocity=0.6,
        min_velocity=0.05,
        reverse_only=True,
        max_tilt=35.0,
    )
    settings.update(overrides)
    return LevelController(**settings)


def test_level_body_commands_nothing():
    command = controller().update(roll=0.5, pitch=-1.2)
    assert command.level is True
    assert command.active() is False
    assert command.raised_wheel == 0


def test_front_left_on_obstacle_reverses_only_that_wheel():
    # 앞이 들리고(pitch+) 왼쪽이 들리면(roll+) 앞-왼쪽(ID2)이 장애물 위
    command = controller().update(roll=8.0, pitch=8.0)

    assert command.level is False
    assert command.raised_wheel == 2
    assert WHEEL_NAMES[2] == "front-left"

    # ID2 만 후진, 나머지는 정지 (reverse_only)
    assert command.velocities[2] < 0.0
    assert command.velocities[1] == 0.0
    assert command.velocities[4] == 0.0
    assert command.velocities[3] == 0.0


def test_each_corner_maps_to_its_own_wheel():
    cases = {
        1: (-8.0, +8.0),   # 앞 들림 + 오른쪽 들림 -> front-right
        2: (+8.0, +8.0),   # 앞 들림 + 왼쪽 들림   -> front-left
        3: (-8.0, -8.0),   # 뒤 들림 + 오른쪽 들림 -> rear-right
        4: (+8.0, -8.0),   # 뒤 들림 + 왼쪽 들림   -> rear-left
    }
    for expected_wheel, (roll, pitch) in cases.items():
        command = controller().update(roll=roll, pitch=pitch)
        assert command.raised_wheel == expected_wheel
        assert command.velocities[expected_wheel] < 0.0


def test_speed_grows_with_tilt_but_is_capped():
    small = controller().update(roll=0.0, pitch=6.0)
    large = controller().update(roll=0.0, pitch=20.0)
    assert abs(large.velocities[2]) > abs(small.velocities[2])

    capped = controller(max_velocity=0.3).update(roll=0.0, pitch=30.0)
    assert all(abs(v) <= 0.3 + 1e-9 for v in capped.velocities.values())


def test_excessive_tilt_stops_instead_of_driving():
    command = controller(max_tilt=20.0).update(roll=0.0, pitch=25.0)
    assert command.active() is False
    assert "max_tilt" in command.reason


def test_reverse_only_false_drives_lowered_wheels_forward():
    command = controller(reverse_only=False).update(roll=0.0, pitch=10.0)
    # 앞이 들렸으므로 앞바퀴는 후진, 뒷바퀴는 전진
    assert command.velocities[1] < 0.0 and command.velocities[2] < 0.0
    assert command.velocities[3] > 0.0 and command.velocities[4] > 0.0


def test_offsets_define_the_level_reference():
    # 장착 오차로 수평일 때 pitch 가 10도로 읽히는 경우
    policy = controller(pitch_offset=10.0)
    assert policy.update(roll=0.0, pitch=10.0).level is True
    assert policy.update(roll=0.0, pitch=0.0).level is False


def test_sign_flags_invert_the_convention():
    normal = controller().update(roll=0.0, pitch=8.0)
    flipped = controller(pitch_sign=-1.0).update(roll=0.0, pitch=8.0)
    assert normal.raised_wheel in (1, 2)      # 앞쪽이 들린 것으로 해석
    assert flipped.raised_wheel in (3, 4)     # 부호를 뒤집으면 뒤쪽


def robot_controller(**overrides):
    """이 로봇의 실제 설정: 측정된 장착 오차 + 데드존 5도."""
    return controller(
        roll_offset=ROBOT_ROLL_OFFSET, pitch_offset=ROBOT_PITCH_OFFSET, **overrides
    )


def test_measured_resting_attitude_is_treated_as_level():
    # 평지에서 실제로 읽히는 값이 그대로 수평이어야 한다
    command = robot_controller().update(
        roll=ROBOT_ROLL_OFFSET, pitch=ROBOT_PITCH_OFFSET
    )
    assert command.level is True
    assert command.active() is False


def test_deadzone_is_five_degrees_around_the_measured_level():
    policy = robot_controller()

    # 기준에서 ±5도 안쪽은 전부 무반응
    for droll, dpitch in [(4.9, 0.0), (-4.9, 0.0), (0.0, 4.9), (0.0, -4.9),
                          (4.0, 4.0), (-4.0, -4.0)]:
        command = policy.update(
            roll=ROBOT_ROLL_OFFSET + droll, pitch=ROBOT_PITCH_OFFSET + dpitch
        )
        assert command.level is True, f"{droll:+.1f}, {dpitch:+.1f} 에서 반응함"
        assert command.active() is False

    # 5도를 넘으면 반응
    command = policy.update(roll=ROBOT_ROLL_OFFSET, pitch=ROBOT_PITCH_OFFSET + 5.5)
    assert command.level is False
    assert command.active() is True


def test_offset_ignored_wheel_choice_still_correct():
    # 오프셋이 있어도 들린 모서리 판정은 그대로여야 한다
    command = robot_controller().update(
        roll=ROBOT_ROLL_OFFSET + 8.0, pitch=ROBOT_PITCH_OFFSET + 8.0
    )
    assert command.raised_wheel == 2          # front-left
    assert command.velocities[2] < 0.0


# 이 로봇에서 네 모서리를 하나씩 손으로 들어올리며 기록한 실측값.
# (offset 보정 후의 roll, pitch) -> 실제로 들어올린 바퀴 ID
MEASURED_CORNERS = [
    ((-5.8, +6.7), 2),   # 앞-왼쪽
    ((-4.7, -7.7), 1),   # 앞-오른쪽
    ((+5.5, +7.2), 4),   # 뒤-왼쪽
    ((+2.1, -7.9), 3),   # 뒤-오른쪽
]

# IMU 가 바디에 수직축으로 90도 돌아간 채로 장착되어 있다
ROBOT_MOUNT_YAW = 90.0


def test_mount_yaw_matches_the_measured_corners():
    """장착 회전각을 넣으면 실측 네 모서리가 모두 올바르게 판정되어야 한다."""
    policy = controller(mount_yaw_deg=ROBOT_MOUNT_YAW)
    for (roll, pitch), expected_wheel in MEASURED_CORNERS:
        command = policy.update(roll=roll, pitch=pitch)
        assert command.raised_wheel == expected_wheel, (
            f"roll={roll}, pitch={pitch} -> {command.raised_wheel} "
            f"(기대: {expected_wheel})"
        )
        assert command.velocities[expected_wheel] < 0.0


def test_without_mount_yaw_the_measured_corners_are_wrong():
    """회전각을 넣지 않으면 판정이 어긋난다 - 이것이 실제로 겪은 증상이다."""
    policy = controller(mount_yaw_deg=0.0)
    wrong = 0
    for (roll, pitch), expected_wheel in MEASURED_CORNERS:
        if policy.update(roll=roll, pitch=pitch).raised_wheel != expected_wheel:
            wrong += 1
    assert wrong == len(MEASURED_CORNERS)


def test_mount_yaw_ninety_swaps_the_axes():
    policy = controller(mount_yaw_deg=90.0)
    # roll' = pitch,  pitch' = -roll
    roll_body, pitch_body = policy.body_tilt(roll=3.0, pitch=8.0)
    assert abs(roll_body - 8.0) < 1e-9
    assert abs(pitch_body + 3.0) < 1e-9


def test_mount_yaw_does_not_change_tilt_magnitude_on_axis():
    policy = controller(mount_yaw_deg=90.0)
    # 한 축으로만 기울면 회전해도 크기는 같고 축만 바뀐다
    roll_body, pitch_body = policy.body_tilt(roll=0.0, pitch=10.0)
    assert abs(roll_body) == 10.0
    assert abs(pitch_body) < 1e-9


def test_tiny_commands_are_suppressed():
    # 임계값은 넘었지만 gain 이 작아 속도가 min_velocity 미만이면 0
    command = controller(gain=0.001, level_threshold=1.0).update(roll=0.0, pitch=4.0)
    assert command.active() is False
