from ak60_driver.imu_level_control import LevelController, LevelSupervisor

LEVEL = (0.0, 0.0)          # 수평
TILTED = (0.0, 12.0)        # 앞이 들림 -> 앞바퀴가 장애물 위


def supervisor(**overrides):
    settings = dict(
        level_threshold=2.5,
        gain=0.08,
        max_velocity=0.6,
        min_velocity=0.05,
        reverse_only=True,
        max_tilt=35.0,
    )
    settings.update(overrides)
    return LevelSupervisor(controller=LevelController(**settings), level_hold_sec=0.3)


def test_level_body_yields_control_to_the_follower():
    command = supervisor().update(*LEVEL, now=0.0)
    assert command.state == "FOLLOW"
    assert command.publish is False       # 아무것도 안 내보내야 마커 추종이 로봇을 몬다
    assert command.leveling() is False


def test_tilt_stops_everything_first():
    policy = supervisor()
    command = policy.update(*TILTED, now=0.0)

    assert command.state == "STOP"
    assert command.publish is True
    assert all(v == 0.0 for v in command.velocities.values())   # 전부 정지
    assert command.leveling() is True


def test_after_the_stop_it_drives_the_raised_wheels_back():
    policy = supervisor()
    policy.update(*TILTED, now=0.0)                    # STOP
    command = policy.update(*TILTED, now=0.05)

    assert command.state == "LEVELING"
    assert command.publish is True
    assert command.velocities[1] < 0.0 and command.velocities[2] < 0.0   # 앞바퀴 후진
    assert command.velocities[3] == 0.0 and command.velocities[4] == 0.0


def test_control_returns_only_after_the_body_stays_level():
    policy = supervisor()
    policy.update(*TILTED, now=0.0)
    policy.update(*TILTED, now=0.05)

    # 수평이 되었지만 아직 유지 시간이 지나지 않았다
    holding = policy.update(*LEVEL, now=1.0)
    assert holding.state == "LEVELING"
    assert holding.publish is True
    assert all(v == 0.0 for v in holding.velocities.values())

    # 유지 시간이 지나면 주도권을 돌려준다
    released = policy.update(*LEVEL, now=1.4)
    assert released.state == "RELEASE"
    assert released.publish is True                     # 마지막으로 한 번 세운다
    assert released.leveling() is False

    # 그다음부터는 다시 마커 추종
    following = policy.update(*LEVEL, now=1.5)
    assert following.state == "FOLLOW"
    assert following.publish is False


def test_tilt_during_the_hold_window_restarts_levelling():
    policy = supervisor()
    policy.update(*TILTED, now=0.0)
    policy.update(*LEVEL, now=0.1)          # 유지 시간 측정 시작
    again = policy.update(*TILTED, now=0.2)  # 다시 기울어짐

    assert again.state == "LEVELING"
    assert again.velocities[1] < 0.0

    # 유지 시간이 초기화되었으므로 0.2초만으로는 복귀하지 않는다
    assert policy.update(*LEVEL, now=0.4).state == "LEVELING"


def test_repeated_obstacles_are_handled_independently():
    policy = supervisor()
    for cycle in range(3):
        base = cycle * 10.0
        assert policy.update(*TILTED, now=base).state == "STOP"
        assert policy.update(*TILTED, now=base + 0.05).state == "LEVELING"
        policy.update(*LEVEL, now=base + 1.0)
        assert policy.update(*LEVEL, now=base + 1.4).state == "RELEASE"
        assert policy.update(*LEVEL, now=base + 1.5).state == "FOLLOW"
