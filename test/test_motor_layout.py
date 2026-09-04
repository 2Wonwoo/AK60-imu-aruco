from ak60_driver.four_wheel_drive_node import FourWheelDriveNode


def test_motor_position_and_forward_direction():
    # ID 1/2 are front; ID 3/4 are rear. Right-side motors use positive
    # controller velocity for vehicle-forward, left-side motors use negative.
    assert FourWheelDriveNode.MOTOR_DIRECTION == {
        1: 1.0,
        2: -1.0,
        3: 1.0,
        4: -1.0,
    }
