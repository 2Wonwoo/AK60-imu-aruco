"""CubeMars AK60-6 V3 velocity-command packing helpers."""

from dataclasses import dataclass


CAN_EFF_FLAG = 0x80000000


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def float_to_uint(value: float, minimum: float, maximum: float, bits: int) -> int:
    """Map a physical value to an unsigned protocol field."""
    value = clamp(value, minimum, maximum)
    scale = ((1 << bits) - 1) / (maximum - minimum)
    return int(round((value - minimum) * scale))


@dataclass(frozen=True)
class Ak60V3Protocol:
    """Protocol values confirmed against the robot's working CAN frames."""

    velocity_min: float = -28.0
    velocity_max: float = 28.0
    position_min: float = -12.5
    position_max: float = 12.5
    kp_min: float = 0.0
    kp_max: float = 500.0
    kd_min: float = 0.0
    kd_max: float = 5.0
    torque_min: float = -15.0
    torque_max: float = 15.0

    @staticmethod
    def control_id(motor_id: int) -> int:
        if motor_id not in (1, 2, 3, 4):
            raise ValueError("motor_id must be one of 1, 2, 3, 4")
        return 0x800 + motor_id

    def velocity_command(self, velocity: float, kd: float = 1.0) -> bytes:
        """Pack [Kp:12][Kd:12][P:16][V:12][T:12] into eight bytes."""
        kp_int = float_to_uint(0.0, self.kp_min, self.kp_max, 12)
        kd_int = float_to_uint(kd, self.kd_min, self.kd_max, 12)
        position_int = float_to_uint(
            0.0, self.position_min, self.position_max, 16
        )
        # The V3 controller frames verified on this robot quantize velocity
        # symmetrically around the exact neutral code 0x800. Keeping the
        # magnitude truncation symmetric reproduces both +2 (0x892) and
        # -2 rad/s (0x76E), unlike a generic endpoint mapping.
        velocity = clamp(velocity, self.velocity_min, self.velocity_max)
        velocity_scale = ((1 << 12) - 1) / (
            self.velocity_max - self.velocity_min
        )
        velocity_delta = int(abs(velocity) * velocity_scale)
        velocity_int = 0x800 + (
            velocity_delta if velocity >= 0.0 else -velocity_delta
        )
        torque_int = float_to_uint(
            0.0, self.torque_min, self.torque_max, 12
        )

        packed = (
            (kp_int << 52)
            | (kd_int << 40)
            | (position_int << 24)
            | (velocity_int << 12)
            | torque_int
        )
        return packed.to_bytes(8, byteorder="big")
