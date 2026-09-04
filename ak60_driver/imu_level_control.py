"""Pure control policy for levelling the body using IMU roll/pitch.

When one wheel climbs an obstacle the body tilts.  Combining roll and pitch
tells which *corner* is raised, so that wheel can be driven backwards until the
body is parallel to the ground again.

Wheel layout seen from above (x forward, y left)::

        front
    ID2 ●────● ID1
        │    │
    ID4 ●────● ID3
        rear

Each wheel contributes to the tilt with a fixed sign, so the raised corner is
found without any geometry beyond "front/rear" and "left/right":

    elevation(i) = pitch_up * front_sign(i) + roll_left_up * left_sign(i)

``pitch_up`` is positive when the nose is up and ``roll_left_up`` is positive
when the left side is up.  For ID2 (front left) both signs are ``+1``, so a
raised front-left corner produces the largest elevation of the four.

Keeping this module free of ROS and pyserial makes the safety logic testable
without a robot attached.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Tuple

# wheel id -> (front_sign, left_sign)
WHEEL_SIGNS: Dict[int, Tuple[float, float]] = {
    1: (+1.0, -1.0),  # front right
    2: (+1.0, +1.0),  # front left
    3: (-1.0, -1.0),  # rear right
    4: (-1.0, +1.0),  # rear left
}

WHEEL_NAMES: Dict[int, str] = {
    1: "front-right",
    2: "front-left",
    3: "rear-right",
    4: "rear-left",
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class LevelCommand:
    """One control cycle's decision."""

    velocities: Dict[int, float]      # wheel id -> rad/s, forward positive
    elevations: Dict[int, float]      # wheel id -> degrees, positive = raised
    raised_wheel: int                 # most raised wheel id (0 when level)
    level: bool                       # within the deadzone
    reason: str

    def active(self) -> bool:
        return any(abs(v) > 1e-9 for v in self.velocities.values())


@dataclass
class LevelController:
    """Drive raised wheels backwards until the body is level again.

    All angles are in degrees; velocities are wheel rad/s with forward positive
    (the driver node applies each motor's own direction sign).
    """

    level_threshold: float = 5.0      # 이 각도 아래면 수평으로 본다 (데드존)
    gain: float = 0.08                # rad/s per degree of elevation
    max_velocity: float = 0.6         # rad/s, 안전을 위해 낮게 잡는다
    min_velocity: float = 0.05        # 이보다 작은 명령은 무시 (모터 떨림 방지)
    reverse_only: bool = True         # 들린 바퀴만 후진. False 면 내려간 바퀴는 전진
    roll_sign: float = 1.0            # IMU 부호 규약 보정 (+1 이면 roll>0 = 왼쪽 위)
    pitch_sign: float = 1.0           # +1 이면 pitch>0 = 앞쪽 위
    roll_offset: float = 0.0          # 수평일 때의 기준값 (tare)
    pitch_offset: float = 0.0
    mount_yaw_deg: float = 0.0        # IMU 가 바디 대비 수직축으로 돌아간 각도
    max_tilt: float = 35.0            # 이보다 크면 비정상으로 보고 정지

    _stopped: Dict[int, float] = field(
        default_factory=lambda: {i: 0.0 for i in WHEEL_SIGNS}, init=False
    )

    def body_tilt(self, roll: float, pitch: float) -> Tuple[float, float]:
        """센서 값을 바디 기준 (왼쪽 들림, 앞쪽 들림) 으로 변환한다.

        IMU 를 바디에 90도 돌려 붙이면 센서의 roll 축이 바디의 pitch 축이 된다.
        기울기를 2차원 벡터로 보고 장착 회전각만큼 되돌린다.
        """
        roll_up = (roll - self.roll_offset) * self.roll_sign
        pitch_up = (pitch - self.pitch_offset) * self.pitch_sign

        if self.mount_yaw_deg:
            psi = math.radians(self.mount_yaw_deg)
            cos_psi, sin_psi = math.cos(psi), math.sin(psi)
            roll_up, pitch_up = (
                roll_up * cos_psi + pitch_up * sin_psi,
                -roll_up * sin_psi + pitch_up * cos_psi,
            )
        return roll_up, pitch_up

    def elevations(self, roll: float, pitch: float) -> Dict[int, float]:
        """바퀴별 들림 정도(도). 양수면 그 모서리가 올라간 것."""
        roll_up, pitch_up = self.body_tilt(roll, pitch)
        return {
            wheel: pitch_up * front + roll_up * left
            for wheel, (front, left) in WHEEL_SIGNS.items()
        }

    def update(self, roll: float, pitch: float) -> LevelCommand:
        elevations = self.elevations(roll, pitch)
        raised = max(elevations, key=lambda w: elevations[w])

        roll_up, pitch_up = self.body_tilt(roll, pitch)
        tilt = max(abs(roll_up), abs(pitch_up))

        if tilt > self.max_tilt:
            return LevelCommand(
                velocities=dict(self._stopped),
                elevations=elevations,
                raised_wheel=raised,
                level=False,
                reason=f"tilt {tilt:.1f} deg exceeds max_tilt {self.max_tilt:.1f}",
            )

        if tilt <= self.level_threshold:
            return LevelCommand(
                velocities=dict(self._stopped),
                elevations=elevations,
                raised_wheel=0,
                level=True,
                reason=f"level (tilt {tilt:.1f} deg)",
            )

        velocities = {}
        for wheel, elevation in elevations.items():
            effective = max(elevation, 0.0) if self.reverse_only else elevation
            # 들린 바퀴는 뒤로 굴려 장애물에서 내려오게 한다
            speed = clamp(-self.gain * effective, -self.max_velocity, self.max_velocity)
            velocities[wheel] = 0.0 if abs(speed) < self.min_velocity else speed

        return LevelCommand(
            velocities=velocities,
            elevations=elevations,
            raised_wheel=raised,
            level=False,
            reason=(
                f"{WHEEL_NAMES[raised]} raised {elevations[raised]:+.1f} deg "
                f"(roll {roll_up:+.1f}, pitch {pitch_up:+.1f})"
            ),
        )

    def stop(self) -> LevelCommand:
        return LevelCommand(
            velocities=dict(self._stopped),
            elevations={wheel: 0.0 for wheel in WHEEL_SIGNS},
            raised_wheel=0,
            level=False,
            reason="stop",
        )
