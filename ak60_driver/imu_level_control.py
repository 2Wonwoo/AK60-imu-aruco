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
from typing import Dict, Optional, Tuple

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

    level_threshold: float = 2.5      # 이 각도 아래면 수평으로 본다 (데드존)
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


@dataclass(frozen=True)
class SupervisorCommand:
    """자세 복원이 주도권을 잡았는지, 무엇을 내보낼지."""

    velocities: Dict[int, float]
    state: str            # FOLLOW / STOP / LEVELING / RELEASE
    publish: bool         # False 면 아무것도 내보내지 않아 다른 동작에 양보한다
    reason: str

    def leveling(self) -> bool:
        return self.state in ("STOP", "LEVELING")


@dataclass
class LevelSupervisor:
    """자세 복원과 마커 추종 사이의 주도권을 관리한다.

    평소에는 아무것도 내보내지 않아 마커 추종이 로봇을 몰게 두고, 바디가
    기울면 즉시 전체를 정지시킨 뒤 자세가 돌아올 때까지 바퀴를 후진시킨다.
    수평이 ``level_hold_sec`` 동안 유지되면 주도권을 돌려준다 (임계값 근처에서
    두 동작이 번갈아 튀는 것을 막는다).
    """

    controller: LevelController
    level_hold_sec: float = 0.3

    _leveling: bool = field(default=False, init=False)
    _level_since: Optional[float] = field(default=None, init=False)

    def _stopped(self) -> Dict[int, float]:
        return {wheel: 0.0 for wheel in WHEEL_SIGNS}

    def update(self, roll: float, pitch: float, now: float) -> SupervisorCommand:
        command = self.controller.update(roll, pitch)

        if not self._leveling:
            if command.level:
                return SupervisorCommand(
                    velocities=self._stopped(),
                    state="FOLLOW",
                    publish=False,
                    reason=command.reason,
                )
            # 기울어졌다. 먼저 전부 세우고 다음 주기부터 복원에 들어간다.
            self._leveling = True
            self._level_since = None
            return SupervisorCommand(
                velocities=self._stopped(),
                state="STOP",
                publish=True,
                reason=f"tilt detected - stopping ({command.reason})",
            )

        if not command.level:
            self._level_since = None
            return SupervisorCommand(
                velocities=command.velocities,
                state="LEVELING",
                publish=True,
                reason=command.reason,
            )

        # 수평으로 돌아왔다. 잠시 유지되는지 확인한 뒤 주도권을 넘긴다.
        if self._level_since is None:
            self._level_since = now
        if now - self._level_since < self.level_hold_sec:
            return SupervisorCommand(
                velocities=self._stopped(),
                state="LEVELING",
                publish=True,
                reason=f"levelled, holding ({now - self._level_since:.1f}s)",
            )

        self._leveling = False
        self._level_since = None
        return SupervisorCommand(
            velocities=self._stopped(),
            state="RELEASE",
            publish=True,
            reason="levelled - handing control back",
        )
