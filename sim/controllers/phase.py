from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class JumpPhase(str, Enum):
    STAND = "stand"
    CROUCH = "crouch"
    EXTEND = "extend"
    FLIGHT = "flight"
    LAND = "land"
    RECOVER = "recover"
    FALLEN = "fallen"


@dataclass(frozen=True)
class JumpPhaseParams:
    crouch_duration: float = 0.20
    crouch_target_height: float = 0.09
    crouch_height_tolerance: float = 0.005
    extend_duration: float = 0.05
    extend_timeout: float = 0.6
    flight_timeout: float = 0.5
    recover_duration: float = 0.2
    pitch_fallen_threshold: float = 1.0


class JumpPhaseMachine:
    def __init__(self, params: JumpPhaseParams | None = None) -> None:
        self.params = params or JumpPhaseParams()
        self.phase = JumpPhase.STAND
        self.time_in_phase = 0.0

    def start_jump(self) -> None:
        if self.phase == JumpPhase.STAND:
            self._set_phase(JumpPhase.CROUCH)

    def update(
        self,
        *,
        dt: float,
        contact_count: int,
        leg_height: float | None = None,
        vz: float | None = None,
        pitch: float | None = None,
    ) -> JumpPhase:
        # FALLEN 是终态：进入后保持，由控制器输出零控制让机器人原地不动，避免乱跑。
        if self.phase == JumpPhase.FALLEN:
            return self.phase

        self.time_in_phase += dt

        if pitch is not None and abs(float(pitch)) > self.params.pitch_fallen_threshold:
            self._set_phase(JumpPhase.FALLEN)
            return self.phase

        if self.phase == JumpPhase.CROUCH and (
            self.time_in_phase >= self.params.crouch_duration
            or (
                leg_height is not None
                and leg_height <= self.params.crouch_target_height + self.params.crouch_height_tolerance
            )
        ):
            self._set_phase(JumpPhase.EXTEND)
        elif self.phase == JumpPhase.EXTEND and self.time_in_phase >= self.params.extend_duration and contact_count == 0:
            self._set_phase(JumpPhase.FLIGHT)
        elif self.phase == JumpPhase.EXTEND and self.time_in_phase >= self.params.extend_timeout:
            self._set_phase(JumpPhase.LAND)
        elif self.phase == JumpPhase.FLIGHT:
            if contact_count > 0 and (vz is None or vz <= 0.0):
                self._set_phase(JumpPhase.LAND)
            elif self.time_in_phase >= self.params.flight_timeout:
                self._set_phase(JumpPhase.LAND)
        elif (
            self.phase == JumpPhase.LAND
            and self.time_in_phase >= self.params.recover_duration
            and contact_count > 0
            and (vz is None or abs(vz) < 0.2)
        ):
            self._set_phase(JumpPhase.RECOVER)
        elif (
            self.phase == JumpPhase.RECOVER
            and self.time_in_phase >= self.params.recover_duration
            and contact_count > 0
            and (vz is None or abs(vz) < 0.2)
        ):
            self._set_phase(JumpPhase.STAND)
        return self.phase

    def _set_phase(self, phase: JumpPhase) -> None:
        if self.phase != phase:
            self.phase = phase
            self.time_in_phase = 0.0
