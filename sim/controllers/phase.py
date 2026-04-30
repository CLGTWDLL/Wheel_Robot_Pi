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
    FAIL = "fail"


@dataclass(frozen=True)
class JumpPhaseParams:
    crouch_duration: float = 0.15
    extend_duration: float = 0.12
    flight_timeout: float = 0.5
    recover_duration: float = 0.2
    min_safe_height: float = 0.0


class JumpPhaseMachine:
    def __init__(self, params: JumpPhaseParams | None = None) -> None:
        self.params = params or JumpPhaseParams()
        self.phase = JumpPhase.STAND
        self.time_in_phase = 0.0

    def start_jump(self) -> None:
        self._set_phase(JumpPhase.CROUCH)

    def update(self, *, dt: float, base_height: float, vertical_velocity: float, contact_count: int) -> JumpPhase:
        if base_height < self.params.min_safe_height:
            self._set_phase(JumpPhase.FAIL)
            return self.phase

        self.time_in_phase += dt
        if self.phase == JumpPhase.CROUCH and self.time_in_phase >= self.params.crouch_duration:
            self._set_phase(JumpPhase.EXTEND)
        elif self.phase == JumpPhase.EXTEND and self.time_in_phase >= self.params.extend_duration:
            self._set_phase(JumpPhase.FLIGHT)
        elif self.phase == JumpPhase.FLIGHT:
            if contact_count > 0 and vertical_velocity <= 0.0:
                self._set_phase(JumpPhase.LAND)
            elif self.time_in_phase >= self.params.flight_timeout:
                self._set_phase(JumpPhase.LAND)
        elif self.phase == JumpPhase.LAND and self.time_in_phase >= self.params.recover_duration:
            self._set_phase(JumpPhase.RECOVER)
        elif self.phase == JumpPhase.RECOVER and self.time_in_phase >= self.params.recover_duration:
            self._set_phase(JumpPhase.STAND)
        return self.phase

    def _set_phase(self, phase: JumpPhase) -> None:
        if self.phase != phase:
            self.phase = phase
            self.time_in_phase = 0.0
