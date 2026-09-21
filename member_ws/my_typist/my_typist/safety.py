"""Arm-control safety checks: an input-staleness watchdog and the press-safety gate.

Kept separate from typist.py so perception/planning (target computation) and
arm-control safety are two clearly distinct pieces of code, per the
"separation between perception, planning and arm control" requirement.
"""

from typing import Optional, Sequence


class StalenessWatchdog:
    """Tracks whether a periodic input (e.g. /joint_states) is still arriving on time."""

    def __init__(self, timeout: float) -> None:
        self.timeout = timeout
        self._last_seen: Optional[float] = None

    def touch(self, now: float) -> None:
        self._last_seen = now

    def is_stale(self, now: float) -> bool:
        if self._last_seen is None:
            return True
        return (now - self._last_seen) > self.timeout


def is_safe_to_press(velocities: Sequence[float], velocity_tol: float = 1e-2) -> bool:
    """Safe to press only once every joint's commanded velocity has settled
    below `velocity_tol` - i.e. the PID loop has converged and the arm is
    holding its target pose rather than still moving toward it."""
    return all(abs(v) < velocity_tol for v in velocities)


def clamp_velocities(velocities: Sequence[float], v_max: Sequence[float]) -> list:
    """Clip each commanded joint velocity to its documented hard limit
    (docs/INTERFACES.md section 3.1) before it is published. Only ever
    reduces a magnitude that already exceeds the limit; a legal command
    passes through unchanged."""
    return [max(-limit, min(limit, v)) for v, limit in zip(velocities, v_max)]
