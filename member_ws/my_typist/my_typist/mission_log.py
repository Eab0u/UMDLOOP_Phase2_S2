"""Exportable CSV log of arm-control commands and press decisions."""

import csv
import os
from typing import Sequence


class MissionLog:
    def __init__(self, path: str = "control_log.csv") -> None:
        self.path = path
        is_new = not os.path.exists(path)
        self._file = open(path, "a", newline="")
        self._writer = csv.writer(self._file)
        if is_new:
            self._writer.writerow(["time", "event", "state_or_key", "velocities"])

    def log_command(self, t: float, state: str, velocities: Sequence[float]) -> None:
        self._writer.writerow([t, "command", state, list(velocities)])
        self._file.flush()

    def log_press(self, t: float, key: str) -> None:
        self._writer.writerow([t, "press", key, ""])
        self._file.flush()

    def close(self) -> None:
        self._file.close()
