"""Separate CSV log for Tanya's perception and evaluation contribution."""

import csv
import os


class VisionLog:
    """Append timestamped perception, planning, and result records."""

    def __init__(self, path="/ws/perception_log.csv"):
        self.path = path
        is_new = not os.path.exists(path)
        self._file = open(path, "a", newline="")
        self._writer = csv.writer(self._file)
        if is_new:
            self._writer.writerow(["time", "event", "detail", "values"])

    def write(self, timestamp, event, detail, values=""):
        """Write and immediately flush one record."""
        self._writer.writerow([f"{timestamp:.6f}", event, detail, values])
        self._file.flush()

    def close(self):
        """Close the CSV file."""
        self._file.close()
