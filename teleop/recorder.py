"""Diagnostic logger — every frame of every session, to CSV.

Format A: raw telemetry for understanding the arm. One row per frame, one column
group per motor. Deliberately convertible to a LeRobotDataset later: fixed rate,
absolute timestamps, and the same state/action split the dataset format uses.

    from teleop.recorder import Recorder
    rec = Recorder(robot, tag="panel")
    rec.log(action=target, extra={"selected": name, "speed": 10.0})
    rec.close()

Files land in data/logs/<date>_<time>_<tag>.csv with a .meta.json beside them.
"""

import csv
import json
import platform
import time
from pathlib import Path

LOG_DIR = Path("data/logs")

# Registers read every frame. Voltage and temperature are cheap and have already
# proved diagnostic once (a transient voltage dip, a joint heating under strain).
PER_MOTOR = ["Present_Position", "Present_Load", "Present_Temperature", "Present_Voltage"]
SHORT = {"Present_Position": "pos_raw", "Present_Load": "load",
         "Present_Temperature": "temp", "Present_Voltage": "volt"}


class Recorder:
    def __init__(self, robot, tag: str = "session", extra_fields: list[str] | None = None):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        self.robot = robot
        self.names = list(robot.bus.motors)
        self.t0 = time.time()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t0))
        self.path = LOG_DIR / f"{stamp}_{tag}.csv"
        self.extra_fields = extra_fields or []

        cols = ["t", "wall"]
        for n in self.names:
            cols += [f"{n}.norm", f"{n}.cmd"] + [f"{n}.{SHORT[r]}" for r in PER_MOTOR]
        cols += self.extra_fields
        self.cols = cols

        self.fh = self.path.open("w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(cols)
        self.rows = 0

        (self.path.with_suffix(".meta.json")).write_text(json.dumps({
            "started": stamp,
            "tag": tag,
            "motors": self.names,
            "calibration": {m: vars(c) if hasattr(c, "__dict__") else c
                            for m, c in robot.calibration.items()},
            "host": platform.platform(),
            "columns": cols,
        }, indent=2, default=str) + "\n")

    def log(self, action: dict[str, float] | None = None, extra: dict | None = None) -> None:
        """One row. Never raises: a dropped packet must not stop a session."""
        try:
            obs = self.robot.get_observation()
        except Exception:
            obs = {}
        row = [round(time.time() - self.t0, 4), round(time.time(), 3)]
        for n in self.names:
            row.append(obs.get(f"{n}.pos", ""))
            row.append("" if not action else action.get(n, action.get(f"{n}.pos", "")))
            for r in PER_MOTOR:
                try:
                    row.append(self.robot.bus.read(r, n, normalize=False, num_retry=1))
                except Exception:
                    row.append("")
        for f in self.extra_fields:
            row.append("" if not extra else extra.get(f, ""))
        self.w.writerow(row)
        self.rows += 1
        if self.rows % 150 == 0:
            self.fh.flush()

    def close(self) -> None:
        try:
            self.fh.flush()
            self.fh.close()
        except Exception:
            pass
        dur = time.time() - self.t0
        print(f"\nlog: {self.path}  ({self.rows} righe, {dur:.0f}s)")


class LightRecorder(Recorder):
    """Position and load only — one bus read per motor instead of four.

    Reading four registers per motor per frame costs ~24 reads at 30Hz, which starves
    the control loop. Use this in anything that also has to command the arm; the full
    Recorder is for standalone measurement runs.
    """

    def log(self, action: dict[str, float] | None = None, extra: dict | None = None) -> None:
        try:
            obs = self.robot.get_observation()
        except Exception:
            obs = {}
        row = [round(time.time() - self.t0, 4), round(time.time(), 3)]
        for n in self.names:
            row.append(obs.get(f"{n}.pos", ""))
            row.append("" if not action else action.get(n, action.get(f"{n}.pos", "")))
            row.append("")  # pos_raw: derivable from .norm plus the calibration in .meta
            try:
                row.append(self.robot.bus.read("Present_Load", n, normalize=False, num_retry=1))
            except Exception:
                row.append("")
            row += ["", ""]  # temp, volt sampled by the caller instead
        for f in self.extra_fields:
            row.append("" if not extra else extra.get(f, ""))
        self.w.writerow(row)
        self.rows += 1
        if self.rows % 150 == 0:
            self.fh.flush()
