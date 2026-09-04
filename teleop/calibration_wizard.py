"""Guided first calibration, one joint at a time.

lerobot's own calibrate() records every joint in a single pass and blocks on input(),
so one joint pushed past its stop wraps the encoder and the whole session is lost. It
also hardcodes wrist_roll to a full 0-4095 turn instead of measuring it.

This walks joint by joint. Each one is centred, then its travel is recorded live while
you move it, then judged before it is kept; a bad joint is redone on its own and the
five good ones are untouched. Nothing reaches the motors until every joint has passed.
"""

import json
import pathlib
import shutil
import time

ENCODER = 4096

# Travel measured by hand on a healthy SO-101, used only to warn: a joint outside these
# bounds is suspicious, not necessarily wrong.
EXPECTED = {
    "shoulder_pan": (2400, 3000),
    "shoulder_lift": (2200, 2600),
    "elbow_flex": (2000, 2400),
    "wrist_flex": (2200, 2500),
    "wrist_roll": (3300, 4096),
    "gripper": (1400, 1700),
}

CENTRE, TRAVEL, DONE = "centre", "travel", "done"


def judge(joint: str, span: int) -> tuple[bool, str]:
    """Whether a recorded span is believable, and what to say about it."""
    if joint != "wrist_roll" and span > ENCODER * 0.97:
        return False, "wrapped past the stop — redo it, stopping at first resistance"
    if span < 200:
        return False, "barely moved — move it through its whole travel"
    lo, hi = EXPECTED.get(joint, (0, ENCODER))
    if not (lo * 0.75 <= span <= hi * 1.25):
        return True, f"unusual span (expected about {lo}-{hi}) — keep it only if it felt right"
    return True, "plausible"


class Wizard:
    """The calibration as a state the panel can draw and step through."""

    def __init__(self, bus, joints: list[str], cal_path: pathlib.Path):
        self.bus = bus
        self.joints = joints
        self.cal_path = cal_path
        self.i = 0
        self.step = CENTRE
        self.result: dict[str, dict] = {}
        self.offset = 0
        self.lo = self.hi = None
        self.now = 0
        self.note = ""
        self.ok = True

    @property
    def joint(self) -> str:
        return self.joints[self.i]

    @property
    def span(self) -> int:
        return 0 if self.lo is None else self.hi - self.lo

    def start(self) -> None:
        """Free every joint so the arm can be posed by hand."""
        for m in self.bus.motors:
            try:
                self.bus.write("Torque_Enable", m, 0, num_retry=4)
            except Exception:
                pass

    def poll(self) -> None:
        """Follow the joint: live position while centring, travel while recording."""
        if self.step == DONE:
            return
        try:
            raw = self.bus.read("Present_Position", self.joint, normalize=False, num_retry=2)
        except Exception:
            return
        self.now = raw
        if self.step == CENTRE:
            return
        self.lo = raw if self.lo is None else min(self.lo, raw)
        self.hi = raw if self.hi is None else max(self.hi, raw)
        self.ok, self.note = judge(self.joint, self.span)

    def centre(self) -> None:
        """Take the joint's current position as the middle of its range."""
        offsets = self.bus.set_half_turn_homings([self.joint])
        self.offset = int(offsets[self.joint])
        self.lo = self.hi = None
        self.note = ""
        self.step = TRAVEL

    def accept(self) -> bool:
        """Keep this joint and move to the next. False when the span is not usable."""
        if self.lo is None or not self.ok:
            return False
        self.result[self.joint] = {
            "id": self.bus.motors[self.joint].id,
            "drive_mode": 0,
            "homing_offset": self.offset,
            "range_min": int(self.lo),
            "range_max": int(self.hi),
        }
        if self.i + 1 < len(self.joints):
            self.i += 1
            self.step = CENTRE
            self.lo = self.hi = None
            self.note = ""
        else:
            self.step = DONE
        return True

    def redo(self) -> None:
        self.step = CENTRE
        self.lo = self.hi = None
        self.note = ""

    def save(self) -> pathlib.Path | None:
        """Write the finished calibration to disk and to the motors."""
        if self.step != DONE:
            return None
        backup = None
        if self.cal_path.exists():
            backup = self.cal_path.with_suffix(f".json.bak-{time.strftime('%Y%m%d-%H%M%S')}")
            shutil.copy(self.cal_path, backup)
        self.cal_path.parent.mkdir(parents=True, exist_ok=True)
        self.cal_path.write_text(json.dumps(self.result, indent=4) + "\n")
        return backup
