"""Calibration-derived scales and the numbers that bound movement.

Both the hardware layer and the drawing code need these, so they live apart from either.
`rebuild_scales()` re-reads the file after a calibration changes it on disk.
"""

import json

from teleop.config import CALIBRATION as CAL_PATH

ENCODER = 4096
MARGIN = 3.0

FPS = 30
SPEEDS = [2.0, 5.0, 10.0, 20.0, 35.0, 55.0]

# At P=36 a healthy lift peaks near 850, so the old 200/350 thresholds fired on normal
# movement. These sit above working load and below the 1000 ceiling at which the motor
# trips its own overload protection.
LOAD_WARN = 700
LOAD_STOP = 950

# lerobot's configure() writes P_Coefficient=16, half the STS3215 factory default of 32
# (huggingface/lerobot#3400). At 16 the loop stops pushing well before the available
# torque is used: measured on this arm, shoulder_lift climbed 2.9 deg at P=16 and
# 36.7 deg at P=36. The step is sharp between 34 and 36; 38 gains nothing and adds load.
POSITION_P = 36

# How far the commanded position may run ahead of the joint's real one. Without this the
# target advances every frame while the joint stalls, winding the error up without limit
# and leaving the motor straining at a goal tens of degrees away.
MAX_LAG = 8.0

LOAD_EVERY = 3
TEMP_EVERY = 30      # temperature and voltage move slowly; once a second is plenty
STALE_AFTER = 2.0    # seconds without a good read before a joint is shown unresponsive

DESC = {
    "shoulder_pan": "base rotation, left / right",
    "shoulder_lift": "shoulder, raise and lower",
    "elbow_flex": "elbow, bends and extends",
    "wrist_flex": "wrist, up and down",
    "wrist_roll": "wrist, axial rotation",
    "gripper": "gripper, open and close",
}

TRAVEL: dict[str, tuple[float, float]] = {}
RAW_RANGE: dict[str, tuple[int, int]] = {}
SCALE: dict[str, tuple[float, float]] = {}


def load_calibration() -> dict:
    """The calibration file, or an empty dict when there is none.

    The panel has to open and report what is wrong; dying at import time on a missing
    file is the failure it exists to show.
    """
    try:
        return json.loads(CAL_PATH.read_text())
    except Exception:
        return {}


def rebuild_scales() -> None:
    """Re-read the calibration into the display scales after it changes on disk.

    With use_degrees=True every joint but the gripper is normalised as DEGREES from the
    centre of its calibrated range, so the endpoints are not +/-100 — they are however
    many degrees that joint spans. Treating them as percentages made healthy joints look
    out of scale.
    """
    TRAVEL.clear()
    RAW_RANGE.clear()
    SCALE.clear()
    for name, c in load_calibration().items():
        RAW_RANGE[name] = (c["range_min"], c["range_max"])
        if name == "gripper":
            TRAVEL[name] = (MARGIN, 100.0 - MARGIN)
            SCALE[name] = (0.0, 100.0)
        else:
            half = (c["range_max"] - c["range_min"]) / 2 * 360 / 4095
            TRAVEL[name] = (-half + MARGIN, half - MARGIN)
            SCALE[name] = (-half, half)


rebuild_scales()
