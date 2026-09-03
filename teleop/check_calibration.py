"""Judge a calibration before trusting it.

    python teleop/check_calibration.py                    # the active one
    python teleop/check_calibration.py path/to/file.json

`lerobot-calibrate` happily records a range of 8..4093 when a joint crosses its stop and
the encoder wraps (4095 -> 0): both extremes look legitimate to the recorder. The result
is a scale that no longer matches the hardware, which is how a whole session gets lost.

Run this straight after any calibration.
"""

import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from teleop.config import CALIBRATION as CAL  # noqa: E402

ENCODER = 4096

# Travel measured by hand with torque off — teleop/limits/measured.md.
EXPECTED = {
    "shoulder_pan": (2400, 3000),
    "shoulder_lift": (2200, 2600),
    "elbow_flex": (2000, 2400),
    "wrist_flex": (2200, 2500),
    "wrist_roll": (3300, 4096),
    "gripper": (1400, 1700),
}


def main() -> None:
    path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else CAL
    if not path.exists():
        raise SystemExit(f"No calibration at {path}\n"
                         "Run the panel and press C to calibrate the arm.")
    cal = json.loads(path.read_text())
    print(f"{path}\n")
    print(f"{'joint':16}{'range':>14}{'span':>7}{'expected':>13}{'offset':>9}   verdict")

    problems = []
    for m, c in cal.items():
        lo, hi = c["range_min"], c["range_max"]
        span = hi - lo
        elo, ehi = EXPECTED.get(m, (0, ENCODER))
        notes = []
        # wrist_roll really does travel almost a full turn — its stops are at the
        # encoder's own ends (measured raw 8..4086). For every other joint a full span
        # means the count wrapped.
        if span > ENCODER * 0.97 and m != "wrist_roll":
            notes.append("WRAPPED: span covers the whole encoder")
        elif not (elo * 0.75 <= span <= ehi * 1.25):
            notes.append(f"span off expected {elo}-{ehi}")
        if (lo < 30 or hi > ENCODER - 30) and m != "wrist_roll":
            notes.append("range touches an encoder end")
        if notes:
            problems.append((m, notes))
        print(f"{m:16}{lo:6}-{hi:<7}{span:7}{elo:6}-{ehi:<6}{c['homing_offset']:9}   "
              f"{'; '.join(notes) if notes else 'ok'}")

    print()
    if problems:
        print(f"{len(problems)} joint(s) look wrong:")
        for m, notes in problems:
            print(f"  {m}: {'; '.join(notes)}")
        print("\nA wrapped range means the joint was pushed past its stop while recording.")
        print("Redo the calibration, stopping at the first resistance on every joint.")
    else:
        print("All joints plausible.")


if __name__ == "__main__":
    main()
