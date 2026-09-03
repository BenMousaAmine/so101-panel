"""Write the calibration file back into the motors.

    python teleop/restore_calibration.py

Unplugging the arm and moving it by hand leaves each motor homed on wherever it
happened to be sitting, so the offsets in EEPROM no longer match the file. The file is
the calibration that was measured and checked; the motors just hold whatever the last
power-up inferred. This pushes the file back onto the motors.

Torque is dropped first: rewriting an offset changes the position a motor believes it
is at, and a powered joint would snap to it. Support the arm before running this.

The current EEPROM values are archived to data/calibrations/ first, so the state before
the write is never lost.

`controller.py` calls mismatched_joints() at startup and restore() on confirmation, so
the same check runs whether you come through the panel or this script.
"""

import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402

from teleop.config import CALIBRATION as CAL  # noqa: E402
from teleop.config import PORT, ROBOT_ID  # noqa: E402

ARCHIVE = pathlib.Path("data/calibrations")

FIELDS = ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit")


def read_eeprom(bus) -> dict[str, dict[str, int]]:
    """What the motors currently believe, straight off the EEPROM."""
    return {m: {f: bus.read(f, m, normalize=False, num_retry=3) for f in FIELDS}
            for m in bus.motors}


def mismatched_joints(bus, cal: dict | None = None) -> list[tuple[str, int, int]]:
    """Joints whose stored homing offset disagrees with the calibration file.

    Returns (joint, motor_offset, file_offset) per disagreement — empty when the motors
    and the file agree. Read-only, so it is safe to call before torque goes on.
    """
    cal = cal if cal is not None else json.loads(CAL.read_text())
    out = []
    for m in bus.motors:
        off = bus.read("Homing_Offset", m, normalize=False, num_retry=3)
        if off != cal[m]["homing_offset"]:
            out.append((m, off, cal[m]["homing_offset"]))
    return out


def archive_eeprom(bus) -> pathlib.Path:
    """Snapshot the live EEPROM so the pre-restore state is recoverable."""
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    dump = ARCHIVE / f"eeprom-before-restore-{time.strftime('%Y%m%d-%H%M%S')}.json"
    dump.write_text(json.dumps(read_eeprom(bus), indent=4) + "\n")
    return dump


def restore(robot, cal: dict | None = None) -> list[str]:
    """Drop torque, archive the EEPROM, write the file to the motors, verify.

    Returns the joints that did not take. Torque is left off: the caller decides when it
    is safe to arm.
    """
    cal = cal if cal is not None else json.loads(CAL.read_text())
    bus = robot.bus
    for m in bus.motors:
        bus.write("Torque_Enable", m, 0, num_retry=4)

    archive_eeprom(bus)
    bus.write_calibration(robot.calibration)

    after = read_eeprom(bus)
    bad = [m for m in bus.motors
           if not (after[m]["Homing_Offset"] == cal[m]["homing_offset"]
                   and after[m]["Min_Position_Limit"] == cal[m]["range_min"]
                   and after[m]["Max_Position_Limit"] == cal[m]["range_max"])]

    for m in bus.motors:
        try:
            bus.write("Torque_Enable", m, 0, num_retry=4)
        except Exception:
            pass
    return bad


def main() -> None:
    if PORT is None:
        raise SystemExit("No serial port found — plug the arm in and power it.")
    cal = json.loads(CAL.read_text())
    robot = SO101Follower(SO101FollowerConfig(port=PORT, id=ROBOT_ID))
    robot.connect(calibrate=False)
    bus = robot.bus

    wrong = mismatched_joints(bus, cal)
    if not wrong:
        print("Motors already agree with the calibration file. Nothing to do.")
        robot.disconnect()
        return

    print(f"{len(wrong)} joint(s) disagree with the file:\n")
    print(f"{'joint':16}{'motor':>9}{'file':>9}")
    for m, off, fo in wrong:
        print(f"{m:16}{off:9}{fo:9}")
    print("\nSUPPORT THE ARM — torque goes off before the write.")
    if input("\nRestore the calibration from file? [y/N] ").strip().lower() != "y":
        print("nothing written")
        robot.disconnect()
        return

    print("\ntorque OFF — writing...")
    bad = restore(robot, cal)

    obs = robot.get_observation()
    print("\nposition now reported:")
    for m in bus.motors:
        print(f"  {m:16}{obs[f'{m}.pos']:+8.1f}")

    print(f"\n{len(bad)} joint(s) did not take: {', '.join(bad)}" if bad
          else "\nAll six joints match the file. Torque left OFF.")
    robot.disconnect()


if __name__ == "__main__":
    main()
