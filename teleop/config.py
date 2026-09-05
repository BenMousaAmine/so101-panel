"""Where the arm is plugged in.

The serial port is the one thing every script needs and the one thing that changes
between machines and USB ports. It was a constant copied into a dozen files, so a new
cable meant editing them all; here it is detected, and remembered once chosen.

    from teleop.config import PORT          # what to connect to, or None

The arm's id defaults to "so101" and can be overridden with SO101_ID, which matters
only when one machine drives more than one arm.

Resolution order: the port saved in device.json if it is still present, otherwise the
only port on the bus, otherwise None — several candidates is a choice for the caller,
not something to guess at.
"""

import glob
import json
import os
import pathlib

DEVICE = pathlib.Path(__file__).with_name("device.json")

# lerobot files a calibration per robot id, so two arms on one machine stay apart.
ROBOT_ID = os.environ.get("SO101_ID", "so101")
CALIBRATION = (pathlib.Path.home() / ".cache/huggingface/lerobot/calibration/robots"
               / "so_follower" / f"{ROBOT_ID}.json")

# The Feetech bus board enumerates as a usbmodem; other adapters use the FTDI/CP210x name.
PATTERNS = ("/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.SLAB_USBtoUART*",
            "/dev/ttyACM*", "/dev/ttyUSB*")


def find_ports() -> list[str]:
    """Every serial port that could be the arm, in a stable order."""
    return sorted(p for pat in PATTERNS for p in glob.glob(pat))


def saved_port() -> str | None:
    try:
        return json.loads(DEVICE.read_text())["port"]
    except Exception:
        return None


def save_port(port: str) -> None:
    DEVICE.write_text(json.dumps({"port": port}, indent=4) + "\n")


def resolve_port() -> str | None:
    """The port to use right now, or None when it cannot be decided without asking."""
    ports = find_ports()
    saved = saved_port()
    if saved in ports:
        return saved
    if len(ports) == 1:
        return ports[0]
    return None


PORT = resolve_port()
