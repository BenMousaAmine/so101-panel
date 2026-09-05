"""The arm as a state, not as an assumption.

The panel used to build a robot before it drew anything, so an unplugged arm was a stack
trace instead of a screen. Connecting is something the panel does while running: this
reports NO_PORT / CONNECTING / MISMATCH / LIMP / UNCALIBRATED / CALIBRATING / READY /
FAILED, and every hardware call is inert when there is nothing to talk to.
"""

import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

from teleop.calibration_wizard import Wizard
from teleop.config import ROBOT_ID, find_ports, resolve_port, save_port
from teleop.limits import CAL_PATH, POSITION_P, load_calibration, rebuild_scales
from teleop.restore_calibration import mismatched_joints, restore


class Arm:

    NO_PORT = "no_port"
    CONNECTING = "connecting"
    MISMATCH = "mismatch"
    LIMP = "limp"
    UNCALIBRATED = "uncalibrated"
    CALIBRATING = "calibrating"
    READY = "ready"
    FAILED = "failed"

    RETRY_EVERY = 1.0   # seconds between scans; plugging in should feel immediate

    def __init__(self):
        self.robot = None
        self.view = None        # a SimView mirroring this arm, when the panel drew one
        self.wizard = None
        self.port = None
        self.names: list[str] = []
        self.state = self.NO_PORT
        self.detail = "no serial port found"
        self.wrong: list[tuple[str, int, int]] = []
        self.ports: list[str] = []
        self._last_try = 0.0

    @property
    def live(self) -> bool:
        """True only when it is safe to command the arm."""
        return self.state == self.READY and self.robot is not None

    def poll(self) -> None:
        """Look for the arm and connect. Cheap enough to call every frame."""
        if self.state in (self.READY, self.MISMATCH, self.LIMP, self.UNCALIBRATED,
                          self.CALIBRATING):
            return
        now = time.time()
        if now - self._last_try < self.RETRY_EVERY:
            return
        self._last_try = now

        self.ports = find_ports()
        port = resolve_port()
        if port is None:
            self.state = self.NO_PORT
            self.detail = ("several ports found — unplug the others"
                           if len(self.ports) > 1 else "no serial port found")
            return
        if self.state == self.FAILED:
            self.detail = f"retrying {port}"
        self.connect(port)

    def connect(self, port: str) -> None:
        self.state = self.CONNECTING
        self.detail = f"opening {port}"
        try:
            robot = SO101Follower(SO101FollowerConfig(
                port=port, id=ROBOT_ID, max_relative_target=None))
            robot.connect(calibrate=False)
        except Exception as e:
            self.robot = None
            self.state = self.FAILED
            self.detail = str(e)[:70]
            return

        self.robot = robot
        self.port = port
        self.names = list(robot.bus.motors)
        save_port(port)

        for m in self.names:
            try:
                if robot.bus.read("P_Coefficient", m, normalize=False, num_retry=2) != POSITION_P:
                    robot.bus.write("P_Coefficient", m, POSITION_P, num_retry=3)
            except Exception:
                pass

        if not load_calibration():
            self.state = self.UNCALIBRATED
            self.detail = "no calibration file for this arm yet"
            return

        # Moving the arm by hand while unpowered rehomes the motors; torque on top of a
        # stale offset snaps the arm to a position it was never at.
        try:
            self.wrong = mismatched_joints(robot.bus)
        except Exception as e:
            self.state = self.FAILED
            self.detail = str(e)[:70]
            return
        if self.wrong:
            self.state = self.MISMATCH
            self.detail = f"{len(self.wrong)} joint(s) disagree with the file"
            return
        self.state = self.READY
        self.detail = f"connected on {port}"

    def restore_calibration(self) -> bool:
        """Write the file back to the motors. Torque is off throughout."""
        if self.robot is None:
            return False
        try:
            bad = restore(self.robot)
        except Exception as e:
            self.state = self.FAILED
            self.detail = str(e)[:70]
            return False
        if bad:
            self.detail = f"did not take: {', '.join(bad)}"
            return False
        self.wrong = []
        # Torque is off and the arm has sagged wherever gravity left it. Arming now
        # would read one pose, energise into another, and yank the arm there. The
        # operator has to take its weight first.
        self.state = self.LIMP
        self.detail = "calibration restored — the arm is limp"
        return True

    def recheck(self) -> list[tuple[str, int, int]]:
        """Compare the motors against the file again, while the panel is running.

        The offsets only drift when the arm is unplugged and moved, which cannot happen
        mid-session — but a bus glitch or a second process writing to the motors can,
        and checking costs one read per joint.
        """
        if self.robot is None:
            return []
        try:
            self.wrong = mismatched_joints(self.robot.bus)
        except Exception as e:
            self.detail = str(e)[:70]
            return []
        return self.wrong

    def start_calibration(self) -> None:
        """Hand the arm to the guided calibration; nothing is powered until it finishes."""
        if self.robot is None:
            return
        self.wizard = Wizard(self.robot.bus, self.names or list(self.robot.bus.motors), CAL_PATH)
        self.wizard.start()
        self.state = self.CALIBRATING
        self.detail = "guided calibration"

    def finish_calibration(self) -> None:
        """Save what the wizard recorded, push it to the motors and rebuild the scales."""
        if self.wizard is None or self.robot is None:
            return
        self.wizard.save()
        rebuild_scales()
        self.wizard = None
        port = self.port
        # Reconnect so the robot reloads the file it just wrote, then push it to the
        # motors: the file alone leaves the EEPROM on the old homing offsets, which
        # would greet a freshly calibrated arm with a mismatch it just resolved.
        try:
            self.robot.disconnect()
        except Exception:
            pass
        self.robot = None
        self.connect(port)
        if self.state == self.MISMATCH:
            self.restore_calibration()
        elif self.state == self.READY:
            # Every joint was left free for the calibration, so the arm is hanging.
            self.state = self.LIMP
            self.detail = "calibration saved — the arm is limp"

    def drop(self, why: str) -> None:
        """Give up the connection and go back to looking for one."""
        try:
            if self.robot is not None:
                self.robot.disconnect()
        except Exception:
            pass
        self.robot = None
        self.state = self.NO_PORT
        self.detail = why
        self._last_try = time.time()

    def observation(self) -> dict:
        if not self.live:
            return {}
        try:
            return self.robot.get_observation()
        except Exception:
            return {}

    def torque(self, joint: str, on: bool) -> None:
        if not self.live:
            return
        try:
            self.robot.bus.write("Torque_Enable", joint, 1 if on else 0, num_retry=4)
        except Exception:
            pass

    def send(self, action: dict) -> None:
        if not self.live or not action:
            return
        try:
            self.robot.send_action(action)
        except Exception:
            pass

    def read(self, field: str, joint: str):
        if not self.live:
            return None
        try:
            return self.robot.bus.read(field, joint, normalize=False, num_retry=1)
        except Exception:
            return None
