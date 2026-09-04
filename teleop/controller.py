"""SO-101 Controller — a graphical panel for driving one joint at a time.

    python teleop/controller.py

Direct joint control, no inverse kinematics: nothing can ask a joint for a position it
cannot reach. Each joint shows its live position against its measured travel, so you
always see where it is and how much room is left.

    1-6            select joint
    left / right   drive the selected joint while held
    up / down      speed
    T              toggle torque on the selected joint
    H              hold every joint where it is
    F              free every joint (support the arm first)
    R              re-arm everything after a stop
    C              guided calibration, one joint at a time
    V              verify the motors against the calibration file, and restore it
    SPACE (hold 1s) release all torque — the arm will drop, so support it first
    Q / ESC        quit

On startup the motors are checked against the calibration file: moving the arm by hand
while unpowered leaves them homed somewhere else, and arming on top of that makes the
arm snap. A mismatch is offered a fix before anything is powered, and V runs the same
check again at any time.

Every frame is logged to data/logs/.
"""

import json
import os
import pathlib
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig  # noqa: E402

from teleop.calibration_wizard import CENTRE, DONE, ENCODER, Wizard  # noqa: E402
from teleop.calibration_wizard import TRAVEL as STEP_TRAVEL  # noqa: E402
from teleop.config import CALIBRATION as CAL_PATH  # noqa: E402
from teleop.config import ROBOT_ID, find_ports, resolve_port, save_port  # noqa: E402
from teleop.recorder import LightRecorder  # noqa: E402
from teleop.restore_calibration import mismatched_joints, restore  # noqa: E402

FPS = 30
SPEEDS = [2.0, 5.0, 10.0, 20.0, 35.0, 55.0]
# At P=36 a normal lift peaks around 850, so the old 200/350 thresholds fired on healthy
# movement. These sit above working load and below the 1000 ceiling that trips the
# motor's own overload protection.
LOAD_WARN = 700
LOAD_STOP = 950
# lerobot's configure() writes P_Coefficient=16, half the STS3215 factory default of 32
# (huggingface/lerobot#3400). At 16 the loop stops pushing well before the available
# torque is used: shoulder_lift climbed 2.9 deg at P=16 and 36.7 deg at P=36, measured
# on this arm. The step is sharp between 34 and 36; 38 gains nothing and costs load.
POSITION_P = 36
# How far the commanded position may run ahead of where the joint actually is. Without
# this the target keeps advancing every frame while the joint stalls, so the error winds
# up without limit and the motor is left straining at a goal tens of degrees away.
MAX_LAG = 8.0
LOAD_EVERY = 3
TEMP_EVERY = 30      # temperature and voltage move slowly; once a second is plenty
STALE_AFTER = 2.0    # seconds without a good read before a joint is shown as unresponsive

MARGIN = 3.0  # stay clear of the mechanical stop


def load_calibration() -> dict:
    """The calibration file, or an empty dict when there is none.

    The panel has to be able to open and say what is wrong; dying at import time on a
    missing file is the failure it is meant to report.
    """
    try:
        return json.loads(CAL_PATH.read_text())
    except Exception:
        return {}


def build_travel() -> dict[str, tuple[float, float]]:
    """Limits in the units lerobot actually reports.

    With `use_degrees=True` (the SO-101 default) every joint except the gripper is
    normalised as DEGREES from the centre of its calibrated range:
        deg = (raw - (min+max)/2) * 360 / 4095
    So the endpoints are not ±100 — they are however many degrees that joint's range
    spans. Treating them as percentages is what made healthy joints look out of scale.
    """
    cal = load_calibration()
    out = {}
    for name, c in cal.items():
        if name == "gripper":
            out[name] = (MARGIN, 100.0 - MARGIN)
        else:
            half = (c["range_max"] - c["range_min"]) / 2 * 360 / 4095
            out[name] = (-half + MARGIN, half - MARGIN)
    return out


TRAVEL: dict[str, tuple[float, float]] = {}
RAW_RANGE: dict[str, tuple[int, int]] = {}
SCALE: dict[str, tuple[float, float]] = {}


def rebuild_scales() -> None:
    """Re-read the calibration into the display scales, after it changes on disk."""
    TRAVEL.clear()
    TRAVEL.update(build_travel())
    RAW_RANGE.clear()
    SCALE.clear()
    for name, c in load_calibration().items():
        RAW_RANGE[name] = (c["range_min"], c["range_max"])
        if name == "gripper":
            SCALE[name] = (0.0, 100.0)
        else:
            half = (c["range_max"] - c["range_min"]) / 2 * 360 / 4095
            SCALE[name] = (-half, half)


rebuild_scales()
DESC = {
    "shoulder_pan": "base rotation, left / right",
    "shoulder_lift": "shoulder, raise and lower",
    "elbow_flex": "elbow, bends and extends",
    "wrist_flex": "wrist, up and down",
    "wrist_roll": "wrist, axial rotation",
    "gripper": "gripper, open and close",
}

PORT_HINT = ("/dev/cu.usbmodem*", "/dev/cu.usbserial*")

W, H = 1060, 760
PAD = 28
ROW_H = 78
ROW_GAP = 6

BG = (16, 17, 21)
PANEL = (26, 28, 34)
PANEL_SEL = (33, 38, 50)
STROKE = (44, 47, 56)
TEXT = (236, 238, 243)
DIM = (124, 129, 143)
FAINT = (78, 82, 94)
ACCENT = (96, 165, 250)
GOOD = (74, 201, 141)
WARN = (245, 186, 74)
BAD = (240, 100, 100)
TRACK = (38, 41, 49)


def lerp(a, b, t):
    return tuple(int(x + (y - x) * t) for x, y in zip(a, b))


class UI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((W, H))
        pygame.display.set_caption("SO-101 Controller")
        mono = "menlo,monaco,dejavusansmono,consolas,monospace"
        sans = "helveticaneue,helvetica,arial,sans-serif"
        self.f_title = pygame.font.SysFont(sans, 22, bold=True)
        self.f_sub = pygame.font.SysFont(sans, 13)
        self.f_head = pygame.font.SysFont(sans, 16, bold=True)
        self.f_name = pygame.font.SysFont(mono, 15, bold=True)
        self.f_big = pygame.font.SysFont(mono, 19, bold=True)
        self.f_val = pygame.font.SysFont(mono, 14)
        self.f_small = pygame.font.SysFont(mono, 11)
        self.f_key = pygame.font.SysFont(mono, 11, bold=True)

    def text(self, s, font, colour, x, y, right=False, centre=False):
        surf = font.render(s, True, colour)
        r = surf.get_rect()
        if right:
            r.topright = (x, y)
        elif centre:
            r.midtop = (x, y)
        else:
            r.topleft = (x, y)
        self.screen.blit(surf, r)
        return r

    def pill(self, label, colour, x, y, font=None):
        font = font or self.f_small
        w = font.size(label)[0] + 14
        pygame.draw.rect(self.screen, colour, pygame.Rect(x, y, w, 17), border_radius=8)
        self.text(label, font, BG, x + 7, y + 3)
        return x + w + 6

    def joint_row(self, i, name, pos, cmd, load, temp, volt, torque, selected, stale, y,
                  blocked=0):
        # Telemetry can be missing for a joint that has not answered yet; drawing must
        # never be the thing that takes the panel down.
        load, temp, volt = load or 0, temp or 0, volt or 0.0
        lo, hi = SCALE[name]
        clo, chi = TRAVEL[name]
        unit = "%" if name == "gripper" else "\u00b0"
        outside = pos < clo or pos > chi

        rect = pygame.Rect(PAD, y, W - PAD * 2, ROW_H)
        pygame.draw.rect(self.screen, PANEL_SEL if selected else PANEL, rect, border_radius=10)
        pygame.draw.rect(self.screen, ACCENT if selected else STROKE, rect, 1, border_radius=10)
        if selected:
            pygame.draw.rect(self.screen, ACCENT, pygame.Rect(PAD, y + 14, 3, ROW_H - 28),
                             border_radius=2)

        self.text(str(i + 1), self.f_name, ACCENT if selected else FAINT, PAD + 18, y + 13)
        self.text(name, self.f_name, TEXT if torque else DIM, PAD + 40, y + 12)
        self.text(DESC[name], self.f_small, FAINT, PAD + 40, y + 32)

        px2 = self.pill("HOLD" if torque else "FREE", GOOD if torque else WARN,
                        PAD + 40, y + 52)
        if blocked:
            self.pill(f"BLOCKED {'>' if blocked > 0 else '<'}", BAD, px2, y + 52)
        elif stale:
            self.text("not responding", self.f_small, BAD, px2, y + 55)

        bx, by, bw, bh = 330, y + 26, 380, 10
        pygame.draw.rect(self.screen, TRACK, pygame.Rect(bx, by, bw, bh), border_radius=5)

        span = hi - lo
        frac = max(0.0, min(1.0, (pos - lo) / span))
        near = frac < 0.04 or frac > 0.96
        strained = load > LOAD_WARN
        if not torque:
            fill = FAINT
        elif outside or near or load > LOAD_STOP:
            fill = BAD
        elif strained:
            fill = WARN
        else:
            fill = GOOD

        # A position on a centred axis: filling from the left would read as a quantity.
        origin = 0.0 if name == "gripper" else 0.5
        ox, px = bx + int(bw * origin), bx + int(bw * frac)
        if abs(px - ox) > 1:
            pygame.draw.rect(self.screen, fill,
                             pygame.Rect(min(ox, px), by, abs(px - ox), bh), border_radius=5)

        for edge in (clo, chi):
            ex = bx + int(bw * (edge - lo) / span)
            pygame.draw.line(self.screen, FAINT, (ex, by - 2), (ex, by + bh + 2), 1)

        pygame.draw.line(self.screen, STROKE, (ox, by - 4), (ox, by + bh + 4), 1)

        cfrac = max(0.0, min(1.0, (cmd - lo) / span))
        if abs(cfrac - frac) > 0.005:
            cx = bx + int(bw * cfrac)
            pygame.draw.line(self.screen, WARN, (cx, by - 5), (cx, by + bh + 5), 2)

        pygame.draw.circle(self.screen, TEXT if torque else DIM, (px, by + bh // 2), 5)
        pygame.draw.circle(self.screen, BG, (px, by + bh // 2), 5, 1)

        fmt = "{:.0f}" if name == "gripper" else "{:+.0f}"
        self.text(fmt.format(lo) + unit, self.f_small, FAINT, bx, by + 16)
        self.text(fmt.format(hi) + unit, self.f_small, FAINT, bx + bw, by + 16, right=True)
        self.text(("{:.1f}" if name == "gripper" else "{:+.1f}").format(pos) + unit, self.f_big,
                  BAD if outside else (TEXT if torque else DIM), bx + bw + 74, y + 22, right=True)
        if outside:
            self.text("outside travel", self.f_small, BAD, bx + bw + 74, y + 46, right=True)

        tx = W - PAD - 116
        lcol = BAD if load > LOAD_STOP else (WARN if load > LOAD_WARN else DIM)

        tcol = BAD if temp >= 55 else (WARN if temp >= 45 else DIM)
        for col, (label, value, colour) in enumerate((
            ("load", f"{load}", lcol),
            ("temp", f"{temp}\u00b0" if temp else "--", tcol),
            ("volt", f"{volt:.1f}V" if volt else "--", DIM),
        )):
            cx = tx + col * 40
            self.text(label, self.f_small, FAINT, cx, y + 20)
            self.text(value, self.f_val, colour, cx, y + 36)

    def keycap(self, label, x, y, active=False):
        w = self.f_key.size(label)[0] + 14
        pygame.draw.rect(self.screen, ACCENT if active else TRACK,
                         pygame.Rect(x, y, w, 19), border_radius=5)
        if not active:
            pygame.draw.rect(self.screen, STROKE, pygame.Rect(x, y, w, 19), 1, border_radius=5)
        self.text(label, self.f_key, BG if active else DIM, x + 7, y + 4)
        return x + w + 5


class Arm:
    """The arm as a state, not as an assumption.

    The panel used to build a robot before it drew anything, so an unplugged arm was a
    stack trace instead of a screen. Here connecting is something the panel does while
    running: it reports NO_PORT / CONNECTING / MISMATCH / READY, and every hardware call
    goes through methods that simply do nothing when there is nothing to talk to.
    """

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


def draw_wizard(ui, arm) -> None:
    """The guided calibration: one joint, one instruction, live travel."""
    w = arm.wizard
    s = ui.screen
    s.fill(BG)
    pygame.draw.line(s, STROKE, (PAD, 62), (W - PAD, 62), 1)
    ui.text("Calibration", ui.f_title, TEXT, PAD, 20)
    ui.text("one joint at a time", ui.f_sub, FAINT, PAD + 128, 27)
    ui.text(f"joint {w.i + 1} of {len(w.joints)}", ui.f_small, DIM, W - PAD, 32, right=True)

    for j in range(len(w.joints)):
        col = GOOD if j < w.i or w.step == DONE else (ACCENT if j == w.i else TRACK)
        seg = (W - PAD * 2 - 5 * 6) // len(w.joints)
        pygame.draw.rect(s, col, pygame.Rect(PAD + j * (seg + 6), 76, seg, 4), border_radius=2)

    card = pygame.Rect(PAD, 104, W - PAD * 2, 340 if w.step == DONE else 262)
    pygame.draw.rect(s, PANEL, card, border_radius=12)
    pygame.draw.rect(s, STROKE, card, 1, border_radius=12)
    x, y = PAD + 30, 132

    if w.step == DONE:
        ui.text("All six joints recorded", ui.f_head, GOOD, x, y)
        ui.text("Nothing has been written yet. Saving stores the file and pushes it",
                ui.f_small, DIM, x, y + 34)
        ui.text("to the motors.", ui.f_small, DIM, x, y + 52)
        ui.text(f"{'joint':18}{'min':>8}{'max':>8}{'span':>8}", ui.f_small, FAINT, x, y + 86)
        for i, (name, c) in enumerate(w.result.items()):
            ui.text(f"{name:18}{c['range_min']:>8}{c['range_max']:>8}"
                    f"{c['range_max'] - c['range_min']:>8}", ui.f_val, TEXT, x, y + 106 + i * 19)
        bx = ui.keycap("ENTER", x, 396, active=True)
        ui.text("save and use it", ui.f_small, TEXT, bx + 4, 399)
        bx = ui.keycap("ESC", bx + 130, 396)
        ui.text("discard", ui.f_small, FAINT, bx + 4, 399)
        pygame.display.flip()
        return

    ui.text(w.joint, ui.f_head, TEXT, x, y)
    ui.text(DESC.get(w.joint, ""), ui.f_small, FAINT, x + ui.f_head.size(w.joint)[0] + 16, y + 4)

    if w.step == CENTRE:
        ui.text("Step 1  —  put this joint in the MIDDLE of its travel by hand",
                ui.f_small, TEXT, x, y + 40)
        ui.text("Every joint is limp. Aim for the marker near the centre line.",
                ui.f_small, DIM, x, y + 62)

        # The encoder's own midpoint. It is not the joint's true mechanical centre —
        # that is what this step is about to define — but it is the reference the
        # operator can actually see, and starting far from it wastes half the travel.
        bx, by, bw, bh = x, y + 104, W - PAD * 2 - 60, 14
        pygame.draw.rect(s, TRACK, pygame.Rect(bx, by, bw, bh), border_radius=7)
        mid = bx + bw // 2
        pygame.draw.rect(s, PANEL_SEL,
                         pygame.Rect(mid - int(bw * 0.08), by, int(bw * 0.16), bh),
                         border_radius=7)
        pygame.draw.line(s, ACCENT, (mid, by - 6), (mid, by + bh + 6), 2)

        frac = max(0.0, min(1.0, w.now / ENCODER))
        nx = bx + int(bw * frac)
        off = w.now - ENCODER // 2
        near = abs(off) < ENCODER * 0.08
        pygame.draw.circle(s, GOOD if near else WARN, (nx, by + bh // 2), 7)
        pygame.draw.circle(s, BG, (nx, by + bh // 2), 7, 1)

        ui.text("0", ui.f_small, FAINT, bx, by + 22)
        ui.text("centre", ui.f_small, ACCENT, mid, by + 22, centre=True)
        ui.text(str(ENCODER), ui.f_small, FAINT, bx + bw, by + 22, right=True)
        ui.text(f"raw {w.now}", ui.f_val, TEXT, x, by + 46)
        ui.text("good — press ENTER" if near else
                f"{abs(off)} counts {'above' if off > 0 else 'below'} centre",
                ui.f_small, GOOD if near else WARN, x + 110, by + 49)

        bx2 = ui.keycap("ENTER", x, 320, active=True)
        ui.text("this is the middle", ui.f_small, TEXT, bx2 + 4, 323)
    else:
        ui.text("Step 2  —  move it through its FULL travel, both directions",
                ui.f_small, TEXT, x, y + 40)
        ui.text("Stop at the first resistance. Forcing past the stop wraps the encoder.",
                ui.f_small, WARN, x, y + 62)

        bx, by, bw, bh = x, y + 104, W - PAD * 2 - 60, 14
        pygame.draw.rect(s, TRACK, pygame.Rect(bx, by, bw, bh), border_radius=7)
        if w.lo is not None and w.span:
            f_lo, f_hi = w.lo / ENCODER, w.hi / ENCODER
            pygame.draw.rect(s, GOOD if w.ok else BAD,
                             pygame.Rect(bx + int(bw * f_lo), by,
                                         max(3, int(bw * (f_hi - f_lo))), bh), border_radius=7)
            nx = bx + int(bw * w.now / ENCODER)
            pygame.draw.circle(s, TEXT, (nx, by + bh // 2), 6)
            pygame.draw.circle(s, BG, (nx, by + bh // 2), 6, 1)

        if w.lo is None:
            ui.text("waiting for movement", ui.f_small, FAINT, bx, by + 26)
        else:
            ui.text(f"min {w.lo}", ui.f_val, DIM, bx, by + 26)
            ui.text(f"now {w.now}", ui.f_val, TEXT, bx + bw // 2, by + 26, centre=True)
            ui.text(f"max {w.hi}", ui.f_val, DIM, bx + bw, by + 26, right=True)
            ui.text(f"span {w.span}", ui.f_val, TEXT, bx, by + 50)
            ui.text(w.note, ui.f_small, GOOD if w.ok else BAD, bx + 110, by + 53)

        cx = ui.keycap("ENTER", x, 320, active=bool(w.lo is not None and w.ok))
        ui.text("accept this joint", ui.f_small, TEXT if w.ok else FAINT, cx + 4, 323)
        cx = ui.keycap("R", cx + 140, 320)
        ui.text("redo it", ui.f_small, FAINT, cx + 4, 323)

    ui.keycap("ESC", PAD, H - 44)
    ui.text("cancel — nothing is written", ui.f_small, FAINT, PAD + 44, H - 41)
    pygame.display.flip()


def draw_disconnected(ui, arm) -> None:
    """The panel when there is no arm to drive: what is wrong, and what to do about it."""
    s = ui.screen
    s.fill(BG)
    pygame.draw.line(s, STROKE, (PAD, 62), (W - PAD, 62), 1)
    ui.text("SO-101 Controller", ui.f_title, TEXT, PAD, 20)
    ui.text("direct joint control", ui.f_sub, FAINT, PAD + 218, 27)

    dot, head = {
        Arm.NO_PORT: (WARN, "ARM NOT CONNECTED"),
        Arm.CONNECTING: (ACCENT, "CONNECTING"),
        Arm.MISMATCH: (BAD, "CALIBRATION MISMATCH"),
        Arm.UNCALIBRATED: (ACCENT, "NOT CALIBRATED YET"),
        Arm.LIMP: (WARN, "ARM IS LIMP"),
        Arm.FAILED: (BAD, "CONNECTION FAILED"),
    }.get(arm.state, (DIM, arm.state.upper()))

    if arm.state == Arm.MISMATCH:
        body = 244 + len(arm.wrong) * 19
    elif arm.state == Arm.LIMP:
        body = 232
    else:
        body = 178
    card = pygame.Rect(PAD, 110, W - PAD * 2, body)
    pygame.draw.rect(s, PANEL, card, border_radius=12)
    pygame.draw.rect(s, STROKE, card, 1, border_radius=12)

    x, y = PAD + 30, 142
    pygame.draw.circle(s, dot, (x + 6, y + 10), 6)
    ui.text(head, ui.f_head, TEXT, x + 24, y)
    ui.text(arm.detail, ui.f_small, DIM, x + 24, y + 24)

    y += 62
    if arm.state == Arm.MISMATCH:
        ui.text("The arm was moved by hand while unpowered, so the motors are homed",
                ui.f_small, DIM, x, y)
        ui.text("somewhere the calibration file does not know about.",
                ui.f_small, DIM, x, y + 18)

        y += 50
        ui.text(f"{'joint':18}{'motor':>10}{'file':>10}", ui.f_small, FAINT, x, y)
        for i, (m, off, fo) in enumerate(arm.wrong):
            ui.text(f"{m:18}{off:>10}{fo:>10}", ui.f_val, TEXT, x, y + 20 + i * 19)

        y += 34 + len(arm.wrong) * 19
        ui.text("Support the arm — torque stays off during the fix.", ui.f_small, WARN, x, y)
        ui.keycap("Y", x, y + 22, active=True)
        ui.text("restore the calibration from file", ui.f_small, TEXT, x + 32, y + 25)
    elif arm.state == Arm.LIMP:
        ui.text("The motors now agree with the calibration file. Torque is off, so the",
                ui.f_small, DIM, x, y)
        ui.text("arm is hanging under its own weight.", ui.f_small, DIM, x, y + 18)
        ui.text("Take its weight and hold it in a safe pose before arming: powering up",
                ui.f_small, WARN, x, y + 44)
        ui.text("locks it exactly where it is.", ui.f_small, WARN, x, y + 62)
        bx = ui.keycap("ENTER", x, y + 92, active=True)
        ui.text("I am holding the arm — hold this pose", ui.f_small, TEXT, bx + 4, y + 95)
    elif arm.state == Arm.UNCALIBRATED:
        ui.text("This arm has no calibration on this machine yet. Until it does, the",
                ui.f_small, DIM, x, y)
        ui.text("panel cannot know where any joint is.", ui.f_small, DIM, x, y + 18)
        bx = ui.keycap("C", x, y + 48, active=True)
        ui.text("start the guided calibration — about two minutes", ui.f_small, TEXT,
                bx + 4, y + 51)
    else:
        ui.text("Plug the arm in and power it. It is detected automatically.",
                ui.f_small, DIM, x, y)
        ui.text(f"scanning   {'   '.join(PORT_HINT)}", ui.f_small, FAINT, x, y + 26)
        ui.text(f"ports seen   {', '.join(arm.ports) if arm.ports else 'none'}",
                ui.f_small, FAINT, x, y + 46)

    ui.keycap("Q", PAD, H - 44)
    ui.text("quit", ui.f_small, FAINT, PAD + 30, H - 41)
    pygame.display.flip()


def main() -> None:
    ui = UI()
    arm = Arm()
    names: list[str] = []
    rec = None

    target: dict[str, float] = {}
    pos: dict[str, float] = {}
    temps: dict[str, int] = {}
    volts: dict[str, float] = {}
    errors: dict[str, int] = {}
    last_ok: dict[str, float] = {}
    torque: dict[str, bool] = {}
    loads: dict[str, int] = {}
    blocked_dir: dict[str, int] = {}

    def arm_it() -> None:
        """Take up the arm's current pose and hold it. Called once, on connection."""
        nonlocal names, rec, target, pos, temps, volts, errors, last_ok
        nonlocal torque, loads, blocked_dir, sel, status
        names = arm.names
        obs = arm.observation()
        if not obs:
            arm.drop("lost the arm while arming")
            return
        target = {m: float(obs[f"{m}.pos"]) for m in names}
        pos = dict(target)
        for m in names:
            arm.torque(m, True)
        arm.send({f"{m}.pos": target[m] for m in names})
        temps = dict.fromkeys(names, 0)
        volts = dict.fromkeys(names, 0.0)
        errors = dict.fromkeys(names, 0)
        last_ok = {n: time.time() for n in names}
        torque = dict.fromkeys(names, True)
        loads = dict.fromkeys(names, 0)
        blocked_dir = dict.fromkeys(names, 0)
        sel = 0
        if rec is None:
            rec = LightRecorder(arm.robot, tag="controller",
                                extra_fields=["selected", "speed", "moving", "status"])
        status = f"connected on {arm.port}"

    sel, speed_i = 0, 0
    status, stopped, moving = "waiting for the arm", False, 0
    space_since = None
    SPACE_HOLD = 1.0   # seconds SPACE must be held before torque is cut
    dt, tick = 1.0 / FPS, 0
    running = True

    try:
        while running:
            t0 = time.perf_counter()
            tick += 1

            was_live = arm.live
            arm.poll()
            if arm.live and not was_live:
                arm_it()

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if arm.state == Arm.CALIBRATING:
                        w = arm.wizard
                        if ev.key == pygame.K_ESCAPE:
                            arm.wizard = None
                            arm.state = Arm.NO_PORT
                            arm.detail = "calibration cancelled — nothing written"
                            arm._last_try = 0.0
                        elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                            if w.step == CENTRE:
                                w.centre()
                            elif w.step == STEP_TRAVEL:
                                w.accept()
                            else:
                                arm.finish_calibration()
                                status = "calibration saved"
                        elif ev.key == pygame.K_r and w.step == STEP_TRAVEL:
                            w.redo()
                    elif ev.key in (pygame.K_q, pygame.K_ESCAPE):
                        running = False
                    elif arm.state == Arm.UNCALIBRATED and ev.key == pygame.K_c:
                        arm.start_calibration()
                    elif arm.live and ev.key == pygame.K_c:
                        for n in names:
                            arm.torque(n, False)
                        arm.start_calibration()
                    elif arm.state == Arm.MISMATCH and ev.key == pygame.K_y:
                        status = "restoring calibration — torque off"
                        if not arm.restore_calibration():
                            status = arm.detail
                    elif arm.state == Arm.LIMP and ev.key in (pygame.K_RETURN,
                                                              pygame.K_KP_ENTER):
                        arm.state = Arm.READY
                        arm_it()
                    elif arm.live and ev.key == pygame.K_v:
                        wrong = arm.recheck()
                        if not wrong:
                            status = "motors match the calibration file"
                        else:
                            for n in names:
                                arm.torque(n, False)
                            if not arm.restore_calibration():
                                status = arm.detail
                    elif not arm.live:
                        pass          # nothing else means anything without an arm
                    elif pygame.K_1 <= ev.key <= pygame.K_6:
                        sel = ev.key - pygame.K_1
                        status = f"selected {names[sel]}"
                    elif ev.key == pygame.K_UP:
                        speed_i = min(len(SPEEDS) - 1, speed_i + 1)
                    elif ev.key == pygame.K_DOWN:
                        speed_i = max(0, speed_i - 1)
                    elif ev.key == pygame.K_t:
                        m = names[sel]
                        torque[m] = not torque[m]
                        arm.torque(m, torque[m])
                        if torque[m]:
                            target[m] = float(arm.observation().get(f"{m}.pos", target[m]))
                            blocked_dir[m] = 0
                        status = f"{m} torque {'on' if torque[m] else 'off'}"
                    elif ev.key == pygame.K_f:
                        for n in names:
                            arm.torque(n, False)
                            torque[n] = False
                        status = "all joints free — support the arm"
                    elif ev.key == pygame.K_h:
                        now_obs = arm.observation()
                        for n in names:
                            target[n] = float(now_obs.get(f"{n}.pos", target[n]))
                            arm.torque(n, True)
                            torque[n] = True
                            blocked_dir[n] = 0
                        stopped = False
                        status = "all joints holding"
                    elif ev.key == pygame.K_r:
                        now = arm.observation()
                        for n in names:
                            target[n] = float(now.get(f"{n}.pos", target[n]))
                            arm.torque(n, True)
                            torque[n] = True
                            blocked_dir[n] = 0
                        stopped = False
                        status = "re-armed"
                    elif ev.key == pygame.K_SPACE:
                        space_since = time.time()

            keys = pygame.key.get_pressed()
            focused = pygame.key.get_focused()
            if arm.state == Arm.CALIBRATING:
                arm.wizard.poll()
                draw_wizard(ui, arm)
                time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
                continue
            if not arm.live:
                draw_disconnected(ui, arm)
                time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
                continue

            # Cutting all torque drops the arm, so it takes a hold, not a stray tap.
            if keys[pygame.K_SPACE] and focused and space_since is not None:
                held_for = time.time() - space_since
                if held_for >= SPACE_HOLD:
                    for n in names:
                        try:
                            arm.torque(n, False)
                        except Exception:
                            pass
                        torque[n] = False
                    stopped = True
                    space_since = None
                    status = "stopped — press R to re-arm"
                else:
                    status = f"hold SPACE to release torque... {SPACE_HOLD - held_for:.1f}s"
            elif not keys[pygame.K_SPACE]:
                if space_since is not None:
                    status = "release cancelled"
                space_since = None
            moving = 0
            if focused:
                moving = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)

            m = names[sel]
            if moving:
                # A joint sitting outside its travel is straining against its own stop,
                # so its load stays high and the block would re-arm every tick. The way
                # back inside is the fix for that strain, never something to refuse.
                mlo, mhi = TRAVEL[m]
                escaping = (pos[m] < mlo and moving > 0) or (pos[m] > mhi and moving < 0)
                if escaping:
                    blocked_dir[m] = 0
                elif blocked_dir[m] == moving:
                    status = f"{m} is stuck this way — drive it the other way"
                    moving = 0
                elif blocked_dir[m]:
                    blocked_dir[m] = 0      # driving the opposite way clears the block
            if moving and torque[m]:
                lo, hi = TRAVEL[m]
                step = moving * SPEEDS[speed_i] * dt
                nxt = target[m] + step
                # A joint that starts outside its travel — the arm sagged past a limit
                # while unpowered — must still be drivable back inside. Clamping it to
                # the limit it is already beyond would pin it there under load.
                if nxt < lo:
                    nxt = target[m] if target[m] < lo and step < 0 else lo
                elif nxt > hi:
                    nxt = target[m] if target[m] > hi and step > 0 else hi
                if nxt == target[m]:
                    status = f"{m} at limit ({lo:.0f} .. {hi:.0f})"
                    moving = 0
                # Never command further than MAX_LAG beyond the joint's real position:
                # a joint that cannot keep up must not accumulate an ever-growing error.
                lag_lo, lag_hi = pos[m] - MAX_LAG, pos[m] + MAX_LAG
                if nxt > lag_hi:
                    nxt = max(target[m], lag_hi) if target[m] > lag_hi else lag_hi
                    status = f"{m} is not keeping up — it may not have the torque here"
                elif nxt < lag_lo:
                    nxt = min(target[m], lag_lo) if target[m] < lag_lo else lag_lo
                    status = f"{m} is not keeping up — it may not have the torque here"
                target[m] = nxt

            held = {f"{n}.pos": target[n] for n in names if torque[n]}
            if held:
                arm.send(held)

            if tick % LOAD_EVERY == 0:
                for n in names:
                    # A joint that slammed into a stop goes quiet for a moment and reads
                    # None; keep the last good value rather than stalling the loop or
                    # poisoning the telemetry with a None the panel would then draw.
                    raw = arm.read("Present_Load", n)
                    if raw is None:
                        errors[n] += 1
                    else:
                        loads[n] = abs(raw)
                        last_ok[n] = time.time()

            if tick % TEMP_EVERY == 0:
                n = names[(tick // TEMP_EVERY) % len(names)]
                t = arm.read("Present_Temperature", n)
                v = arm.read("Present_Voltage", n)
                if t is None or v is None:
                    errors[n] += 1
                else:
                    temps[n] = t
                    volts[n] = v / 10

            if tick % LOAD_EVERY == 0:
                for n in names:
                    if blocked_dir[n] and loads[n] <= LOAD_WARN:
                        # Well below the warning threshold the joint is plainly not
                        # straining any more, so the block is a leftover flag that would
                        # refuse a command the motor can now carry out.
                        blocked_dir[n] = 0

                over = [n for n in names if loads[n] > LOAD_STOP]
                if over:
                    # Resetting the target to the current position instead oscillated at
                    # 30Hz against a held key: load hit 448 for 0.5 degrees in 15 seconds.
                    for n in over:
                        nlo, nhi = TRAVEL[n]
                        if pos[n] < nlo or pos[n] > nhi:
                            # Outside travel: the strain is the stop itself. Pinning the
                            # target to the present position would freeze the joint there
                            # for good, with no key able to move it.
                            continue
                        # Keep the direction that first caused the strain: recomputing
                        # it once the target has been pinned to pos flips it every tick,
                        # which made the badge alternate and the message contradict itself.
                        if not blocked_dir[n]:
                            blocked_dir[n] = 1 if target[n] > pos[n] else -1
                        target[n] = pos[n]
                    moving = 0
                    n0 = over[0]
                    status = (f"{n0} load {loads[n0]} — stuck. Press "
                              f"{names.index(n0) + 1} to select it, then drive it the "
                              "other way, or T to free just that joint.")
                elif "— stuck." in status:
                    # The strain is gone; leaving the warning up makes a working joint
                    # look jammed. The direction block stays until it is driven clear.
                    still = [n for n in names if blocked_dir[n]]
                    status = (f"{still[0]} still blocked one way — press "
                              f"{names.index(still[0]) + 1} and drive it back"
                              if still else "ready")
                # Not cleared on load: blocking is what makes load fall, so a low reading
                # would unblock instantly. Only driving the other way clears it.

            obs = arm.observation()
            if obs:
                for n in names:
                    if f"{n}.pos" in obs:
                        pos[n] = float(obs[f"{n}.pos"])
                        last_ok[n] = time.time()
            else:
                errors["_bus"] = errors.get("_bus", 0) + 1
            if rec is not None:
                rec.log(action=target,
                        extra={"selected": names[sel], "speed": SPEEDS[speed_i],
                               "moving": moving, "status": status})

            s = ui.screen
            s.fill(BG)

            pygame.draw.line(s, STROKE, (PAD, 62), (W - PAD, 62), 1)
            ui.text("SO-101 Controller", ui.f_title, TEXT, PAD, 20)
            ui.text("direct joint control", ui.f_sub, FAINT, PAD + 218, 27)

            stuck_out = [n for n in names
                         if not (TRAVEL[n][0] <= pos[n] <= TRAVEL[n][1])]
            if stuck_out:
                # A joint past its calibrated limit has no travel left in that direction:
                # it will not move however hard it is driven, and every further command
                # pushes it deeper into the mechanical stop. Nothing else matters until
                # it is back inside.
                banner, bcol = "JOINT OUTSIDE TRAVEL", BAD
                sub = f"{', '.join(stuck_out)} — drive back inside, or F and reposition"
            elif not focused:
                banner, bcol, sub = "CLICK TO TAKE CONTROL", WARN, "keys only reach the focused window"
            elif stopped:
                banner, bcol, sub = "STOPPED", BAD, "press R to re-arm"
            elif moving:
                banner, bcol, sub = (f"MOVING {names[sel]}", GOOD,
                                     f"{'right' if moving > 0 else 'left'} at {SPEEDS[speed_i]:.0f} deg/s")
            else:
                banner, bcol, sub = "READY", GOOD, f"{arm.port}"
            pygame.draw.circle(s, bcol, (W - PAD - 250, 33), 5)
            ui.text(banner, ui.f_head, bcol, W - PAD - 236, 22)
            ui.text(sub, ui.f_small, FAINT, W - PAD, 44, right=True)

            now = time.time()
            y = 78
            for i, n in enumerate(names):
                ui.joint_row(i, n, pos[n], target[n], loads[n], temps[n], volts[n],
                             torque[n], i == sel, now - last_ok[n] > STALE_AFTER, y,
                             blocked_dir.get(n, 0))
                y += ROW_H + ROW_GAP

            py = y + 4
            held_n = sum(torque.values())
            hottest = max(names, key=lambda n: temps[n])
            loaded = max(names, key=lambda n: loads[n])
            outside = [n for n in names if not (TRAVEL[n][0] <= pos[n] <= TRAVEL[n][1])]
            errs = sum(v for k, v in errors.items() if k in names)

            cells = [
                ("holding", f"{held_n}/6", GOOD if held_n == 6 else WARN),
                ("hottest", f"{hottest.split('_')[0]} {temps[hottest]}\u00b0" if temps[hottest] else "--",
                 BAD if temps[hottest] >= 55 else (WARN if temps[hottest] >= 45 else TEXT)),
                ("peak load", f"{loads[loaded]}",
                 BAD if loads[loaded] > LOAD_STOP else (WARN if loads[loaded] > LOAD_WARN else TEXT)),
                ("outside travel", str(len(outside)), BAD if outside else TEXT),
                ("read errors", str(errs), WARN if errs else TEXT),
                ("log rows", str(rec.rows if rec else 0), TEXT),
            ]
            cw = (W - PAD * 2) // len(cells)
            for j, (label, value, colour) in enumerate(cells):
                cx = PAD + j * cw + cw // 2
                if j:
                    pygame.draw.line(s, STROKE, (PAD + j * cw, py + 6), (PAD + j * cw, py + 40), 1)
                ui.text(label, ui.f_small, FAINT, cx, py + 8, centre=True)
                ui.text(value, ui.f_val, colour, cx, py + 26, centre=True)

            fy = py + 62
            pygame.draw.line(s, STROKE, (PAD, fy - 12), (W - PAD, fy - 12), 1)

            ui.text("SPEED", ui.f_small, FAINT, PAD, fy + 4)
            for j, sp in enumerate(SPEEDS):
                sel_sp = j == speed_i
                pygame.draw.rect(s, ACCENT if sel_sp else TRACK,
                                 pygame.Rect(PAD + 52 + j * 42, fy, 36, 19), border_radius=5)
                ui.text(f"{sp:.0f}", ui.f_key, BG if sel_sp else DIM,
                        PAD + 70 + j * 42, fy + 4, centre=True)

            x = PAD + 330
            x = ui.keycap("1-6", x, fy)
            x = ui.keycap("<", x, fy, active=moving < 0)
            x = ui.keycap(">", x, fy, active=moving > 0)
            x = ui.keycap("T", x, fy)
            x = ui.keycap("H", x, fy)
            x = ui.keycap("F", x, fy)
            x = ui.keycap("R", x, fy, active=stopped)
            x = ui.keycap("SPACE", x, fy)
            x = ui.keycap("C", x, fy)
            x = ui.keycap("V", x, fy)
            ui.keycap("Q", x, fy)

            ui.text("T torque  H hold  F free  R re-arm  SPACE release  C calibrate  V verify",
                    ui.f_small, FAINT, PAD + 330, fy + 26)
            ui.text(status, ui.f_small, WARN if stopped else DIM, PAD, fy + 44)
            ui.text(f"calibration ok    {arm.port}", ui.f_small, GOOD, W - PAD, fy + 44,
                    right=True)

            pygame.display.flip()

            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if rec is not None:
            rec.close()
        pygame.quit()
        for n in names:
            arm.torque(n, False)
        try:
            if arm.robot is not None:
                arm.robot.disconnect()
        except Exception:
            pass
        if names:
            print("torque released — support the arm before it drops")


if __name__ == "__main__":
    main()
