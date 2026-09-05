"""Everything the panel draws: palette, widgets, and the screens that are not the
joint list itself.

Kept apart from the control loop so a change to how something looks cannot alter what
the arm does.
"""

import pygame

from teleop.arm import Arm
from teleop.calibration_wizard import CENTRE, DONE
from teleop.park import ARRIVED, ORDER, unreachable
from teleop.limits import (
    DESC,
    ENCODER,
    LOAD_STOP,
    LOAD_WARN,
    RAW_RANGE,
    SCALE,
    SPEEDS,
    STALE_AFTER,
    TRAVEL,
)

PORT_HINT = ("/dev/cu.usbmodem*", "/dev/ttyACM*")

W, H = 1060, 760
# --sim adds a column for the model to the right of the panel. W stays the panel's own
# width so every widget keeps its geometry; only the window knows about the extra space.
SIM_W = 620
WINDOW_W = W       # the surface; W stays the panel's own width, so no widget moves


def widen_for_sim() -> None:
    global WINDOW_W
    WINDOW_W = W + SIM_W


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
        self.screen = pygame.display.set_mode((WINDOW_W, H))
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
        # Judged against the calibrated range, not against TRAVEL. TRAVEL keeps MARGIN in
        # hand so commands never push into the stop; a joint that gravity settled a
        # degree past it is still somewhere the arm reaches on its own, and calling that
        # "outside travel" contradicts the bar drawn right beside it.
        outside = pos < lo or pos > hi

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
        present(ui, arm)
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
    present(ui, arm)


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
    present(ui, arm)


def draw_panel(ui, arm, names, pos, target, loads, temps, volts, torque, errors,
               last_ok, blocked_dir, sel, speed_i, status, stopped, moving, focused,
               rec, now) -> None:
    """The armed panel: joint rows, the summary strip, and the key legend."""
    s = ui.screen
    s.fill(BG)

    pygame.draw.line(s, STROKE, (PAD, 62), (W - PAD, 62), 1)
    ui.text("SO-101 Controller", ui.f_title, TEXT, PAD, 20)
    ui.text("direct joint control", ui.f_sub, FAINT, PAD + 218, 27)

    stuck_out = [n for n in names
                 if not (SCALE[n][0] <= pos[n] <= SCALE[n][1])]
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
    present(ui, arm)


def present(ui, arm) -> None:
    """Show the frame, with the simulated arm drawn in first when there is one."""
    draw_sim_view(ui, arm)
    pygame.display.flip()


def draw_sim_view(ui, arm) -> None:
    """The simulated arm, rendered into the column --sim added. A no-op on real hardware."""
    view = getattr(arm, "view", None)
    if view is None:
        return
    s = ui.screen
    x = W + 8
    panel = pygame.Rect(x, 78, SIM_W - PAD - 8, H - 78 - 96)
    pygame.draw.rect(s, PANEL, panel, border_radius=10)
    pygame.draw.rect(s, STROKE, panel, 1, border_radius=10)

    frame = view.frame()
    if frame is None:
        ui.text(view.error or "no render", ui.f_small, BAD, panel.x + 16, panel.y + 16)
        return
    fw, fh = frame.get_size()
    scale = min((panel.w - 24) / fw, (panel.h - 52) / fh)
    if scale < 1.0:
        frame = pygame.transform.smoothscale(frame, (int(fw * scale), int(fh * scale)))
    s.blit(frame, frame.get_rect(center=(panel.centerx, panel.centery + 12)))
    ui.text("SIMULATION", ui.f_small, FAINT, panel.x + 16, panel.y + 14)
    ui.text(f"{view.camera_name}   TAB", ui.f_small, DIM,
            panel.right - 16, panel.y + 14, right=True)


def draw_park(ui, arm, park, pose: dict, asked_to_quit: bool) -> None:
    """The parking run: which joint is moving now, and what happened to the rest."""
    s = ui.screen
    s.fill(BG)
    pygame.draw.line(s, STROKE, (PAD, 62), (W - PAD, 62), 1)
    ui.text("Parking", ui.f_title, TEXT, PAD, 20)
    ui.text("one joint at a time, wrist to base", ui.f_sub, FAINT, PAD + 108, 27)

    card = pygame.Rect(PAD, 104, W - PAD * 2, 340 if park and park.done else 320)
    pygame.draw.rect(s, PANEL, card, border_radius=12)
    pygame.draw.rect(s, STROKE, card, 1, border_radius=12)
    x, y = PAD + 30, 132

    if park is None:
        if not pose:
            ui.text("No safe pose saved yet", ui.f_head, WARN, x, y)
            ui.text("Move the arm where it should rest — by hand with F, or with the",
                    ui.f_small, DIM, x, y + 34)
            ui.text("arrow keys — then save that pose here.", ui.f_small, DIM, x, y + 52)
            bx = ui.keycap("S", x, y + 84, active=True)
            ui.text("save the current pose as the safe one", ui.f_small, TEXT, bx + 4, y + 87)
        else:
            bad = unreachable(pose)
            if bad:
                ui.text("This pose is outside the calibrated travel", ui.f_head, BAD, x, y)
                ui.text("A park to here could never finish: every command is clamped to",
                        ui.f_small, DIM, x, y + 30)
                ui.text("the limits, so these joints would stop short every time.",
                        ui.f_small, DIM, x, y + 48)
                for i, (j, at, limit) in enumerate(bad):
                    ui.text(f"{j:16}{at:+8.1f}   needs {limit:+.1f} or inside",
                            ui.f_val, TEXT, x, y + 80 + i * 22)
                fy2 = y + 92 + len(bad) * 22
                ui.text("Move those joints just inside, then save again.",
                        ui.f_small, WARN, x, fy2)
                bx = ui.keycap("S", x, fy2 + 26, active=True)
                ui.text("save the current pose", ui.f_small, TEXT, bx + 4, fy2 + 29)
                ui.keycap("ESC", PAD, H - 44)
                ui.text("back to the panel", ui.f_small, FAINT, PAD + 44, H - 41)
                present(ui, arm)
                return
            ui.text("Park before quitting?" if asked_to_quit else "Park now?",
                    ui.f_head, TEXT, x, y)
            ui.text("The arm moves to the saved pose, one joint at a time, wrist first.",
                    ui.f_small, DIM, x, y + 34)
            ui.text("It stops on its own if any joint strains or falls short.",
                    ui.f_small, DIM, x, y + 52)
            bx = ui.keycap("ENTER", x, y + 84, active=True)
            ui.text("one joint at a time — wrist to base", ui.f_small, TEXT,
                    bx + 4, y + 87)
            bx = ui.keycap("T", x, y + 112)
            ui.text("all together, timed so they arrive at once", ui.f_small, TEXT,
                    bx + 4, y + 115)
            bx = ui.keycap("S", x, y + 144)
            ui.text("re-save this pose", ui.f_small, FAINT, bx + 4, y + 147)
            if asked_to_quit:
                bx = ui.keycap("Q", x + 480, y + 84)
                ui.text("quit now — torque off, the arm drops", ui.f_small, BAD,
                        bx + 4, y + 87)
        ui.keycap("ESC", PAD, H - 44)
        ui.text("back to the panel", ui.f_small, FAINT, PAD + 44, H - 41)
        present(ui, arm)
        return

    seen = {j: r for j, r in park.results}
    for i, j in enumerate(park.joints):
        row = y + i * 26
        if j in seen:
            ok = seen[j] == ARRIVED
            pygame.draw.circle(s, GOOD if ok else BAD, (x + 6, row + 8), 6)
            ui.text(seen[j], ui.f_small, GOOD if ok else BAD, x + 250, row + 2)
        elif j == park.joint and not park.done:
            pygame.draw.circle(s, ACCENT, (x + 6, row + 8), 6)
            ui.text("moving", ui.f_small, ACCENT, x + 250, row + 2)
        else:
            pygame.draw.circle(s, TRACK, (x + 6, row + 8), 6)
        ui.text(j, ui.f_val, TEXT if j in seen or j == park.joint else FAINT,
                x + 24, row)
        ui.text(f"{park.pose[j]:+.1f}", ui.f_small, FAINT, x + 180, row + 2)

    fy = y + len(park.joints) * 26 + 20
    if park.done:
        good = park.outcome == ARRIVED
        ui.text(park.detail, ui.f_head, GOOD if good else BAD, x, fy)
        if not good:
            ui.text("The arm is still held where it stopped. Nothing was forced.",
                    ui.f_small, DIM, x, fy + 28)
        if good:
            ui.text("Is the arm resting on something here?", ui.f_small, TEXT, x, fy + 30)
            ui.text("A pose it rests on needs no motors, and holding one makes heat.",
                    ui.f_small, DIM, x, fy + 48)
            bx = ui.keycap("F", x, fy + 74, active=True)
            ui.text("release torque and quit", ui.f_small, TEXT, bx + 4, fy + 77)
            bx = ui.keycap("ENTER", x + 250, fy + 74)
            ui.text("keep holding and quit", ui.f_small, FAINT, bx + 4, fy + 77)
        else:
            bx = ui.keycap("ENTER", x, fy + 74, active=True)
            ui.text("quit — the arm stays where it stopped", ui.f_small, TEXT,
                    bx + 4, fy + 77)
        bx = ui.keycap("ESC", x, fy + 104)
        ui.text("back to the panel — the arm stays parked", ui.f_small, FAINT,
                bx + 4, fy + 107)
    else:
        together = hasattr(park, "progress")
        if together:
            ui.text(f"moving all six — {park.progress * 100:.0f}%", ui.f_head, ACCENT, x, fy)
            bw = W - PAD * 2 - 60
            pygame.draw.rect(s, TRACK, pygame.Rect(x, fy + 28, bw, 8), border_radius=4)
            pygame.draw.rect(s, ACCENT,
                             pygame.Rect(x, fy + 28, max(3, int(bw * park.progress)), 8),
                             border_radius=4)
            lead = park.joint
            ui.text(f"{lead} has furthest to go" if lead else "", ui.f_small, FAINT,
                    x, fy + 44)
            by = fy + 70
        else:
            ui.text(f"moving {park.joint}", ui.f_head, ACCENT, x, fy)
            by = fy + 34
        bx = ui.keycap("ESC", x, by, active=True)
        ui.text("stop everything — the arm stays held", ui.f_small, WARN, bx + 4, by + 3)
    present(ui, arm)
