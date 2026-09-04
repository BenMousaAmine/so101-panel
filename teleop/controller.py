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

import os
import pathlib
import sys
import time

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from teleop.arm import Arm  # noqa: E402
from teleop.calibration_wizard import CENTRE, DONE  # noqa: E402
from teleop.calibration_wizard import TRAVEL as STEP_TRAVEL  # noqa: E402
from teleop.park import ARRIVED, Park, capture, load_pose, save_pose, unreachable  # noqa: E402
from teleop.limits import (  # noqa: E402
    FPS,
    LOAD_EVERY,
    LOAD_STOP,
    LOAD_WARN,
    MAX_LAG,
    SPEEDS,
    STALE_AFTER,
    TEMP_CUT,
    TEMP_EVERY,
    TRAVEL,
)
from teleop.recorder import LightRecorder  # noqa: E402
from teleop.ui import (  # noqa: E402
    ACCENT,
    BAD,
    BG,
    DIM,
    FAINT,
    GOOD,
    PAD,
    ROW_GAP,
    ROW_H,
    STROKE,
    TEXT,
    TRACK,
    UI,
    W,
    WARN,
    draw_disconnected,
    draw_panel,
    draw_park,
    draw_wizard,
)


def sample_loads(arm, names, loads, errors, last_ok) -> None:
    """Refresh each joint's load, keeping the last good value on a failed read.

    A joint that slammed into a stop goes quiet for a moment and reads None; storing that
    would poison the telemetry the panel then draws.
    """
    for n in names:
        raw = arm.read("Present_Load", n)
        if raw is None:
            errors[n] += 1
        else:
            loads[n] = abs(raw)
            last_ok[n] = time.time()


def sample_climate(arm, joint, temps, volts, errors) -> None:
    """One joint's temperature and voltage. Six reads at once visibly hitch the loop."""
    t = arm.read("Present_Temperature", joint)
    v = arm.read("Present_Voltage", joint)
    if t is None or v is None:
        errors[joint] += 1
    else:
        temps[joint] = t
        volts[joint] = v / 10


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
    park = None            # a Park run, or None when the screen is only offering one
    park_screen = False
    moved_since_park = False   # a finished run is only worth repeating if the arm moved
    quitting = False       # the park screen was opened by Q, so finishing it quits
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
                    if park_screen:
                        finished = park is not None and park.done
                        if ev.key == pygame.K_ESCAPE:
                            if park and not park.done:
                                park.abort()
                            else:
                                # Back to the panel, but the finished run is kept: it is
                                # clearing it that let the next Q drive the whole
                                # sequence a second time.
                                park_screen, quitting = False, False
                        elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                            if park is None and load_pose() and not unreachable(load_pose()):
                                park = Park(arm, load_pose(), dt)
                                moved_since_park = False
                            elif finished:
                                # A finished run is an ending, not a step back into the
                                # panel: returning there leaves park cleared, and the
                                # next Q would drive the whole sequence a second time.
                                running = False
                        elif (ev.key == pygame.K_f and park and park.done
                              and park.outcome == ARRIVED):
                            # The operator says the arm is resting on something, so the
                            # motors have nothing to hold and no reason to make heat.
                            for n in names:
                                arm.torque(n, False)
                                torque[n] = False
                            park.detail = "parked and released — resting, motors cool"
                            park.released = True
                            if quitting:
                                running = False
                        elif ev.key == pygame.K_s and arm.live:
                            save_pose(capture(arm))
                            status = "safe pose saved"
                        elif ev.key == pygame.K_q and quitting and park is None:
                            for n in names:
                                arm.torque(n, False)
                            running = False
                    elif arm.state == Arm.CALIBRATING:
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
                        if arm.live and load_pose():
                            park_screen, quitting = True, True
                            park = None if moved_since_park else park
                        else:
                            running = False
                    elif arm.live and ev.key == pygame.K_p:
                        park_screen, quitting = True, False
                        park = None if moved_since_park else park
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
            if park_screen:
                if park and not park.done:
                    park.step()
                draw_park(ui, arm, park, load_pose(), quitting)
                time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
                continue
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

            if moving:
                moved_since_park = True

            held = {f"{n}.pos": target[n] for n in names if torque[n]}
            if held:
                arm.send(held)

            if tick % LOAD_EVERY == 0:
                sample_loads(arm, names, loads, errors, last_ok)
            if tick % TEMP_EVERY == 0:
                hot = names[(tick // TEMP_EVERY) % len(names)]
                sample_climate(arm, hot, temps, volts, errors)
                # Free the joint before the motor has to save itself by leaving the bus.
                # Only this joint: the rest of the arm keeps holding.
                if temps[hot] >= TEMP_CUT and torque[hot]:
                    arm.torque(hot, False)
                    torque[hot] = False
                    status = (f"{hot} at {temps[hot]}C — torque cut on that joint to let "
                              "it cool. Support the arm.")

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
                        # Recomputing this after pinning target to pos flips it every
                        # tick, making the badge alternate and contradict itself.
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

            draw_panel(ui, arm, names, pos, target, loads, temps, volts, torque,
                       errors, last_ok, blocked_dir, sel, speed_i, status, stopped,
                       moving, focused, rec, time.time())

            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if rec is not None:
            rec.close()
        pygame.quit()
        # A successful park is only worth anything if the arm is still held afterwards;
        # releasing here would drop it straight out of the pose it just reached.
        parked = park is not None and park.done and park.outcome == ARRIVED
        if parked and getattr(park, "released", False):
            parked = False          # already released on purpose; say so honestly below
            released_on_purpose = True
        else:
            released_on_purpose = False
        if not parked:
            for n in names:
                arm.torque(n, False)
        try:
            if arm.robot is not None:
                arm.robot.disconnect()
        except Exception:
            pass
        if released_on_purpose:
            print("parked and released — the arm is resting, motors cool")
        elif parked:
            print("parked — torque left on, the arm is holding its safe pose")
        elif names:
            print("torque released — support the arm before it drops")


if __name__ == "__main__":
    main()
