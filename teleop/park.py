"""Park the arm in a saved safe pose, one joint at a time.

Nothing else in the panel moves the arm without a key held down, so this is the one
place a mistake can drive the arm into something unattended. It is built to stop rather
than to finish:

  * one joint moves at a time, in wrist-to-base order — the far end folds in before the
    shoulder lowers, so the forearm never sweeps the table on the way down
  * a joint that stalls, strains or overshoots ends the whole sequence; later joints are
    never attempted on top of a failure
  * torque is released at the end **only if the pose is a resting one** — the operator
    decides. A pose the arm rests on needs no motors, and holding one costs heat: the
    joint that cooked in testing was one held under load for a long time.

The pose is recorded from the arm itself (`capture`), never written by hand.
"""

import json
import math
import pathlib

POSE_PATH = pathlib.Path(__file__).with_name("safe_pose.json")

# Wrist first, base last: fold the far end in before lowering what carries the weight.
ORDER = ("gripper", "wrist_roll", "wrist_flex", "elbow_flex", "shoulder_lift",
         "shoulder_pan")

SPEED = 10.0        # deg/s — half the panel's usual working speed

# Six motors accelerating together is the heaviest draw the arm ever makes, and at 5.2V
# the rail sags under far less. A floor on the duration caps how fast any joint may go
# when they all move at once.
MIN_DURATION = 3.0  # seconds, however short the longest move is
TOLERANCE = 2.0     # deg; closer than this counts as arrived
STALL_FRAMES = 45   # ~1.5s at 30fps with no progress and the joint has stopped
MIN_PROGRESS = 0.4  # deg of movement that counts as progress
MAX_LAG = 6.0       # commanded position may not outrun the real one by more than this
LOAD_LIMIT = 800    # below the 950 the panel stops at, and well below the 1000 ceiling

# The motors shut themselves off the bus at 70C. Cutting torque well before that keeps a
# joint under the manufacturer's protection rather than at it: shoulder_lift reached 68C
# during testing and had to remove itself to survive.
TEMP_WARN = 50
TEMP_CUT = 58

ARRIVED, STALLED, STRAINED, ABORTED, HOT = ("arrived", "stalled", "strained",
                                            "aborted", "too hot")


def load_pose() -> dict[str, float]:
    try:
        return json.loads(POSE_PATH.read_text())
    except Exception:
        return {}


def save_pose(pose: dict[str, float]) -> None:
    POSE_PATH.write_text(json.dumps(pose, indent=4, sort_keys=True) + "\n")


def capture(arm) -> dict[str, float]:
    """Take the arm's present pose as the safe one. Nothing moves."""
    obs = arm.observation()
    return {m: round(float(obs[f"{m}.pos"]), 1) for m in arm.names if f"{m}.pos" in obs}


def unreachable(pose: dict[str, float]) -> list[tuple[str, float, float]]:
    """Joints of a pose that sit outside their calibrated travel.

    Judged against the calibrated range, the same thing the bars draw. A pose only a
    degree past the driving margin is still reachable — gravity puts the arm there — and
    refusing it would contradict the panel's own display.
    """
    from teleop.limits import SCALE
    out = []
    for j, want in pose.items():
        lo, hi = SCALE.get(j, (want, want))
        if want < lo:
            out.append((j, want, lo))
        elif want > hi:
            out.append((j, want, hi))
    return out


class Park:
    """The parking run as a state the panel can draw, step and stop.

    `step()` is called once per frame and moves the current joint a little closer. It
    never blocks, so the panel keeps drawing and the operator keeps ESC.
    """

    def __init__(self, arm, pose: dict[str, float], dt: float):
        self.arm = arm
        self.pose = pose
        self.dt = dt
        self.joints = [j for j in ORDER if j in pose and j in arm.names]
        self.i = 0
        self.done = False
        self.outcome = ""
        self.detail = ""
        self.results: list[tuple[str, str]] = []
        self.target = None
        self._best = None
        # Counted in frames, not seconds: wall-clock makes this untestable and ties the
        # stall detector to how fast the loop happens to be running.
        self._since_progress = 0

    @property
    def joint(self) -> str | None:
        return self.joints[self.i] if self.i < len(self.joints) else None

    def _finish(self, outcome: str, detail: str) -> None:
        self.done = True
        self.outcome = outcome
        self.detail = detail

    def abort(self) -> None:
        """Stop where we are. Whatever is holding stays holding."""
        if not self.done:
            self.results.append((self.joint or "-", ABORTED))
            self._finish(ABORTED, "stopped by the operator")

    def step(self) -> None:
        j = self.joint
        if self.done or j is None:
            return

        obs = self.arm.observation()
        key = f"{j}.pos"
        if key not in obs:
            self.results.append((j, STALLED))
            self._finish(STALLED, f"{j} stopped answering")
            return
        pos = float(obs[key])
        want = self.pose[j]

        if self.target is None:
            self.target = pos
            self._best = pos
            self._since_progress = 0

        load = self.arm.read("Present_Load", j)
        if load is not None and abs(load) > LOAD_LIMIT:
            self.results.append((j, STRAINED))
            self._finish(STRAINED, f"{j} pushing at load {abs(load)} — something is in the way")
            return

        temp = self.arm.read("Present_Temperature", j)
        if temp is not None and temp >= TEMP_CUT:
            self.results.append((j, HOT))
            self._finish(HOT, f"{j} at {temp}C — stopped before it overheats")
            return

        if abs(pos - want) <= TOLERANCE:
            self.results.append((j, ARRIVED))
            self._next()
            return

        if abs(pos - self._best) >= MIN_PROGRESS:
            self._best = pos
            self._since_progress = 0
        else:
            self._since_progress += 1
        if self._since_progress > STALL_FRAMES:
            self.results.append((j, STALLED))
            self._finish(STALLED, f"{j} stopped {abs(pos - want):.0f} deg short")
            return

        direction = 1.0 if want > pos else -1.0
        nxt = self.target + direction * SPEED * self.dt
        # Clamp only in the direction of travel. Clamping both ways *pushed* the command
        # out to MAX_LAG whenever the joint fell behind, so it sat a full 8 deg ahead and
        # the motor pulled 845 of load all the way to the target — the lag is a ceiling,
        # not a set point.
        if direction > 0:
            nxt = min(nxt, pos + MAX_LAG, want)
        else:
            nxt = max(nxt, pos - MAX_LAG, want)
        self.target = nxt
        self.arm.send({key: nxt})

    def _next(self) -> None:
        self.i += 1
        self.target = None
        self._best = None
        # Counted in frames, not seconds: wall-clock makes this untestable and ties the
        # stall detector to how fast the loop happens to be running.
        self._since_progress = 0
        if self.i >= len(self.joints):
            self._finish(ARRIVED, "parked — torque left on")


class ParkTogether:
    """Every joint moving at once, timed so they arrive together.

    The one-at-a-time sequence is safe because the arm only ever passes through poses the
    order was chosen for. Moving together throws that away unless the joints are
    *synchronised*: give them all the same duration and let the one with furthest to go
    set the pace, so the arm sweeps one predictable path instead of six independent ones.

    Any joint in trouble stops **all** of them. A joint that keeps driving while another
    has jammed is how a coordinated move turns into a collision.
    """

    def __init__(self, arm, pose: dict[str, float], dt: float):
        self.arm = arm
        self.pose = pose
        self.dt = dt
        self.joints = [j for j in ORDER if j in pose and j in arm.names]
        self.done = False
        self.outcome = ""
        self.detail = ""
        self.results: list[tuple[str, str]] = []
        self.elapsed = 0.0
        self.start: dict[str, float] = {}
        self.duration = MIN_DURATION
        self._best: dict[str, float] = {}
        self._since_progress = 0

    @property
    def joint(self) -> str | None:
        """The joint with furthest still to go — the one setting the pace."""
        if self.done or not self.start:
            return None
        obs = self.arm.observation()
        remaining = {j: abs(self.pose[j] - float(obs.get(f"{j}.pos", self.pose[j])))
                     for j in self.joints}
        return max(remaining, key=remaining.get) if remaining else None

    @property
    def progress(self) -> float:
        return 0.0 if not self.duration else min(1.0, self.elapsed / self.duration)

    def _finish(self, outcome: str, detail: str) -> None:
        self.done = True
        self.outcome = outcome
        self.detail = detail

    def _stop_all(self, culprit: str, outcome: str, detail: str) -> None:
        """Hold every joint where it is, then end. Nothing keeps moving."""
        obs = self.arm.observation()
        self.arm.send({f"{j}.pos": float(obs[f"{j}.pos"])
                       for j in self.joints if f"{j}.pos" in obs})
        for j in self.joints:
            self.results.append((j, outcome if j == culprit else ABORTED))
        self._finish(outcome, detail)

    def abort(self) -> None:
        if not self.done:
            self._stop_all("", ABORTED, "stopped by the operator")

    def _begin(self) -> None:
        obs = self.arm.observation()
        self.start = {j: float(obs[f"{j}.pos"]) for j in self.joints if f"{j}.pos" in obs}
        furthest = max((abs(self.pose[j] - self.start[j]) for j in self.start), default=0.0)
        self.duration = max(MIN_DURATION, furthest / SPEED)
        self._best = dict(self.start)

    def step(self) -> None:
        if self.done:
            return
        if not self.start:
            self._begin()
            if not self.start:
                self._finish(STALLED, "the arm did not answer")
            return

        obs = self.arm.observation()
        if not obs:
            self._stop_all("", STALLED, "the arm stopped answering")
            return

        for j in self.joints:
            load = self.arm.read("Present_Load", j)
            if load is not None and abs(load) > LOAD_LIMIT:
                self._stop_all(j, STRAINED,
                               f"{j} at load {abs(load)} — everything stopped")
                return
            temp = self.arm.read("Present_Temperature", j)
            if temp is not None and temp >= TEMP_CUT:
                self._stop_all(j, HOT, f"{j} at {temp}C — everything stopped")
                return

        moved = any(abs(float(obs.get(f"{j}.pos", self._best[j])) - self._best[j])
                    >= MIN_PROGRESS for j in self.joints)
        if moved:
            for j in self.joints:
                self._best[j] = float(obs.get(f"{j}.pos", self._best[j]))
            self._since_progress = 0
        else:
            self._since_progress += 1
        if self._since_progress > STALL_FRAMES:
            self._stop_all("", STALLED, "nothing is moving — everything stopped")
            return

        self.elapsed += self.dt
        a = self.progress
        # Cosine ease: no jolt at either end, and the peak rate stays well under a
        # constant-speed move of the same duration.
        blend = (1 - math.cos(math.pi * a)) / 2

        goal = {}
        for j in self.joints:
            want = self.start[j] + (self.pose[j] - self.start[j]) * blend
            now = float(obs.get(f"{j}.pos", want))
            # As above: hold the command back to MAX_LAG, never push it out to it.
            goal[f"{j}.pos"] = (min(want, now + MAX_LAG) if want > now
                                else max(want, now - MAX_LAG))
        self.arm.send(goal)

        if a >= 1.0:
            short = [j for j in self.joints
                     if abs(float(obs.get(f"{j}.pos", self.pose[j])) - self.pose[j])
                     > TOLERANCE]
            for j in self.joints:
                self.results.append((j, STALLED if j in short else ARRIVED))
            if short:
                self._finish(STALLED, f"{', '.join(short)} stopped short")
            else:
                self._finish(ARRIVED, "parked — all joints together")
