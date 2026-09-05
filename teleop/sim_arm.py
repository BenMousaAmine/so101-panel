"""The arm as pixels, not as hardware.

A stand-in for `Arm` that drives the MuJoCo model in `assets/SO101/` instead of the
Feetech bus, so the panel can be flown without the real arm on the desk. It answers the
same calls `Arm` does and reaches READY on its own: there is no port to find, no
calibration to disagree with, and nothing that can sag.

The panel speaks degrees from the centre of travel (the gripper 0-100%), MuJoCo speaks
radians. Every conversion lives here -- that is the whole point of the file.
"""

import math
import pathlib
import time

import mujoco

SCENE = pathlib.Path(__file__).resolve().parent.parent / "assets" / "SO101" / "scene.xml"

# The panel's gripper is a percentage; the model's is an angle with a little squeeze
# past closed. Mapping the ends onto each other keeps 0 shut and 100 wide open.
GRIP_MIN, GRIP_MAX = math.radians(-10.0), math.radians(100.0)


class SimArm:
    """Same surface as `Arm`, backed by a physics model."""

    NO_PORT = "no_port"
    CONNECTING = "connecting"
    MISMATCH = "mismatch"
    LIMP = "limp"
    UNCALIBRATED = "uncalibrated"
    CALIBRATING = "calibrating"
    READY = "ready"
    FAILED = "failed"

    # The panel steps physics itself once per frame; this is that frame.
    FRAME = 1.0 / 60.0

    def __init__(self, viewer: bool = True):
        self.robot = None          # no LeRobot robot exists; the recorder is told to skip
        self.wizard = None
        self.port = "mujoco"
        self.names: list[str] = []
        self.state = self.CONNECTING
        self.detail = "loading the model"
        self.wrong: list[tuple[str, int, int]] = []
        self.ports: list[str] = ["mujoco"]
        self._last_try = 0.0
        self._want_viewer = viewer
        self.view = None           # a SimView the panel draws, not a window of its own
        self._model = None
        self._data = None
        self._torque: dict[str, bool] = {}
        self._span: dict[str, float] = {}   # joint -> half travel, radians
        self._act: dict[str, int] = {}      # joint -> actuator id
        self._qpos: dict[str, int] = {}     # joint -> qpos address
        self._t0 = time.time()
        self._load()

    # -- setup ---------------------------------------------------------------

    def _load(self) -> None:
        try:
            self._model = mujoco.MjModel.from_xml_path(str(SCENE))
        except Exception as e:
            self.state = self.FAILED
            self.detail = str(e)[:70]
            return
        self._data = mujoco.MjData(self._model)

        m = self._model
        for i in range(m.nu):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            jid = int(m.actuator_trnid[i, 0])
            self.names.append(name)
            self._act[name] = i
            self._qpos[name] = int(m.jnt_qposadr[jid])
            lo, hi = m.jnt_range[jid]
            self._span[name] = float(max(abs(lo), abs(hi)))
        self._seed_scales()
        self._torque = dict.fromkeys(self.names, True)
        self._gain0 = {i: float(m.actuator_gainprm[i, 0]) for i in range(m.nu)}
        self._bias0 = {i: float(m.actuator_biasprm[i, 1]) for i in range(m.nu)}
        self._damp0 = {i: float(m.actuator_biasprm[i, 2]) for i in range(m.nu)}

        # Start folded rather than splayed across the floor, so the first frame looks
        # like an arm at rest instead of a crash.
        mujoco.mj_forward(m, self._data)
        for n in self.names:
            self._data.ctrl[self._act[n]] = self._data.qpos[self._qpos[n]]

        if self._want_viewer:
            # Not MuJoCo's own window: on macOS that needs `mjpython`, which then denies
            # pygame the main thread it needs for the panel. Drawn into the panel instead.
            from teleop.sim_view import SimView
            self.view = SimView(self._model, self._data)
            # A view that failed is kept, not discarded: it draws its own error in the
            # panel. Silently dropping it is what makes a missing 3D column look like a
            # missing feature instead of a bug.

        # Deliberately not READY yet. The panel arms the arm on the *transition* into
        # live, inside its own loop; a model that is ready before the loop starts never
        # makes that transition and leaves the panel with no joints to draw.
        self.state = self.CONNECTING
        self.detail = ("simulated arm — MuJoCo" if self.view is not None
                       else "simulated arm — headless")

    def _seed_scales(self) -> None:
        """Lend the panel the model's own travel when no calibration file exists.

        The scales normally come from the arm's calibration, and --sim is exactly the case
        where there is no arm to have calibrated: without this the panel divides by an
        empty dict on its first frame. A real calibration, when present, is left alone --
        it describes the arm being mirrored, and the model should not overrule it.
        """
        from teleop.limits import SCALE, TRAVEL, MARGIN
        if SCALE:
            return
        for n in self.names:
            if n == "gripper":
                SCALE[n], TRAVEL[n] = (0.0, 100.0), (MARGIN, 100.0 - MARGIN)
                continue
            half = math.degrees(self._span[n])
            SCALE[n] = (-half, half)
            TRAVEL[n] = (-half + MARGIN, half - MARGIN)

    # -- units ---------------------------------------------------------------

    def _to_model(self, joint: str, value: float) -> float:
        """Panel units -> radians."""
        if joint == "gripper":
            frac = min(max(value, 0.0), 100.0) / 100.0
            return GRIP_MIN + frac * (GRIP_MAX - GRIP_MIN)
        return math.radians(value)

    def _from_model(self, joint: str, value: float) -> float:
        """Radians -> panel units."""
        if joint == "gripper":
            frac = (value - GRIP_MIN) / (GRIP_MAX - GRIP_MIN)
            return min(max(frac, 0.0), 1.0) * 100.0
        return math.degrees(value)

    # -- the Arm surface -----------------------------------------------------

    @property
    def live(self) -> bool:
        return self.state == self.READY and self._model is not None

    def poll(self) -> None:
        """Advance physics. `Arm` looks for a port here; this only has to come up once."""
        if self.state == self.CONNECTING and self._model is not None:
            self.state = self.READY
        if not self.live:
            return
        steps = max(1, int(self.FRAME / self._model.opt.timestep))
        for _ in range(steps):
            mujoco.mj_step(self._model, self._data)

    def observation(self) -> dict:
        if not self.live:
            return {}
        return {f"{n}.pos": self._from_model(n, float(self._data.qpos[self._qpos[n]]))
                for n in self.names}

    def send(self, action: dict) -> None:
        """Position targets, in panel units. A limp joint ignores them, as a real one does."""
        if not self.live or not action:
            return
        for key, value in action.items():
            joint = key[:-4] if key.endswith(".pos") else key
            if joint not in self._act or not self._torque.get(joint, True):
                continue
            i = self._act[joint]
            lo, hi = self._model.actuator_ctrlrange[i]
            self._data.ctrl[i] = min(max(self._to_model(joint, float(value)), lo), hi)

    def torque(self, joint: str, on: bool) -> None:
        """Cutting torque hands the joint to gravity, which is what makes it worth testing.

        A position servo goes slack by losing its gain, not by being told to go to zero --
        that would drive the joint to the middle of its travel instead of releasing it.
        """
        if not self.live or joint not in self._act:
            return
        self._torque[joint] = on
        i = self._act[joint]
        if on:
            self._data.ctrl[i] = self._data.qpos[self._qpos[joint]]
        self._model.actuator_gainprm[i, 0] = self._gain0[i] if on else 0.0
        # biasprm[1] is the servo's proportional pull towards its target. Leaving it in
        # place while the gain is zero does not free the joint -- it clamps it to zero.
        self._model.actuator_biasprm[i, 1] = self._bias0[i] if on else 0.0
        self._model.actuator_biasprm[i, 2] = self._damp0[i] if on else 0.0

    def read(self, field: str, joint: str):
        """Telemetry the panel draws. Load is real; the climate is a plausible stand-in."""
        if not self.live or joint not in self._act:
            return None
        if field == "Present_Load":
            return int(abs(self._data.actuator_force[self._act[joint]]) * 100)
        if field == "Present_Temperature":
            # Warms from room temperature towards a steady state, so the gauge moves.
            return int(min(45, 24 + (time.time() - self._t0) / 60))
        if field == "Present_Voltage":
            return 118        # 11.8 V; the panel divides by ten
        if field == "Present_Position":
            return int(self._data.qpos[self._qpos[joint]] / (2 * math.pi) * 4095)
        return None

    # -- states the simulation cannot reach ----------------------------------

    def restore_calibration(self) -> bool:
        return True

    def recheck(self) -> list[tuple[str, int, int]]:
        return []

    def start_calibration(self) -> None:
        """Nothing to calibrate: the model is already true by construction."""
        self.detail = "nothing to calibrate in simulation"

    def finish_calibration(self) -> None:
        self.wizard = None

    def drop(self, why: str) -> None:
        self.state = self.FAILED
        self.detail = why
        self.close()

    def close(self) -> None:
        if self.view is not None:
            self.view.close()
            self.view = None
