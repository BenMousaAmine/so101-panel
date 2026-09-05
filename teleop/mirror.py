"""The model as a mirror of the real arm.

`SimArm` replaces the hardware; this one follows it. The real joints are read every frame
and written straight into the model's `qpos`, so the picture shows where the arm actually
is -- including where it sagged, stalled or was pushed by hand.

Physics is deliberately not stepped. `mj_forward` places the bodies for the pose it is
given and stops there; stepping would let the model fall under its own gravity and drift
away from the arm it is supposed to be showing.
"""

import math

import mujoco

from teleop.sim_arm import GRIP_MAX, GRIP_MIN, SCENE


class Mirror:
    """Drives the MuJoCo model from a live `Arm`'s observations."""

    def __init__(self):
        self.view = None
        self.error = None
        try:
            self._model = mujoco.MjModel.from_xml_path(str(SCENE))
        except Exception as e:
            self.error = str(e)[:60]
            return
        self._data = mujoco.MjData(self._model)
        self._qpos: dict[str, int] = {}
        for i in range(self._model.nu):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            jid = int(self._model.actuator_trnid[i, 0])
            self._qpos[name] = int(self._model.jnt_qposadr[jid])

        from teleop.sim_view import SimView
        self.view = SimView(self._model, self._data)
        if self.view.error:
            self.error = self.view.error

    def update(self, obs: dict) -> None:
        """Place the model where the arm is. An empty observation leaves the last pose up."""
        if self.view is None or not obs:
            return
        for joint, adr in self._qpos.items():
            value = obs.get(f"{joint}.pos")
            if value is None:
                continue
            if joint == "gripper":
                frac = min(max(float(value), 0.0), 100.0) / 100.0
                self._data.qpos[adr] = GRIP_MIN + frac * (GRIP_MAX - GRIP_MIN)
            else:
                self._data.qpos[adr] = math.radians(float(value))
        mujoco.mj_forward(self._model, self._data)

    def close(self) -> None:
        if self.view is not None:
            self.view.close()
            self.view = None
