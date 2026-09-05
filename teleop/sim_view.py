"""The simulated arm as a picture, drawn inside the panel.

macOS will not let pygame and MuJoCo's native viewer share a process: the viewer demands
`mjpython`, which hands the script a secondary thread, and pygame refuses to open a window
off the main thread. So nothing is launched -- the model is rendered offscreen and blitted
into the panel like any other widget, which keeps one process and one window.
"""

import mujoco
import numpy as np
import pygame

# Wide enough to read the arm's shape, small enough to render every frame.
# MuJoCo's offscreen framebuffer is 640x480 unless the model asks for more, and asking
# for a larger image than that is a hard error rather than a resize.
VIEW_W, VIEW_H = 560, 470

# The arm does not live around its base: it reaches from x=-0.05 to +0.32 m and stands
# 0.26 m tall, so a camera aimed at the origin spends half the frame on empty floor and
# lets the arm leave the picture. These look at the middle of that envelope instead, from
# far enough back to hold the fully extended pose.
LOOKAT = (0.13, 0.0, 0.13)

CAMERAS = [
    # (name, azimuth, elevation, distance)
    ("side", 90.0, -12.0, 0.95),
    ("front", 160.0, -12.0, 0.95),
    ("top", 120.0, -45.0, 1.15),
]


class SimView:
    """Offscreen MuJoCo renderer, cached as a pygame surface."""

    def __init__(self, model, data, width: int = VIEW_W, height: int = VIEW_H):
        self._data = data
        self._cam_i = 0
        self._surface = None
        self.error = None
        try:
            self._renderer = mujoco.Renderer(model, height, width)
        except Exception as e:
            self._renderer = None
            self.error = str(e)[:60]
            return
        self._cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self._cam)
        self._apply_camera()

    @property
    def camera_name(self) -> str:
        return CAMERAS[self._cam_i][0]

    def cycle_camera(self) -> None:
        self._cam_i = (self._cam_i + 1) % len(CAMERAS)
        self._apply_camera()

    def _apply_camera(self) -> None:
        _, az, el, dist = CAMERAS[self._cam_i]
        self._cam.azimuth = az
        self._cam.elevation = el
        self._cam.distance = dist
        self._cam.lookat[:] = LOOKAT

    def frame(self) -> pygame.Surface | None:
        """Render the arm where it stands now. None when the renderer is unavailable."""
        if self._renderer is None:
            return None
        try:
            self._renderer.update_scene(self._data, self._cam)
            px = self._renderer.render()
        except Exception as e:
            self.error = str(e)[:60]
            self._renderer = None
            return None
        # MuJoCo hands back (H, W, 3); pygame wants (W, H, 3).
        self._surface = pygame.surfarray.make_surface(np.transpose(px, (1, 0, 2)))
        return self._surface

    def close(self) -> None:
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None
