# SPDX-License-Identifier: GPL-3.0
"""A light MuJoCo window that draws at a fixed frame rate.

mujoco.viewer draws as fast as it can. In Docker there is no GPU, so the CPU
draws every frame, and that uses a lot of power. This window draws only
`fps` times each second, has no shadows, and can draw a lighter copy of the
model (view_model) with simpler meshes. Physics keeps the full model.

Mouse: left drag = rotate, right drag = move, scroll = zoom.
"""
import time

import glfw
import mujoco


class Viewer:

    def __init__(self, model, data, lock, title='MuJoCo : rpo', width=800, height=690,
                 fps=15.0, track_body=-1, key_callback=None, view_model=None):
        self.sim_data, self.lock = data, lock
        # Draw view_model (same bodies and joints, simpler meshes) if we have one.
        self.model = view_model or model
        self.data = mujoco.MjData(self.model)
        self.period = 1.0 / fps
        self.key_callback = key_callback
        if not glfw.init():
            raise RuntimeError('Could not start GLFW. Is DISPLAY set?')
        self.window = glfw.create_window(width, height, title, None, None)
        if not self.window:
            raise RuntimeError('Could not open the MuJoCo window')
        glfw.make_context_current(self.window)
        glfw.swap_interval(0)  # we wait ourselves; there is no vsync in Xvfb

        self.scene = mujoco.MjvScene(self.model, maxgeom=2000)
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0      # shadows are slow without a GPU
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_100)
        self.option = mujoco.MjvOption()
        self.perturb = mujoco.MjvPerturb()
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.trackbodyid = track_body
        self.camera.distance = 3.0
        self.camera.elevation = -15.0
        self.camera.azimuth = 135.0

        self._buttons = {glfw.MOUSE_BUTTON_LEFT: False, glfw.MOUSE_BUTTON_RIGHT: False}
        self._last_cursor = (0.0, 0.0)
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
        glfw.set_cursor_pos_callback(self.window, self._on_cursor)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    def run(self, should_stop):
        """Draw until the window closes or should_stop() is True. Main thread only."""
        try:
            next_frame = time.perf_counter()
            while not glfw.window_should_close(self.window) and not should_stop():
                glfw.poll_events()
                with self.lock:
                    self.data.qpos[:] = self.sim_data.qpos
                    self.data.time = self.sim_data.time
                # Only the parts of mj_forward that drawing needs: body poses,
                # centers of mass (the tracking camera follows them), lights.
                mujoco.mj_kinematics(self.model, self.data)
                mujoco.mj_comPos(self.model, self.data)
                mujoco.mj_camlight(self.model, self.data)
                mujoco.mjv_updateScene(self.model, self.data, self.option, self.perturb,
                                       self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
                width, height = glfw.get_framebuffer_size(self.window)
                viewport = mujoco.MjrRect(0, 0, width, height)
                mujoco.mjr_render(viewport, self.scene, self.context)
                mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                                   viewport, 'H stand   P push   Space pause', '', self.context)
                glfw.swap_buffers(self.window)
                next_frame += self.period
                delay = next_frame - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_frame = time.perf_counter()
        finally:
            self.context.free()
            glfw.destroy_window(self.window)
            glfw.terminate()

    # ----- input ----------------------------------------------------------

    def _on_key(self, _window, key, _scancode, action, _mods):
        if action == glfw.PRESS and self.key_callback:
            self.key_callback(key)

    def _on_mouse_button(self, window, button, action, _mods):
        if button in self._buttons:
            self._buttons[button] = action == glfw.PRESS
        self._last_cursor = glfw.get_cursor_pos(window)

    def _on_cursor(self, window, x, y):
        dx, dy = x - self._last_cursor[0], y - self._last_cursor[1]
        self._last_cursor = (x, y)
        if self._buttons[glfw.MOUSE_BUTTON_LEFT]:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif self._buttons[glfw.MOUSE_BUTTON_RIGHT]:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            return
        height = max(1, glfw.get_window_size(window)[1])
        mujoco.mjv_moveCamera(self.model, action, dx / height, dy / height, self.scene, self.camera)

    def _on_scroll(self, _window, _dx, dy):
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * dy,
                              self.scene, self.camera)
