# SPDX-License-Identifier: GPL-3.0
"""Robot remote: drive the robot with the keyboard, like a game.

Hold W/S to walk, A/D to turn, Q/E to step sideways. The window publishes
/cmd_vel only while you hold a key, so "party walk" in a terminal still works.
"""
import threading
import tkinter as tk

import rclpy

from roboparty_sim import party
from roboparty_sim.robot_link import RobotLink, start_spinning

# key: (forward m/s, side m/s, turn rad/s)
KEYS = {
    'w': (0.5, 0.0, 0.0),
    's': (-0.3, 0.0, 0.0),
    'a': (0.0, 0.0, 0.6),
    'd': (0.0, 0.0, -0.6),
    'q': (0.0, 0.3, 0.0),
    'e': (0.0, -0.3, 0.0),
}
HELLO = 'Click Start. Then click here and hold W A S D.'
FELL = 'The robot fell! Click Reset.'
RELEASE_DELAY_MS = 60  # X11 key repeat sends release+press pairs; ignore short gaps


class Remote:

    def __init__(self, link):
        self.link = link
        self.held = set()
        self.pending_release = {}
        self.busy = False
        self.result = None
        self.root = tk.Tk()
        self.root.title('Robot remote')
        self.root.geometry('480x350+800+0')
        self.root.configure(padx=4, pady=4)
        self.status = tk.StringVar(value=HELLO)
        self.speed = tk.StringVar(value='')
        self._build()
        self.root.bind('<KeyPress>', self.on_press)
        self.root.bind('<KeyRelease>', self.on_release)
        self.root.after(100, self.tick)

    def _build(self):
        font = ('DejaVu Sans', 12, 'bold')
        tk.Label(self.root, text='ROBOT REMOTE', font=('DejaVu Sans', 15, 'bold')).pack()
        pad = tk.Frame(self.root)
        pad.pack(pady=3)
        layout = [('Q\nside ←', 'q', 0, 0), ('W\nforward', 'w', 0, 1), ('E\nside →', 'e', 0, 2),
                  ('A\nturn ⟲', 'a', 1, 0), ('S\nback', 's', 1, 1), ('D\nturn ⟳', 'd', 1, 2)]
        for text, key, row, col in layout:
            button = tk.Button(pad, text=text, width=10, height=2, font=font)
            button.grid(row=row, column=col, padx=3, pady=3)
            button.bind('<ButtonPress-1>', lambda _e, k=key: self.held.add(k))
            button.bind('<ButtonRelease-1>', lambda _e, k=key: self.release(k))
        actions = tk.Frame(self.root)
        actions.pack(pady=4)
        buttons = [('Start', party.cmd_start), ('Push', party.cmd_push), ('Reset', party.cmd_reset),
                   ('Wave', party.cmd_wave), ('Dance', lambda link, _a: party.cmd_motion(link, 'dance0')),
                   ('Dance 2', lambda link, _a: party.cmd_motion(link, 'dance1'))]
        for i, (text, func) in enumerate(buttons):
            tk.Button(actions, text=text, width=10, font=font,
                      command=lambda f=func: self.run_action(f)).grid(row=i // 3, column=i % 3, padx=3, pady=2)
        tk.Label(self.root, textvariable=self.speed, font=('DejaVu Sans Mono', 11)).pack(pady=2)
        tk.Label(self.root, textvariable=self.status, font=('DejaVu Sans', 10), wraplength=460).pack()
        tk.Label(self.root, text='SPACE = stop', font=('DejaVu Sans', 9)).pack()

    def on_press(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self.held.clear()
            self.link.send_velocity()
            return
        if key in KEYS:
            if key in self.pending_release:
                self.root.after_cancel(self.pending_release.pop(key))
            self.held.add(key)

    def on_release(self, event):
        key = event.keysym.lower()
        if key in KEYS:
            self.pending_release[key] = self.root.after(RELEASE_DELAY_MS, lambda: self.release(key))

    def release(self, key):
        self.pending_release.pop(key, None)
        self.held.discard(key)
        if not self.held:
            self.link.send_velocity()  # one zero message when you let go

    def tick(self):
        if not rclpy.ok():  # ROS was shut down (Ctrl+C or supervisor stop)
            self.root.destroy()
            return
        if self.held:
            forward = sum(KEYS[k][0] for k in self.held)
            side = sum(KEYS[k][1] for k in self.held)
            turn = sum(KEYS[k][2] for k in self.held)
            msg = self.link.send_velocity(forward, side, turn)
            self.speed.set(f'/cmd_vel  x={msg.linear.x:+.1f}  y={msg.linear.y:+.1f}  turn={msg.angular.z:+.1f}')
        else:
            self.speed.set('/cmd_vel  (not sending)')
        if self.result:
            self.status.set(self.result)
            self.result = None
        if not self.busy:
            if self.link.is_fallen():
                self.status.set(FELL)
            elif self.status.get() == FELL:
                self.status.set(HELLO)
        self.root.after(100, self.tick)

    def run_action(self, func):
        if self.busy:
            return
        self.busy = True
        self.status.set('Working...')

        def work():
            # Tk is not thread-safe: only set plain fields here, tick() shows them.
            code = func(self.link, None)
            self.result = 'Done' if code == 0 else 'Did not work, see /tmp/remote.log'
            self.busy = False

        threading.Thread(target=work, daemon=True).start()


def main():
    rclpy.init()
    link = RobotLink('remote')
    spinner = start_spinning(link)
    try:
        Remote(link).root.mainloop()
    finally:
        spinner.stop(link)


if __name__ == '__main__':
    main()
