"""A 3-button colour picker for the sorter, with live validation.

Each button sends a colour on /target_color; the color_sort node picks that part
onto the conveyor. The GUI subscribes to /cell/telemetry so it can validate
before sending and give feedback:
  - if the cell is busy (a sort is running), a second press is refused with a
    warning instead of queueing another order,
  - if the requested colour is not on the board, it is refused with a warning,
  - buttons grey out when the cell is busy or that colour is unavailable.

    ros2 run armik_moveit sort_gui
"""
import json
import os
import tkinter as tk

import rclpy
from std_msgs.msg import String

BUTTONS = [("RED", "red", "#d92626"), ("GREEN", "green", "#2ca02c"),
           ("BLUE", "blue", "#2659d9")]
OK, WARN, MUTE = "#2ea043", "#d29922", "#8b949e"


class SortGui:
    def __init__(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        rclpy.init()
        self.node = rclpy.create_node("sort_gui")
        self.pub = self.node.create_publisher(String, "/target_color", 10)
        self.node.create_subscription(String, "/cell/telemetry", self._on_tele, 10)
        self.state = "?"
        self.busy = False
        self.available = set()
        self._build()

    def _on_tele(self, msg):
        try:
            t = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.state = t.get("state", "?")
        self.busy = bool(t.get("busy", self.state == "sorting"))
        self.available = set(t.get("available", []))

    def _build(self):
        self.root = tk.Tk()
        self.root.title("Colour Sorter")
        self.root.configure(bg="#0e1116")
        tk.Label(self.root, text="Send a part to the conveyor", bg="#0e1116",
                 fg="#e6edf3", font=("Sans", 13)).pack(padx=18, pady=(16, 8))
        self.buttons = {}
        for label, color, bg in BUTTONS:
            b = tk.Button(self.root, text=label, bg=bg, fg="white",
                          activebackground=bg, font=("Sans", 14, "bold"),
                          width=16, height=2, relief="raised", bd=0,
                          disabledforeground="#c9d1d9",
                          command=lambda c=color: self._press(c))
            b.pack(padx=18, pady=5)
            self.buttons[color] = (b, bg)
        self.status = tk.Label(self.root, text="waiting for the cell ...",
                               bg="#0e1116", fg=MUTE, font=("Sans", 11),
                               wraplength=240, justify="center")
        self.status.pack(padx=18, pady=(10, 16))

    def _set_status(self, text, color):
        self.status.config(text=text, fg=color)

    def _press(self, color):
        if self.busy:
            self._set_status("Cell is busy — wait for the current sort to finish.", WARN)
            return
        if color not in self.available:
            self._set_status(f"No {color.upper()} object on the board.", WARN)
            return
        self.pub.publish(String(data=color))
        self._set_status(f"Sorting {color.upper()} ...", OK)

    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for color, (btn, bg) in self.buttons.items():
            enabled = (not self.busy) and (color in self.available)
            btn.config(state="normal" if enabled else "disabled",
                       bg=bg if enabled else "#30363d")
        if not self.busy and self.status.cget("fg") == OK and self.state == "idle":
            self._set_status("Ready. Pick a colour to sort.", MUTE)
        elif self.busy and self.status.cget("fg") != WARN:
            self._set_status(f"Cell busy: sorting {self.state} ...", MUTE)
        self.root.after(200, self._tick)

    def run(self):
        self.root.after(200, self._tick)
        try:
            self.root.mainloop()
        finally:
            self.node.destroy_node()
            rclpy.try_shutdown()


def main():
    SortGui().run()


if __name__ == "__main__":
    main()
