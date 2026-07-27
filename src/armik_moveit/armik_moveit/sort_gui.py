"""Operator panel for the colour-sorting cell.

Colour buttons command sorts; a big EMERGENCY STOP button trips the safety layer
and a RESET / ON button releases and re-arms it. Everything is validated against
live telemetry:
  - colour buttons refuse a second order while busy, refuse an absent colour, and
    grey out when the cell is busy / not safe / that colour is gone,
  - EMERGENCY STOP asserts /safety/estop (latched); RESET / ON releases it and
    issues /safety/reset so the cell returns to RUN.

    ros2 run armik_moveit sort_gui
"""
import json
import os
import tkinter as tk

import rclpy
from std_msgs.msg import Bool, String

BUTTONS = [("RED", "red", "#d92626"), ("GREEN", "green", "#2ca02c"),
           ("BLUE", "blue", "#2659d9")]
OK, WARN, MUTE, BAD = "#2ea043", "#d29922", "#8b949e", "#d92626"
SAFE_COLOURS = {"RUN": OK, "REDUCED": WARN, "GUARD_STOP": BAD,
                "ESTOP": BAD, "FAULT": BAD, "INIT": MUTE}


class SortGui:
    def __init__(self):
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
        rclpy.init()
        self.node = rclpy.create_node("sort_gui")
        self.pub = self.node.create_publisher(String, "/target_color", 10)
        self.estop_pub = self.node.create_publisher(Bool, "/safety/estop", 10)
        self.reset_pub = self.node.create_publisher(Bool, "/safety/reset", 10)
        self.node.create_subscription(String, "/cell/telemetry", self._on_tele, 10)
        self.state = "?"
        self.busy = False
        self.available = set()
        self.safety_state = "INIT"
        self.clear = False
        self._build()

    def _on_tele(self, msg):
        try:
            t = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.state = t.get("state", "?")
        self.busy = bool(t.get("busy", self.state == "sorting"))
        self.available = set(t.get("available", []))
        self.safety_state = t.get("safety_state", "INIT")
        self.clear = bool(t.get("clear_to_run", False))

    def _build(self):
        self.root = tk.Tk()
        self.root.title("Colour Sorter — Operator Panel")
        self.root.configure(bg="#0e1116")
        self.safety_lbl = tk.Label(self.root, text="SAFETY: --", bg="#161b22",
                                   fg="#e6edf3", font=("Sans", 12, "bold"),
                                   width=26, pady=8)
        self.safety_lbl.pack(padx=16, pady=(14, 6), fill="x")
        tk.Label(self.root, text="Send a part to the conveyor", bg="#0e1116",
                 fg="#e6edf3", font=("Sans", 12)).pack(padx=16, pady=(8, 4))
        self.buttons = {}
        for label, color, bg in BUTTONS:
            b = tk.Button(self.root, text=label, bg=bg, fg="white",
                          activebackground=bg, font=("Sans", 14, "bold"),
                          width=18, height=1, relief="raised", bd=0,
                          disabledforeground="#c9d1d9",
                          command=lambda c=color: self._press(c))
            b.pack(padx=16, pady=4)
            self.buttons[color] = (b, bg)
        # safety controls
        row = tk.Frame(self.root, bg="#0e1116")
        row.pack(padx=16, pady=(12, 6), fill="x")
        self.estop_btn = tk.Button(row, text="⏻  EMERGENCY STOP", bg=BAD,
                                   fg="white", activebackground="#a31d1d",
                                   font=("Sans", 13, "bold"), height=2, bd=0,
                                   command=self._estop)
        self.estop_btn.pack(side="left", expand=True, fill="x", padx=(0, 4))
        self.on_btn = tk.Button(row, text="↻  RESET / ON", bg=OK,
                                fg="white", activebackground="#22803a",
                                font=("Sans", 13, "bold"), height=2, bd=0,
                                command=self._reset_on)
        self.on_btn.pack(side="left", expand=True, fill="x", padx=(4, 0))
        self.status = tk.Label(self.root, text="waiting for the cell ...",
                               bg="#0e1116", fg=MUTE, font=("Sans", 11),
                               wraplength=280, justify="center")
        self.status.pack(padx=16, pady=(8, 16))

    def _set_status(self, text, color):
        self.status.config(text=text, fg=color)

    def _press(self, color):
        if not self.clear:
            self._set_status(f"Cell not safe to run ({self.safety_state}).", WARN)
            return
        if self.busy:
            self._set_status("Cell is busy — wait for the current sort.", WARN)
            return
        if color not in self.available:
            self._set_status(f"No {color.upper()} object on the board.", WARN)
            return
        self.pub.publish(String(data=color))
        self._set_status(f"Sorting {color.upper()} ...", OK)

    def _estop(self):
        self.estop_pub.publish(Bool(data=True))
        self._set_status("EMERGENCY STOP asserted.", BAD)

    def _reset_on(self):
        self.estop_pub.publish(Bool(data=False))          # release
        self.root.after(300, lambda: self.reset_pub.publish(Bool(data=True)))  # then reset
        self._set_status("Reset — re-arming the cell.", OK)

    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.safety_lbl.config(text=f"SAFETY: {self.safety_state}",
                               bg=SAFE_COLOURS.get(self.safety_state, "#161b22"))
        for color, (btn, bg) in self.buttons.items():
            enabled = self.clear and (not self.busy) and (color in self.available)
            btn.config(state="normal" if enabled else "disabled",
                       bg=bg if enabled else "#30363d")
        cur = self.status.cget("fg")
        if not self.clear and cur not in (BAD, WARN):
            self._set_status(f"Cell held: {self.safety_state}.", WARN)
        elif self.clear and self.busy and cur != WARN:
            self._set_status(f"Cell busy: sorting {self.state} ...", MUTE)
        elif self.clear and not self.busy and cur == OK and self.state == "idle":
            self._set_status("Ready. Pick a colour to sort.", MUTE)
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
