"""A 3-button colour picker for the sorter. Each button sends a colour.

Publishes the chosen colour on /target_color (std_msgs/String); the color_sort
node picks that part and places it on the conveyor.

    ros2 run armik_moveit sort_gui
"""
import os
import tkinter as tk

import rclpy
from std_msgs.msg import String

BUTTONS = [("RED", "red", "#d92626"), ("GREEN", "green", "#2ca02c"),
           ("BLUE", "blue", "#2659d9")]


def main():
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")  # harmless; tk uses X11
    rclpy.init()
    node = rclpy.create_node("sort_gui")
    pub = node.create_publisher(String, "/target_color", 10)

    root = tk.Tk()
    root.title("Colour Sorter")
    tk.Label(root, text="Send a part to the conveyor",
             font=("Sans", 12)).pack(padx=16, pady=(14, 6))
    for label, color, bg in BUTTONS:
        tk.Button(root, text=label, bg=bg, fg="white",
                  font=("Sans", 14, "bold"), width=14, height=2, relief="raised",
                  command=lambda c=color: pub.publish(String(data=c))).pack(padx=16, pady=6)
    tk.Label(root, text="(publishes /target_color)", fg="#666").pack(pady=(4, 12))

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
