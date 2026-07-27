"""Colour sorting to a conveyor: press a colour, the arm sends that part.

Reuses the palletizing cell's arm and motion (the same UR5e + Robotiq 2F-85,
pedestal-mounted, Pilz LIN descents + OMPL transfers + top-down grasp). Three
colour-coded parts sit on the supply bin; pressing a button on the 3-colour GUI
(or publishing the colour on /target_color) makes the arm pick that part and
place it on the conveyor belt.

    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py   # bring up the arm
    ros2 run   armik_moveit color_sort                       # the sorter
    ros2 run   armik_moveit sort_gui                          # the 3 buttons
"""
import sys

import rclpy
from std_msgs.msg import String

from armik_moveit.palletizing import HOME, PART_SIZE, Palletizer
from armik_moveit.scene import (
    CLEARANCE, CONVEYOR_DROP, CONVEYOR_TOP, PART_Z, SORT_PARTS, TRANSIT,
    build_sort_scene,
)


class ColorSorter(Palletizer):
    def __init__(self):
        super().__init__()
        self.pending = None
        self.transit_cfg = None
        self.create_subscription(String, "/target_color", self._on_color, 10)

    def _on_color(self, msg):
        self.pending = msg.data.strip().lower()

    def setup(self):
        self._apply(build_sort_scene(clear=False))
        for color, (x, y) in SORT_PARTS.items():
            self.add_part(f"part_{color}", x, y, PART_Z)
        if not self.go_home():
            print("could not reach home; aborting")
            sys.exit(1)
        self.transit_cfg = self.ik_topdown(*TRANSIT) or HOME
        print("colour sorter ready: press RED / GREEN / BLUE "
              "(or publish red|green|blue on /target_color)")

    def sort(self, color):
        if color not in SORT_PARTS:
            print(f"  unknown colour '{color}'")
            return
        x, y = SORT_PARTS[color]
        pid = f"part_{color}"
        self.add_part(pid, x, y, PART_Z)  # (re)spawn at its bin spot
        drop_z = CONVEYOR_TOP + PART_SIZE / 2 + CLEARANCE
        place = (CONVEYOR_DROP[0], CONVEYOR_DROP[1], drop_z)
        print(f"  sorting {color} -> conveyor ...")
        ok = self.pick_place(pid, (x, y, PART_Z), place, self.transit_cfg)
        if ok:
            self._remove_world(pid)  # the conveyor carries the part away
        print(f"  {color}: {'DONE' if ok else 'FAILED'}")


def main():
    rclpy.init()
    node = ColorSorter()
    node.setup()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.pending:
                color, node.pending = node.pending, None
                node.sort(color)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
