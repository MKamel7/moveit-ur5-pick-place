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
import json
import sys
import time

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
        # Production telemetry, published on /cell/telemetry for the OPC UA
        # server and the dashboard to consume.
        self.state = "starting"
        self.current = ""
        self.counts = {c: 0 for c in SORT_PARTS}
        self.cycle_times = []
        self.t_start = time.time()
        self.alarm = False
        self.alarm_msg = ""
        self.create_subscription(String, "/target_color", self._on_color, 10)
        self.tele = self.create_publisher(String, "/cell/telemetry", 10)
        self.create_timer(1.0, self.publish_telemetry)

    def _on_color(self, msg):
        self.pending = msg.data.strip().lower()

    def publish_telemetry(self):
        sorted_total = sum(self.counts.values())
        uptime = time.time() - self.t_start
        last_cycle = self.cycle_times[-1] if self.cycle_times else 0.0
        throughput = (sorted_total / uptime * 60.0) if uptime > 0 else 0.0
        tele = {
            "state": self.state,
            "current_color": self.current,
            "parts_sorted": sorted_total,
            "counts": dict(self.counts),
            "last_cycle_s": round(last_cycle, 2),
            "throughput_ppm": round(throughput, 2),
            "uptime_s": round(uptime, 1),
            "alarm": self.alarm,
            "alarm_msg": self.alarm_msg,
        }
        self.tele.publish(String(data=json.dumps(tele)))

    def setup(self):
        self._apply(build_sort_scene(clear=False))
        for color, (x, y) in SORT_PARTS.items():
            self.add_part(f"part_{color}", x, y, PART_Z)
        if not self.go_home():
            self.state, self.alarm, self.alarm_msg = "fault", True, "home unreachable"
            self.publish_telemetry()
            print("could not reach home; aborting")
            sys.exit(1)
        self.transit_cfg = self.ik_topdown(*TRANSIT) or HOME
        self.state = "idle"
        self.publish_telemetry()
        print("colour sorter ready: press RED / GREEN / BLUE "
              "(or publish red|green|blue on /target_color)")

    def _convey_away(self, pid, place):
        """Move the placed part along the belt (+y) and off the far end."""
        x, y0, z = place
        for k in range(1, 8):
            self.add_part(pid, x, y0 + 0.24 * k / 7.0, z)  # travel down the belt
            time.sleep(0.18)
        self._remove_world(pid)

    def sort(self, color):
        if color not in SORT_PARTS:
            self.alarm, self.alarm_msg = True, f"unknown colour '{color}'"
            print(f"  unknown colour '{color}'")
            return
        self.alarm, self.alarm_msg = False, ""
        self.state, self.current = "sorting", color
        self.publish_telemetry()
        x, y = SORT_PARTS[color]
        pid = f"part_{color}"
        self.add_part(pid, x, y, PART_Z)  # (re)spawn at its bin spot
        drop_z = CONVEYOR_TOP + PART_SIZE / 2 + CLEARANCE
        place = (CONVEYOR_DROP[0], CONVEYOR_DROP[1], drop_z)
        print(f"  sorting {color} -> conveyor ...")
        t0 = time.time()
        ok = self.pick_place(pid, (x, y, PART_Z), place, self.transit_cfg)
        if ok:
            self._convey_away(pid, place)  # the belt carries the part down and off
            self.counts[color] += 1
            self.cycle_times.append(time.time() - t0)
        else:
            self.alarm, self.alarm_msg = True, f"{color} pick-place failed"
        self.state, self.current = "idle", ""
        self.publish_telemetry()
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
