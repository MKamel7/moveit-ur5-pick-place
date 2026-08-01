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
import math
import random
import sys
import time

import rclpy
from std_msgs.msg import Bool, String

from armik_moveit.palletizing import HOME, PART_SIZE, Palletizer
from armik_moveit.scene import (
    BIN_AREA,
    CLEARANCE,
    CONVEYOR_TOP,
    PART_Z,
    SORT_COLOURS,
    TRANSIT,
    build_sort_scene,
    lane_axis,
    lane_drop,
    lane_end,
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
        self.counts = {c: 0 for c in SORT_COLOURS}
        self.part_pos = {}      # colour -> (x, y) currently on the board
        self.available = set()  # colours present on the board
        self.cycle_times = []
        self.t_start = time.time()
        self.alarm = False
        self.alarm_msg = ""
        # safety state, driven by the safety_supervisor
        self.safe = True
        self.safety_state = "RUN"
        self.safety_reason = "ok"
        self.speed_scale = 1.0
        # A part gripped through a safety stop is released by the operator's
        # RESET / ON, which is what publishes /safety/reset.
        self.release_pending = False
        self.create_subscription(String, "/target_color", self._on_color, 10)
        self.create_subscription(String, "/safety/state", self._on_safety, 10)
        self.create_subscription(Bool, "/safety/reset", self._on_reset, 10)
        self.tele = self.create_publisher(String, "/cell/telemetry", 10)
        self.create_timer(1.0, self.publish_telemetry)

    def _on_color(self, msg):
        self.pending = msg.data.strip().lower()

    def _on_reset(self, msg):
        # Flag only: releasing drives the gripper and edits the planning scene,
        # which the main loop does between cycles rather than in a callback.
        if msg.data:
            self.release_pending = True

    def do_release(self):
        """Let go of a part that a safety stop left in the gripper."""
        self.release_pending = False
        part_id = self.release_held()
        if part_id is None:
            return
        self.alarm, self.alarm_msg = False, ""
        self.state, self.current = "idle", ""
        if not self.available:      # that part is gone; feed a fresh batch
            self._random_batch()
        self.publish_telemetry()
        print(f"  reset: released {part_id} from the gripper")

    def _on_safety(self, msg):
        try:
            s = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.safe = bool(s.get("clear_to_run", True))
        self.safety_state = s.get("state", "?")
        self.safety_reason = s.get("reason", "")
        self.speed_scale = float(s.get("speed_scale", 1.0))

    def publish_telemetry(self):
        sorted_total = sum(self.counts.values())
        uptime = time.time() - self.t_start
        last_cycle = self.cycle_times[-1] if self.cycle_times else 0.0
        throughput = (sorted_total / uptime * 60.0) if uptime > 0 else 0.0
        tele = {
            "state": self.state,
            "current_color": self.current,
            "busy": self.state == "sorting",
            # Part still in the gripper after a safety stop (empty when none).
            "held": self.held_after_abort or "",
            "available": sorted(self.available),
            "parts_sorted": sorted_total,
            "counts": dict(self.counts),
            "last_cycle_s": round(last_cycle, 2),
            "throughput_ppm": round(throughput, 2),
            "uptime_s": round(uptime, 1),
            "alarm": self.alarm,
            "alarm_msg": self.alarm_msg,
            "safety_state": self.safety_state,
            "clear_to_run": self.safe,
            "speed_scale": round(self.speed_scale, 2),
            "safety_reason": self.safety_reason,
        }
        self.tele.publish(String(data=json.dumps(tele)))

    def setup(self):
        self._apply(build_sort_scene(clear=False))
        if not self.go_home():
            self.state, self.alarm, self.alarm_msg = "fault", True, "home unreachable"
            self.publish_telemetry()
            print("could not reach home; aborting")
            sys.exit(1)
        self.transit_cfg = self.ik_topdown(*TRANSIT) or HOME
        self._random_batch()
        self.state = "idle"
        self.publish_telemetry()
        print("colour sorter ready: press RED / GREEN / BLUE "
              "(or publish red|green|blue on /target_color)")

    def _random_batch(self):
        """Spawn the three colours at random, non-overlapping spots on the bin."""
        xmin, xmax, ymin, ymax = BIN_AREA
        pts = []
        for _ in range(300):
            if len(pts) == len(SORT_COLOURS):
                break
            p = (random.uniform(xmin, xmax), random.uniform(ymin, ymax))
            if all(math.hypot(p[0] - q[0], p[1] - q[1]) > 0.09 for q in pts):
                pts.append(p)
        colours = list(SORT_COLOURS)
        random.shuffle(colours)  # randomise the colour-to-spot assignment too
        self.part_pos.clear()
        self.available.clear()
        for colour, (x, y) in zip(colours, pts, strict=False):
            self.part_pos[colour] = (x, y)
            self.available.add(colour)
            self.add_part(f"part_{colour}", x, y, PART_Z)
        print("  new batch on the bin: "
              + ", ".join(f"{c}({x:.2f},{y:.2f})" for c, (x, y) in self.part_pos.items()))

    def _convey_away(self, pid, place, colour):
        """Run the placed part out along its own lane and off the far end."""
        x0, y0, z = place
        ax, ay = lane_axis(colour)
        ex, ey = lane_end(colour)
        travel = math.hypot(ex - x0, ey - y0)
        for k in range(1, 8):
            d = travel * k / 7.0
            self.add_part(pid, x0 + ax * d, y0 + ay * d, z)
            time.sleep(0.18)
        self._remove_world(pid)

    def sort(self, color):
        if color not in SORT_COLOURS:
            self.alarm, self.alarm_msg = True, f"unknown colour '{color}'"
            self.publish_telemetry()
            print(f"  unknown colour '{color}'")
            return
        if color not in self.available:
            # the cell rejects an order for a colour that is not on the board
            self.alarm, self.alarm_msg = True, f"no {color} object on the board"
            self.publish_telemetry()
            print(f"  rejected: no {color} object on the board")
            return
        if not self.safe:
            # safety interlock: refuse to start motion when the cell is not safe
            self.alarm, self.alarm_msg = True, f"safety interlock ({self.safety_reason})"
            self.publish_telemetry()
            print(f"  refused: safety interlock ({self.safety_state}: {self.safety_reason})")
            return
        self.alarm, self.alarm_msg = False, ""
        self.speed_factor = self.speed_scale  # apply SSM speed reduction
        self.state, self.current = "sorting", color
        self.publish_telemetry()
        x, y = self.part_pos[color]
        pid = f"part_{color}"
        drop_z = CONVEYOR_TOP + PART_SIZE / 2 + CLEARANCE
        dx, dy = lane_drop(color)          # each colour has its own outfeed lane
        place = (dx, dy, drop_z)
        print(f"  sorting {color} at ({x:.2f}, {y:.2f}) -> {color} lane "
              f"({dx:.2f}, {dy:.2f}) ...")
        t0 = time.time()
        ok = self.pick_place(pid, (x, y, PART_Z), place, self.transit_cfg,
                             hold_on_abort=lambda: not self.safe)
        if ok:
            self._convey_away(pid, place, color)  # its lane carries it out of the cell
            self.counts[color] += 1
            self.cycle_times.append(time.time() - t0)
            self.available.discard(color)
            self.part_pos.pop(color, None)
            if not self.available:      # board empty -> feed a fresh random batch
                self._random_batch()
            self.state, self.current = "idle", ""
        elif self.held_after_abort:
            # Stopped mid-transfer: the part is still gripped and stays gripped.
            # It is off the board, so it is no longer orderable, but it is not
            # sorted either: it only clears when the operator resets.
            self.available.discard(color)
            self.part_pos.pop(color, None)
            self.alarm = True
            self.alarm_msg = f"safety stop holding {color}; RESET / ON to release"
            self.state, self.current = "held", color
        else:
            self.alarm, self.alarm_msg = True, f"{color} pick-place failed"
            self.state, self.current = "idle", ""
        self.publish_telemetry()
        print(f"  {color}: {'DONE' if ok else 'HELD' if self.held_after_abort else 'FAILED'}")


def main():
    rclpy.init()
    node = ColorSorter()
    node.setup()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.release_pending:
                node.do_release()
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
