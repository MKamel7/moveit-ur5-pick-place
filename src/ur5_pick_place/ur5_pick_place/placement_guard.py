#!/usr/bin/env python3
"""Confirm a commanded pose actually took, instead of trusting the request.

WHY THIS IS ITS OWN MODULE

`_gz_set_pose` returning True means the Gazebo service accepted the request. It
does not mean the part is there, and on 2026-09-01 the difference cost two rows
out of thirteen in a grasp campaign. `part_animator` answers `reset` by writing
every part back to `PART_HOMES`, and a trial that commanded its placement while
that reset was still in flight had the part put back at part_green's home of
(0.55, 0.17) underneath it. Perception then found a green cube on the table, the
table-bounds guard passed because home IS on the table, and the CSV recorded a
133.8 mm and a 161.8 mm "perception error" against a part that was never at the
sampled pose. One of those two rows scored SUCCESS: the arm really did pick and
place a part, just not the one the row is about.

Sleeping longer after the reset would only have made the race rarer. The fix is
to stop inferring the world state and read it: Gazebo publishes every model's
pose on `/world/<world>/pose/info`.

It lives beside `detection_outcome.py` and for the same reason: the decision is
pure, so it can be tested on a plain runner with no ROS, no Gazebo and no arm,
against poses a test chooses.
"""
from __future__ import annotations

import math
import threading
import time

WORLD_NAME = "pick_place"

# Where ground_truth_node republishes the simulator's true part poses. It is
# named here so the publisher and the reader cannot drift apart.
GROUND_TRUTH_TOPIC = "/ground_truth/parts"

# Where part_animator confirms it has finished a /carry_cmd op.
ACK_TOPIC = "/carry_ack"

PLACEMENT_TOL_M = 0.005

# How long to wait for a commanded pose to appear in Gazebo's own pose feed.
# A set_pose is applied on the next simulation step, so anything beyond a
# fraction of a second means it is not going to happen.
PLACEMENT_TIMEOUT_S = 3.0


class GroundTruth:
    """The poses Gazebo actually holds, not the ones we asked it for.

    Reads the TF that `ground_truth_node` republishes from the simulator's own
    pose feed, and keeps the latest pose per model, so a caller can ask what the
    simulator believes rather than what it was told.

    WHY NOT SUBSCRIBE TO GZ TRANSPORT DIRECTLY, WHICH WOULD NEED NO BRIDGE

    Because it does not work here, and it fails silently. A `gz.transport13`
    subscription to that topic returns True from `subscribe()` and then delivers
    nothing inside the launched benchmark process, while the identical code
    receives poses from a plain shell, from a `setsid` script and from under
    `ros2 launch`, on the same box against the same running simulation. Service
    requests from that same process work, which is why the placements themselves
    were landing. Rather than ship a campaign whose ground truth depends on a
    mechanism that is dead in exactly one process, this takes the same data down
    the path the rest of the cell already uses.

    It spins its own node on a background thread. The benchmark's main thread
    spends minutes at a time blocked inside MoveIt's C++ execution, and a feed
    that only advances when the main thread is idle would be stale exactly when
    a placement is being confirmed.
    """

    def __init__(self, topic: str = GROUND_TRUTH_TOPIC) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from tf2_msgs.msg import TFMessage

        self._lock = threading.Lock()
        self._poses: dict[str, tuple[float, float, float]] = {}
        self._node = rclpy.create_node("benchmark_ground_truth")
        self._node.create_subscription(TFMessage, topic, self._on_tf, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_tf(self, msg) -> None:
        with self._lock:
            for tf in msg.transforms:
                v = tf.transform.translation
                self._poses[tf.child_frame_id] = (v.x, v.y, v.z)

    def get(self, name: str) -> tuple[float, float, float] | None:
        with self._lock:
            return self._poses.get(name)

    def wait_for_feed(self, timeout_s: float = 10.0) -> bool:
        """True once any pose has arrived.

        Called before a campaign starts. A dead feed would otherwise fail every
        trial identically at the placement check, which is a hundred rows of
        noise where one refusal to start is the useful answer.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                if self._poses:
                    return True
            time.sleep(0.05)
        return False


def at_pose(pose: tuple[float, float, float] | None, x: float, y: float,
            tol_m: float = PLACEMENT_TOL_M) -> bool:
    """True if a ground-truth pose is within tol_m of a commanded x, y.

    z is deliberately ignored. A cube dropped 1 mm above the table settles, and
    the question this answers is whether the part is where the trial says it
    is, which is a question about the table plane.
    """
    if pose is None:
        return False
    return math.hypot(pose[0] - x, pose[1] - y) <= tol_m


def wait_until_at(truth, name: str, x: float, y: float,
                  timeout_s: float = PLACEMENT_TIMEOUT_S,
                  tol_m: float = PLACEMENT_TOL_M) -> bool:
    """Block until Gazebo reports `name` at x, y, or the timeout expires."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if at_pose(truth.get(name), x, y, tol_m):
            return True
        time.sleep(0.05)
    return False



class AnimatorAck:
    """Waits for part_animator to confirm an operation, rather than guessing.

    `reset` makes the animator write all three parts back to their homes. Those
    writes are asynchronous, and a benchmark that commands a placement while
    they are in flight measures a part the animator has since moved. Watching
    the poses is not enough on its own: when the parts are already home the
    check passes without the animator having done anything at all.
    """

    def __init__(self, topic: str = ACK_TOPIC) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String

        self._seen = threading.Event()
        self._node = rclpy.create_node("benchmark_animator_ack")
        self._node.create_subscription(String, topic, self._on_ack, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def _on_ack(self, msg) -> None:
        self._seen.set()

    def arm(self) -> None:
        """Forget any earlier acknowledgement, so only a fresh one counts."""
        self._seen.clear()

    def wait(self, timeout_s: float = 5.0) -> bool:
        return self._seen.wait(timeout_s)
