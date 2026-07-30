"""Mirror the running cell into the Gazebo twin world.

This is a VIEW, not a simulation. The arm that moves is the UR5e on the real
controller (URSim over RTDE, or mock hardware); robot_state_publisher already
turns its /joint_states into an exact transform for every link, which is the same
data RViz draws. So the twin does what RViz does: it reads those transforms and
places the matching Gazebo model. It never simulates, never commands, and cannot
affect the robot.

An earlier version of this node did simulate: the twin was an articulated model
whose 12 joints were driven by velocity PID through the physics solver, chasing
the real robot. That cannot be made smooth. Tracking lag trades against wobble,
inertia and contact fight the target, gravity topples an unanchored base, and a
grasped part visibly lags the gripper. Reading a transform has none of those
failure modes and no gains to tune.

    /joint_states -> robot_state_publisher -> TF -+-> RViz
                                                  +-> this node -> Gazebo poses

Parts are mirrored from the MoveIt planning scene. A part being carried is posed
against the link it is attached to, so it comes from the same TF chain as the
gripper and rides it rigidly.

    ros2 run armik_moveit gz_twin --ros-args -p world:=sort_cell_twin
"""
import threading

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
import tf2_ros

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.boolean_pb2 import Boolean

from armik_moveit.twin_world import model_name

# Parts mirrored into Gazebo, and where to park one that is not in the scene
# (conveyed off the belt): out of every camera's view, under the table.
PART_MODELS = ["part_red", "part_green", "part_blue"]
PARKED = ((0.0, 0.0, -1.0), (0.0, 0.0, 0.0, 1.0))

# The planning frame. The control URDF roots at a link called "world" on the
# floor under the pedestal, which is also the Gazebo world origin, so poses drop
# straight in with no offset.
PLANNING_FRAME = "world"

MIRROR_HZ = 30.0    # link and part poses
# Safety-net rate for re-reading the planning scene. Attach and detach do not
# wait for it: /monitored_planning_scene fires on every scene change and triggers
# an immediate re-read. Polling alone is not enough, because the grasp moment is
# exactly when being half a poll behind is most visible: the gripper lifts by the
# approach clearance while the part is still drawn on the bin, so the part
# appears to jump ~150 mm into the gripper a moment later.
SCENE_HZ = 1.0

# A pose has to move by more than this to be worth resending.
POS_EPS = 2e-4
ROT_EPS = 5e-4


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rot(q, v):
    """Rotate v by quaternion q (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def compose(pa, pb):
    """Compose transform pa with pb, each ((x,y,z), (qx,qy,qz,qw))."""
    ta, qa = pa
    tb, qb = pb
    rt = quat_rot(qa, tb)
    return ((ta[0] + rt[0], ta[1] + rt[1], ta[2] + rt[2]), quat_mul(qa, qb))


def from_msg(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def moved(a, b):
    if a is None or b is None:
        return True
    return (any(abs(x - y) > POS_EPS for x, y in zip(a[0], b[0]))
            or any(abs(x - y) > ROT_EPS for x, y in zip(a[1], b[1])))


class GzTwin(Node):
    def __init__(self):
        super().__init__("gz_twin")
        self.declare_parameter("world", "sort_cell_twin")
        self.declare_parameter("links", [""])
        self.declare_parameter("mirror_hz", MIRROR_HZ)
        self.declare_parameter("scene_hz", SCENE_HZ)
        world = self.get_parameter("world").value
        self.links = [l for l in self.get_parameter("links").value if l]
        mirror_hz = float(self.get_parameter("mirror_hz").value)
        scene_hz = float(self.get_parameter("scene_hz").value)
        self.pose_srv = f"/world/{world}/set_pose_vector"

        if not self.links:
            self.get_logger().error(
                "no 'links' parameter: nothing to mirror. The launch file passes "
                "the link list generated from the robot description.")

        self.gz = GzNode()

        # Part structure, refreshed slowly from the planning scene.
        self.free_poses = {}    # pid -> world pose of a part sitting in the scene
        self.held = {}          # pid -> (attach link, pose of part in that link)

        # Poses the worker thread should push, and what it last pushed.
        self.target = {}
        self.sent = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.missing_tf = set()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.scene_req = GetPlanningScene.Request()
        self.scene_req.components = PlanningSceneComponents(
            components=PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        self.scene_future = None
        self.scene_dirty = True
        self.warned_scene = False
        # move_group publishes here whenever the scene changes, including the
        # attach and detach that bracket a carry. Any message just marks the
        # cached structure stale; the authoritative read is still the service,
        # so there is no diff-merging to get wrong.
        self.create_subscription(PlanningScene, "/monitored_planning_scene",
                                 self._on_scene_changed, 10)

        self.create_timer(1.0 / mirror_hz, self.mirror)
        self.create_timer(1.0 / scene_hz, self.poll_scene)
        # A short timer is what actually reacts to the notification above.
        self.create_timer(0.02, self._service_refresh)

        self.worker = threading.Thread(target=self._pose_worker,
                                       args=(mirror_hz,), daemon=True)
        self.worker.start()

        self.get_logger().info(
            f"gz_twin mirroring {len(self.links)} links + {len(PART_MODELS)} parts "
            f"into gz world '{world}' at {mirror_hz:g} Hz")

    # --- the mirror --------------------------------------------------------
    def mirror(self):
        poses = {}

        for link in self.links:
            t = self._lookup(link)
            if t is None:
                if link not in self.missing_tf:
                    self.missing_tf.add(link)
                    self.get_logger().warn(f"no transform for '{link}' yet",
                                           throttle_duration_sec=10.0)
                continue
            self.missing_tf.discard(link)
            # The URDF visual origin is baked into the model's <visual><pose>,
            # so the model pose is the link pose with nothing to compose.
            poses[model_name(link)] = t

        for pid in PART_MODELS:
            grasp = self.held.get(pid)
            if grasp is not None:
                link, offset = grasp
                t_link = self._lookup(link)
                if t_link is not None:
                    poses[pid] = compose(t_link, offset)
                    continue
            poses[pid] = self.free_poses.get(pid, PARKED)

        with self.lock:
            self.target = poses

    # --- part structure ----------------------------------------------------
    def _on_scene_changed(self, _msg):
        self.scene_dirty = True

    def _service_refresh(self):
        """Collect a finished request, and start one if the scene went stale."""
        if self.scene_future is not None:
            if not self.scene_future.done():
                return
            result = self.scene_future.result()
            self.scene_future = None
            if result is not None:
                self._read_scene(result.scene)
        if not self.scene_dirty:
            return
        if not self.scene_cli.service_is_ready():
            if not self.warned_scene:
                self.get_logger().warn("waiting for /get_planning_scene (move_group)")
                self.warned_scene = True
            return
        self.warned_scene = False
        self.scene_dirty = False
        self.scene_future = self.scene_cli.call_async(self.scene_req)

    def poll_scene(self):
        """Safety net: re-read even if no change notification arrived."""
        self.scene_dirty = True

    def _read_scene(self, scene):
        free, held = {}, {}
        for obj in scene.world.collision_objects:
            if obj.id in PART_MODELS:
                free[obj.id] = self._object_pose(obj)
        for aco in scene.robot_state.attached_collision_objects:
            obj = aco.object
            if obj.id in PART_MODELS:
                link = aco.link_name or obj.header.frame_id
                held[obj.id] = (link, self._object_pose(obj))
        self.free_poses, self.held = free, held

    def _object_pose(self, obj):
        """Pose of a box collision object in its own header frame: the object
        pose composed with the primitive's offset within it."""
        p = from_msg(obj.pose)
        if obj.primitive_poses:
            p = compose(p, from_msg(obj.primitive_poses[0]))
        return p

    def _lookup(self, frame):
        try:
            tf = self.tf_buffer.lookup_transform(
                PLANNING_FRAME, frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    # --- Gazebo pose pushes ------------------------------------------------
    def _pose_worker(self, hz):
        """Push changed poses to Gazebo off the ROS executor.

        set_pose is a blocking service call. Batching every model into one
        set_pose_vector and running it here keeps that latency away from the
        timer callbacks.
        """
        period = 1.0 / hz
        while not self.stop.wait(period):
            with self.lock:
                target = dict(self.target)
            changed = {m: v for m, v in target.items() if moved(v, self.sent.get(m))}
            if not changed:
                continue
            req = Pose_V()
            for name, ((x, y, z), (qx, qy, qz, qw)) in changed.items():
                m = req.pose.add()
                m.name = name
                m.position.x, m.position.y, m.position.z = x, y, z
                m.orientation.x, m.orientation.y = qx, qy
                m.orientation.z, m.orientation.w = qz, qw
            ok, rep = self.gz.request(self.pose_srv, req, Pose_V, Boolean, 150)
            if ok and rep.data:
                self.sent.update(changed)
            else:
                self.get_logger().warn("set_pose_vector failed",
                                       throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = GzTwin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
