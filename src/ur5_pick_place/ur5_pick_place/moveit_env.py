"""Shared MoveIt/scene constants and small message helpers.

Kept separate from the node so the constants (planning group, frames) have a
single source of truth and the message builders can be exercised without a
running MoveItPy instance.
"""
from __future__ import annotations

from collections.abc import Sequence

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

PLANNING_GROUP = "ur_manipulator"
EEF_LINK = "tool0"
BASE_FRAME = "base_link"


def make_pose(position: Sequence[float], orientation: Sequence[float]) -> Pose:
    """Build a geometry_msgs/Pose from a 3-vector and an xyzw quaternion."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(c) for c in position)
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = (float(c) for c in orientation)
    return pose


def make_box(
    object_id: str,
    frame_id: str,
    size: Sequence[float],
    position: Sequence[float],
) -> CollisionObject:
    """Build an axis-aligned box CollisionObject at a position (identity orientation)."""
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(s) for s in size]

    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.primitives.append(box)
    obj.primitive_poses.append(make_pose(position, [0.0, 0.0, 0.0, 1.0]))
    obj.operation = CollisionObject.ADD
    return obj
