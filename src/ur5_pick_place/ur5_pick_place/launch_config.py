"""Shared MoveIt configuration for the moveit_py launch files.

Building the UR5e MoveIt config (with the moveit_py-specific parameter fixes) in
one place keeps the pick-place and obstacle-demo launches identical and avoids
drift.
"""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# moveit_py needs an explicit default plan-request profile.
PLAN_REQUEST_PARAMS = {
    "plan_request_params": {
        "planning_attempts": 10,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.3,
        "max_acceleration_scaling_factor": 0.3,
        "planning_time": 5.0,
    }
}


def moveit_params() -> list:
    """Return the parameter list to pass to a moveit_py Node."""
    params = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur_description"), "urdf", "ur.urdf.xacro"
            ),
            mappings={"name": "ur5e", "ur_type": "ur5e"},
        )
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur5e"})
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
        .to_dict()
    )
    # move_group wants planning_pipelines as a flat list; MoveItCpp (moveit_py)
    # wants a nested map with pipeline_names, or it fails to load pipelines.
    params["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    return [params, PLAN_REQUEST_PARAMS, {"use_sim_time": True}]
