"""Launch the moveit_py pick-and-place node against a running UR5e sim.

Bring up the simulation first (Gazebo + MoveIt + RViz):
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e

then run this to execute the pick-and-place:
    ros2 launch ur5_pick_place pick_place.launch.py
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    return (
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
    )


# moveit_py needs an explicit default plan-request profile.
PLAN_REQUEST_PARAMS = {
    "plan_request_params": {
        "planning_attempts": 10,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "planning_time": 5.0,
    }
}


def generate_launch_description():
    params = build_moveit_config().to_dict()
    # move_group wants planning_pipelines as a flat list, but MoveItCpp (moveit_py)
    # wants it as a nested map with pipeline_names. to_dict() emits the list form,
    # so rewrite it here or MoveItCpp reports "Failed to load any planning pipelines".
    params["planning_pipelines"] = {"pipeline_names": ["ompl"]}

    pick_place_node = Node(
        package="ur5_pick_place",
        executable="pick_place_node",
        output="screen",
        parameters=[
            params,
            PLAN_REQUEST_PARAMS,
            {"use_sim_time": True},
        ],
    )
    return LaunchDescription([pick_place_node])
