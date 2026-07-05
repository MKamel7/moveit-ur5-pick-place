"""Launch the moveit_py obstacle-avoidance demo against a running UR5e sim.

    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
    ros2 launch ur5_pick_place obstacle_demo.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ur5_pick_place.launch_config import moveit_params


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur5_pick_place",
                executable="obstacle_demo",
                output="screen",
                parameters=moveit_params(),
            )
        ]
    )
