"""Start the cell's three action servers against a running simulation.

    ros2 launch ur5_pick_place demo_bringup.launch.py gazebo_gui:=false
    ros2 launch ur5_pick_place cell_actions.launch.py

Separate from the bringup on purpose. The bringup is the cell; this is one way
of driving it, and the placement benchmark is another. Starting them together
would make every measurement pay for a MoveIt instance it does not use.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ur5_pick_place.launch_config import moveit_params


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur5_pick_place",
            executable="cell_actions",
            name="cell_actions",
            output="screen",
            parameters=moveit_params(),
        )
    ])
