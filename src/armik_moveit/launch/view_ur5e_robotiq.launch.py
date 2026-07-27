"""Visualize the combined UR5e + Robotiq 2F-85 description in RViz.

Description-only view (no MoveIt, no controllers): robot_state_publisher loads
the combined xacro, joint_state_publisher_gui gives sliders to pose the arm and
open/close the gripper, and RViz shows the model. Handy for eyeballing that the
2F-85 is mounted correctly before the full MoveIt config is built.

    ros2 launch armik_moveit view_ur5e_robotiq.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("armik_moveit")
    xacro_file = os.path.join(pkg, "description", "ur5e_robotiq.urdf.xacro")
    rviz_config = os.path.join(pkg, "config", "view.rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]), value_type=str
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                # Force XWayland: the GUI's Qt build has no Wayland platform
                # plugin, so on a Wayland session it exits without xcb.
                additional_env={"QT_QPA_PLATFORM": "xcb"},
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
