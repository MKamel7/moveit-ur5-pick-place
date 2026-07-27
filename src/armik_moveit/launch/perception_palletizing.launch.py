"""One-command Phase 3 bringup: perception + MoveIt for perception-driven picking.

Starts the Gazebo RGB-D perception (world + camera bridge + detector) and the
UR5e + 2F-85 MoveIt cell together, so the palletizer can pick what the camera
sees. Run the cell after this is up:

    ros2 launch armik_moveit perception_palletizing.launch.py   # RViz on by default
    PICK_SOURCE=perception ros2 run armik_moveit palletize
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("armik_moveit")
    launch_rviz = LaunchConfiguration("launch_rviz")

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "perception.launch.py")))

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "ur5e_gripper_moveit.launch.py")),
        launch_arguments={"launch_rviz": launch_rviz}.items())

    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch the MoveIt RViz GUI."),
        perception,
        # Give Gazebo a head start so its /clock and the camera settle before
        # the MoveIt stack comes up alongside it.
        TimerAction(period=4.0, actions=[moveit]),
    ])
