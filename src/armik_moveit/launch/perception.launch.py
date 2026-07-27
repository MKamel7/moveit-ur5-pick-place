"""Phase 3 perception bringup: Gazebo RGB-D camera over the bin + detector.

Runs the perception_cell world (headless Gazebo server), bridges the RGB-D
camera topics from Gazebo to ROS, and starts the detector that segments the
coloured parts and publishes their 3D poses on /detected_parts.

    ros2 launch armik_moveit perception.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("armik_moveit")
    world = os.path.join(pkg, "worlds", "perception_cell.sdf")
    gui = LaunchConfiguration("gui")

    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "1", world],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    detector = Node(
        package="armik_moveit",
        executable="detector",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false",
                                  description="Open the Gazebo GUI."),
            gz_server,
            TimerAction(period=3.0, actions=[bridge]),
            TimerAction(period=5.0, actions=[detector]),
        ]
    )
