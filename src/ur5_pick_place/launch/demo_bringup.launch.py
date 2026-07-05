"""One-command bringup for the vision-guided pick-and-place.

Spawns the UR5e into the custom world (table, green object, downward RGB-D
camera), starts MoveIt + RViz, bridges the camera topics from gz to ROS, and
runs the object detector. Run the pick-and-place separately once this is up:

    ros2 launch ur5_pick_place demo_bringup.launch.py
    ros2 launch ur5_pick_place pick_place.launch.py   # (perception-driven)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_color = LaunchConfiguration("target_color")
    pkg = get_package_share_directory("ur5_pick_place")
    world_file = os.path.join(pkg, "worlds", "pick_place.sdf")
    ur_sim = get_package_share_directory("ur_simulation_gz")

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_sim, "launch", "ur_sim_control.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "safety_limits": "true",
            "launch_rviz": "false",
            "world_file": world_file,
        }.items(),
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_sim_time": "true",
            # RViz + Gazebo together overwhelm the integrated GPU and RViz hangs.
            # The demo does not need RViz (planning runs in move_group), so keep it off.
            "launch_rviz": "false",
        }.items(),
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        arguments=[
            "/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/scene_camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        parameters=[{"use_sim_time": True}],
    )

    detector = Node(
        package="ur5_pick_place",
        executable="detector_node",
        name="object_detector",
        output="screen",
        parameters=[{"use_sim_time": True, "target_color": target_color}],
    )

    animator = Node(
        package="ur5_pick_place",
        executable="part_animator",
        name="part_animator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_color",
                default_value="green",
                choices=["red", "green", "blue"],
                description="Which coloured part the arm should pick and place on the belt.",
            ),
            ur_control,
            ur_moveit,
            camera_bridge,
            detector,
            animator,
        ]
    )
