"""Gazebo digital twin of the colour-sorting cell.

Brings up the sim-side mirror of a cell that is already running: the headless
Gazebo world, the camera bridges, the RGB-D detector, and the gz_twin node that
poses every robot link from TF. It does NOT bring up the arm or the sorter, so
run it alongside them:

    # 1. the cell (mock hardware, or real UR5e / URSim over RTDE)
    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py launch_rviz:=false
    # 2. the twin
    ros2 launch armik_moveit sort_cell_twin.launch.py
    # 3. the sorter
    ros2 run armik_moveit color_sort

The world is generated here rather than committed whole: worlds/sort_cell_twin.sdf
holds only the static scene (table, bin, conveyor, parts, cameras), and the
robot's link visuals are injected from description/ur5e_robotiq.urdf.xacro at
launch. That keeps one robot description for ros2_control, MoveIt and the twin.

The twin is one-way: it reads the cell and writes to Gazebo, never the reverse.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from armik_moveit.twin_world import build_world
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory("armik_moveit")
    world_name = LaunchConfiguration("world_name").perform(context)
    scene = os.path.join(pkg, "worlds", f"{world_name}.sdf")
    xacro_file = os.path.join(pkg, "description", "ur5e_robotiq.urdf.xacro")
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")

    # Expand the cell's own description. The hardware flags are irrelevant here
    # (only link visuals are used) but must be supplied for xacro to evaluate.
    import subprocess
    urdf = subprocess.run(
        ["xacro", xacro_file, "use_mock_hardware:=true"],
        check=True, capture_output=True, text=True).stdout

    out = os.path.join(tempfile.gettempdir(), f"{world_name}_generated.sdf")
    world, links = build_world(scene, urdf, out)
    print(f"[sort_cell_twin] generated {world} with {len(links)} link models")

    # The adapter-plate mesh is the one URI xacro cannot make absolute, so
    # Gazebo needs the ROS share tree on its resource path to resolve it.
    ros_share = os.path.join(os.environ.get("ROS_DISTRO_PREFIX", "/opt/ros/jazzy"), "share")
    resource_path = os.pathsep.join(
        p for p in (ros_share, os.environ.get("GZ_SIM_RESOURCE_PATH", "")) if p
    )

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
            "/iso_camera@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    detector = Node(package="armik_moveit", executable="detector", output="screen")

    # The supervisor is cell infrastructure, not an optional extra, and it was
    # started by hand or not at all: no launch file anywhere referenced it, and
    # nothing reported its absence. A safety layer an operator can forget to
    # start is not a safety layer (docs/safety_fmea.md, row 5.2). It also has to
    # be running for the cell to move now that color_sort fails safe on a
    # missing safety message rather than assuming full speed.
    safety = Node(
        package="armik_moveit", executable="safety_supervisor", output="screen"
    )

    twin = Node(
        package="armik_moveit",
        executable="gz_twin",
        output="screen",
        parameters=[{"world": world_name, "links": links}],
    )

    # Headless by default: the camera sensors are what the demo video needs, and
    # a GUI would only compete with them for the GPU. gui:=true adds the Gazebo
    # client for watching the cell by hand. This box is Wayland, and ogre2 is far
    # happier on XWayland, so the client is pinned to xcb.
    env = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path)]
    extra = []
    if gui:
        env.append(SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"))
        extra.append(TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=["gz", "sim", "-g", "-v", "1"], output="screen")]))
    else:
        env.append(SetEnvironmentVariable("QT_QPA_PLATFORM", "offscreen"))

    return env + [
        gz_server,
        TimerAction(period=6.0, actions=[bridge]),
        TimerAction(period=9.0, actions=[detector, twin]),
        # Started with the rest of the cell rather than waited on: it holds a
        # protective stop until its inputs report, so bringing it up early only
        # means the stop is in force from the beginning, which is the point.
        safety,
    ] + extra


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="sort_cell_twin",
                              description="Scene SDF basename under worlds/."),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Open the Gazebo GUI client as well as "
                                          "the server."),
        OpaqueFunction(function=launch_setup),
    ])
