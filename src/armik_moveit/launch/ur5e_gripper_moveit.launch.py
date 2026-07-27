"""UR5e + Robotiq 2F-85 MoveIt 2 bringup (mock hardware).

Unlike ur5e_moveit.launch.py (which wraps the stock bare-arm UR launches), this
loads the combined UR5e + 2F-85 description into BOTH ros2_control and MoveIt,
so the gripper is a real planning/execution end effector:

  - ros2_control_node with the combined description + ros2_controllers.yaml,
  - joint_state_broadcaster, joint_trajectory_controller (arm),
    robotiq_gripper_controller (2F-85),
  - move_group built from the combined URDF + SRDF via MoveItConfigsBuilder,
  - optional RViz MotionPlanning GUI.

    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py                # RViz
    ros2 launch armik_moveit ur5e_gripper_moveit.launch.py launch_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    pkg = get_package_share_directory("armik_moveit")

    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="armik_moveit")
        .robot_description(file_path="description/ur5e_robotiq.urdf.xacro")
        .robot_description_semantic(file_path="config/ur5e_robotiq.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    ros2_controllers = os.path.join(pkg, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(pkg, "config", "moveit.rviz")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
            output="screen",
        )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch the MoveIt RViz MotionPlanning GUI.",
            ),
            control_node,
            robot_state_publisher,
            spawner("joint_state_broadcaster"),
            spawner("joint_trajectory_controller"),
            spawner("robotiq_gripper_controller"),
            move_group_node,
            rviz_node,
        ]
    )
