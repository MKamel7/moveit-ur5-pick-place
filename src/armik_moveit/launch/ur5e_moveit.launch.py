"""One-command UR5e + MoveIt 2 bringup for Phase 2 (mock hardware).

Wraps the two stock Universal Robots launches into a single entry point and
bakes in the Jazzy-specific fixes this repo's bringup needs:

  1. The mock-hardware flag on Jazzy is ``use_mock_hardware`` (it was renamed
     from ``use_fake_hardware``); with it, no real robot IP is contacted.
  2. ``ur_control`` can leave ``scaled_joint_trajectory_controller`` *inactive*
     after its consistent-controller-set spawn, so MoveIt would have nothing to
     execute on. A short-delayed step activates it (only if it is not already
     active) once bringup settles.
  3. ``launch_rviz`` is resolved to a literal string before being forwarded to
     the MoveIt include. Passing the same-named ``LaunchConfiguration`` into a
     scoped include self-references and resolves empty, so RViz's IfCondition
     would silently stay false and no RViz window would open.

Usage:
    ros2 launch armik_moveit ur5e_moveit.launch.py                # with RViz
    ros2 launch armik_moveit ur5e_moveit.launch.py launch_rviz:=false

Then plan+execute (headless smoke test):
    ros2 run armik_moveit plan_execute_smoke
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Resolve launch_rviz to a concrete "true"/"false" string so the forward
    # into the (scoped) MoveIt include is unambiguous. See fix (3) above.
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    ur_driver = get_package_share_directory("ur_robot_driver")
    ur_moveit = get_package_share_directory("ur_moveit_config")

    # 1. ros2_control + mock hardware (no real robot contacted).
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_driver, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": "0.0.0.0",  # ignored under mock hardware
            "use_mock_hardware": "true",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "launch_rviz": "false",  # MoveIt brings up its own RViz
        }.items(),
    )

    # 2. MoveGroup + RViz (reads /robot_description from the control stack).
    #    move_group ships its own wait_for_robot_description gate, so it can be
    #    started directly; wrapping it in a TimerAction breaks ur_moveit's
    #    declare_arguments() scoping (warehouse_sqlite_path stops resolving).
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit, "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "use_sim_time": "false",
            "launch_rviz": launch_rviz,
            # Passed explicitly: when ur_moveit.launch.py is *included* rather
            # than run directly, its own default for this arg is not propagated
            # and MoveGroup's warehouse setup raises "launch configuration
            # 'warehouse_sqlite_path' does not exist".
            "warehouse_sqlite_path": os.path.join(
                os.path.expanduser("~"), ".ros", "warehouse_ros.sqlite"
            ),
        }.items(),
    )

    # 3. Ensure the trajectory controller MoveIt executes on is active.
    #    ur_control's consistent-set spawn *intermittently* leaves
    #    scaled_joint_trajectory_controller inactive. Activate it only if it is
    #    not already active, so this is a quiet no-op when the bringup was fine.
    jtc = "scaled_joint_trajectory_controller"
    activate_jtc = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    f'ros2 control list_controllers | grep -qE "{jtc}.*active" '
                    f"|| ros2 control set_controller_state {jtc} active",
                ],
                output="screen",
            )
        ],
    )

    return [control, moveit, activate_jtc]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch the MoveIt RViz interactive-marker GUI.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
