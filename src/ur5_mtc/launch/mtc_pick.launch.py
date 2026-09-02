"""Plan one pick as an MTC task graph against a running cell.

    ros2 launch ur5_pick_place demo_bringup.launch.py gazebo_gui:=false
    ros2 launch ur5_mtc mtc_pick.launch.py

Not started by the bringup. This is one way of driving the cell and the
placement benchmark is another; loading a second MoveIt into every run would
cost every measurement a planning stack it does not use.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ur5_pick_place.launch_config import moveit_params


def generate_launch_description():
    args = [
        DeclareLaunchArgument('target_x', default_value='0.55'),
        DeclareLaunchArgument('target_y', default_value='0.17'),
        DeclareLaunchArgument('target_z', default_value='0.25'),
        DeclareLaunchArgument('yaw_count', default_value='6'),
        DeclareLaunchArgument('execute', default_value='false'),
    ]
    return LaunchDescription(args + [
        Node(
            package='ur5_mtc', executable='mtc_pick', name='mtc_pick',
            output='screen',
            parameters=moveit_params() + [{
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_z': LaunchConfiguration('target_z'),
                'yaw_count': LaunchConfiguration('yaw_count'),
                'execute': LaunchConfiguration('execute'),
            }]),
    ])
