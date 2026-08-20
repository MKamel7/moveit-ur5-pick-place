"""Launch the randomised-placement benchmark against a running UR5e sim.

Bring the simulation and perception up first:
    ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
    ros2 launch ur5_pick_place perception.launch.py

then measure:
    ros2 launch ur5_pick_place benchmark_placements.launch.py

Environment variables (BENCH_TRIALS, BENCH_SEED, BENCH_COLOR, BENCH_CSV) are
documented in ur5_pick_place/benchmark_placements.py. They are read by the node
itself, so they must be exported before this launch, not passed as launch args:
    BENCH_TRIALS=20 BENCH_SEED=0 ros2 launch ur5_pick_place benchmark_placements.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ur5_pick_place.launch_config import moveit_params


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ur5_pick_place",
                executable="benchmark_placements",
                output="screen",
                parameters=moveit_params(),
            )
        ]
    )
