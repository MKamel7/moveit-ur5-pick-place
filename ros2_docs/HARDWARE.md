# Running on real hardware

The cell is built on `ros2_control`, so switching from simulation to a physical
UR5e + Robotiq 2F-85 is a launch flag, not a code change. The motion, perception,
safety, and OPC UA layers are identical.

```bash
# Simulation (default): mock hardware
ros2 launch armik_moveit ur5e_gripper_moveit.launch.py

# Real hardware
ros2 launch armik_moveit ur5e_gripper_moveit.launch.py \
    use_mock_hardware:=false robot_ip:=192.168.1.10
```

`use_mock_hardware:=false` selects the real hardware interfaces in the combined
description (`description/ur5e_robotiq.urdf.xacro`):

- **Arm** → `ur_robot_driver/URPositionHardwareInterface` (RTDE to the UR5e at
  `robot_ip`).
- **Gripper** → `robotiq_driver/RobotiqGripperHardwareInterface` (serial, set
  `gripper_com_port`, default `/dev/ttyUSB0`).

## Prerequisites for the physical cell

1. **Network** — the UR5e reachable at `robot_ip`; the controller PC on the same
   subnet.
2. **UR side** — install the *External Control* URCap and point it at the control
   PC; put the robot in *Remote Control*.
3. **Kinematics calibration** — extract the robot's factory calibration with
   `ur_calibration` and pass it as `kinematics_parameters_file`. Without it the
   tool pose drifts by a few mm; with it, sub-mm.
4. **Gripper** — install `ros-jazzy-robotiq-driver`; connect the 2F-85 over
   USB/RS-485; grant serial permissions (`dialout` group).
5. **Hand-eye calibration** — calibrate the camera-to-base transform so detected
   grasp poses map accurately to the robot frame.

## Controllers

The same `config/ros2_controllers.yaml` drives both: `joint_trajectory_controller`
for the arm and a `GripperActionController` for the gripper. On real hardware you
may prefer the UR `scaled_joint_trajectory_controller` (honours the teach-pendant
speed slider); it is a one-line swap in the controller config.

## Status

The hardware-ready description and launch are provided and the mock path is fully
verified. The real-hardware path is validated only in simulation here (no physical
UR5e was available); the interfaces, parameters, and bringup follow the standard
Universal Robots + Robotiq ROS 2 driver setup.
