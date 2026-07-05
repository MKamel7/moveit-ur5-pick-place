# moveit-ur5-pick-place

Vision-guided, collision-aware pick-and-place for a simulated Universal Robots
UR5e using MoveIt 2 and ROS 2 Jazzy.

This is the sequel to my from-scratch kinematics work in
[robot-arm-ik](https://github.com/MKamel7/robot-arm-ik). There I wrote forward
kinematics, inverse kinematics, and trajectory generation by hand. Here I drive
the production motion-planning framework that industry actually deploys
(MoveIt 2 with OMPL), and I add a perception front-end so the grasp target
comes from a camera rather than a hardcoded pose.

> Status: work in progress. This README grows as each stage lands, and every
> number in the Results section is measured on my own runs, not aspirational.
> See `BUILD_LOG.md` for the running journal.

## What it does (target)

1. Bring up a UR5e in Gazebo with MoveIt 2 and plan a pick-and-place.
2. Plan around obstacles in a populated planning scene using OMPL.
3. Detect the target object with an RGB-D camera (classical HSV colour and
   shape segmentation, since this machine has no NVIDIA GPU), lift the 2D
   detection to a 3D pose using the depth image and the camera-to-base
   transform, and use that pose as the pick target.
4. Execute a collision-free pick-and-place, evaluated over randomized object
   placements.

## Architecture

The reusable, ROS-independent logic lives in `ur5_pick_place/` and is unit
tested without a running robot:

- `perception.py`: pinhole de-projection and frame transforms. Turns a pixel
  plus a depth reading into a 3D point in the base frame
  (`pixel_to_base`).
- `segmentation.py`: HSV colour + largest-blob segmentation and robust depth
  sampling. The classical, CPU-only detector.
- `grasp.py`: top-down grasp-pose generation with pre-grasp and retreat
  stand-offs along the approach axis.

The ROS nodes (added in later stages) wire these into MoveIt and Gazebo.

## Build

Requires ROS 2 Jazzy with MoveIt 2 and the Universal Robots packages.

    cd moveit-ur5-pick-place
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## Test

The core geometry and perception logic have automated unit tests:

    cd src/ur5_pick_place
    python3 -m pytest test/ -q

Current suite: 26 unit tests (perception-to-pose, grasp planning, colour
segmentation and depth sampling).

## Results

Measured results (success rate over randomized placements, planning time, path
length) will be reported here once the simulation stages are complete.

## License

MIT. See `LICENSE`.
