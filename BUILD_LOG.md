# Build log and resume notes: moveit-ur5-pick-place

This file is a running journal so work can resume cleanly across sessions
(including after any usage limit reset). Newest entries at the bottom of each
section. It doubles as the RESUME file requested in the brief.

## Environment (decided once, 2026-07-05)
- Machine: Ubuntu 24.04.4, 12 cores, 15 GB RAM.
- ROS 2 Jazzy installed at /opt/ros/jazzy (source it before ROS work).
- NO NVIDIA GPU (Intel Iris Xe only). Using CPU paths: classical HSV colour +
  shape segmentation for perception (no YOLO/CUDA).
- Installed via apt: ros-jazzy-moveit, moveit-py, ur-moveit-config,
  ur-description, ur-robot-driver, ur-client-library, ros-gz, nav2-bringup,
  nav2-minimal-tb3-sim, turtlebot3-gazebo, gh, colcon, rosdep, pytest, numpy.
- Python libs present: numpy 1.26, cv2 4.6, scipy 1.11, pytest 7.4, yaml 6.0.
- git identity set globally to Mohamed Kamel / mkamel860@gmail.com.
- gh CLI installed but NOT yet authenticated. Needs interactive login WITH
  `workflow` scope before the final push (this scope blocked CI on robot-arm-ik).
- Project 2 LLM decision: qwen3:8b via Ollama (chosen by user; slow on CPU).

## How to run tests
    cd /home/kamel/moveit-ur5-pick-place/src/ur5_pick_place
    python3 -m pytest test/ -q
(These pure-Python tests need no ROS runtime.)

## Progress

### Stage 0: package skeleton + perception core (TDD) - DONE (code), tests green
- Created ament_python package `ur5_pick_place` under src/.
- perception.py: CameraIntrinsics, deproject_pixel_to_camera, transform_point,
  transform_matrix_from_quaternion, pixel_to_base. 11 tests.
- grasp.py: GraspPose, top_down_grasp, pregrasp_pose, retreat_pose. 7 tests.
- segmentation.py: Detection, segment_largest_blob, sample_depth (HSV colour
  segmentation, CPU path). 8 tests.
- Total: 26 unit tests passing. Written test-first (red then green).

## Next steps (resume here)
1. Finish ament package metadata (package.xml, setup.py, setup.cfg, ruff cfg),
   confirm `colcon build` + `colcon test` succeed.
2. git init + first commits (author = Mohamed Kamel only, NO AI attribution).
3. Stage 1: UR5e bringup in Gazebo+RViz via ur_moveit_config; moveit_py
   pick-and-place at a fixed pose. NEEDS A DISPLAY + user visual review.
4. Stage 2: collision-aware planning around an obstacle (quantify time/length).
5. Stage 3: RGB-D camera + segmentation -> 3D pose -> pick target.
6. Stage 4: 10-trial randomized eval (>=8/10), one-command demo, CI, README,
   evidence GIF, CV bullet. Create public repo, push, confirm CI green.

## Review checkpoints (need user's eyes)
- Stage 1: RViz/Gazebo pick-and-place motion looks correct.
- Stage 2: obstacle-avoidance path is visibly sane.
- Stage 3: detection overlay + estimated pose look right.
- Final: demo GIF and README before publishing.
