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

### Stage 1: UR5e bringup + fixed-pose pick-and-place - DONE, verified in sim
- Installed ur_simulation_gz (+ gz_ros2_control, ros_gz_image) for the official
  UR5e Gazebo integration. Bringup: `ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e`.
- Wrote moveit_py node `pick_place_node.py` + `moveit_env.py` + `launch/pick_place.launch.py`.
  Planning group `ur_manipulator`, base_link -> tool0, robot name ur5e.
- Verified: full sequence (ready, pre-grasp, grasp, lift, transfer, place, retreat)
  planned with OMPL RRTConnect and executed, controllers SUCCEEDED, result SUCCESS.
  Evidence: docs/media/stage1_success_log.txt (+ two Gazebo screenshots from bringup).

Key technical facts learned (IMPORTANT for resume):
- moveit_py needs planning_pipelines as a nested map {"pipeline_names": ["ompl"]},
  NOT the flat list to_dict() emits, or it errors "Failed to load any planning pipelines".
  The launch post-processes to_dict() to fix this.
- Grasp must target the object TOP surface (z_offset = half object height + clearance),
  else the flange is inside the collision box and every goal is in collision.
- Move the arm to SRDF named state "up" in an EMPTY scene BEFORE adding the table,
  or the start state collides with the table.
- Gazebo uses dartsim physics which cannot build UR mesh collisions -> no reliable
  friction grasping; we grasp via MoveIt attach (AttachedCollisionObject). This is
  standard for UR Gazebo demos.
- SCREEN CAPTURE: scrot/import (XWayland) capture BLACK for Gazebo (native Wayland).
  Do NOT rely on scrot for Gazebo/RViz evidence. Instead capture the in-sim RGB-D
  CAMERA topic to image files via ROS (planned for Stage 3), which is headless and
  gives real rendered frames. User can also use Gazebo's own screenshot button
  (saves to ~/.gz/gui/pictures).
- moveit_py can SIGSEGV during C++ teardown after the result prints; main() guards it.

## Next steps (resume here)
1. Stage 2: collision-aware planning. Add a tall obstacle between start and goal in
   the planning scene, show OMPL routes around it where a straight line would hit.
   Quantify plan time and path length (add a small eval script; log numbers).
2. Stage 3: add an RGB-D camera to the Gazebo world looking at the table, run
   segmentation.py on the RGB image, sample depth, use perception.pixel_to_base with
   the camera->base TF to get the pick pose (NOT hardcoded). Write a detector_node.
   Capture camera frames as evidence. Add tests for the detector wiring.
3. Stage 4: randomized 10-trial eval (>=8/10 success), one-command launch that starts
   sim + runs demo, GitHub Actions CI (colcon build + pytest), finish README with
   measured numbers + demo GIF (built from camera frames), one CV bullet.
   Then: gh auth (needs workflow scope, interactive), create PUBLIC repo
   moveit-ur5-pick-place under MKamel7, push, confirm CI green.
4. THEN Project 2 (llm-robot-commander) per the brief.

## Overnight auto-resume mechanism (set up 2026-07-05 ~02:41 CEST)
- Script: `scripts/auto_resume.sh`. Relaunches `claude -p ... --dangerously-skip-permissions`
  to continue the build. Guards: dormant until an armed epoch (`.auto_resume_arm_epoch`,
  set to ~07:41 CEST, 5h after setup), flock so only one runs, and a done-sentinel
  (`.auto_resume_done`) that stops the loop when both projects are verified done.
- Headless `claude -p` auth was tested and works (returned RESUME_OK, exit 0).
- Cron install was BLOCKED by the auto-mode safety classifier (it will not let an agent
  silently install an unattended skip-permissions loop). Mohamed must install the cron
  line himself, once, with:
      ( crontab -l 2>/dev/null | grep -v auto_resume.sh ; \
        echo "*/30 * * * * /home/kamel/moveit-ur5-pick-place/scripts/auto_resume.sh >> /home/kamel/moveit-ur5-pick-place/docs/auto_resume_logs/cron.log 2>&1" ) | crontab -
  Verify: `crontab -l`. Logs land in `docs/auto_resume_logs/`.
- The resume agent has NO sudo password: it does user-space work and records anything
  needing sudo under a "NEEDS SUDO" heading here. It also cannot push if gh lacks the
  workflow scope; in that case it writes exact push steps here.

## Review checkpoints (need user's eyes)
- Stage 1: RViz/Gazebo pick-and-place motion looks correct.
- Stage 2: obstacle-avoidance path is visibly sane.
- Stage 3: detection overlay + estimated pose look right.
- Final: demo GIF and README before publishing.
