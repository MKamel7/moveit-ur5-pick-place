# Recording the cell demo video

The demo video shows the colour sorting cell from four angles at once: what the
camera sees, the Gazebo digital twin, the real UR teach pendant, and the live
telemetry. This file is the procedure, because the capture is fiddly enough that
it is not worth rediscovering.

## Why it is not just a screen recording

This box runs GNOME on Wayland. `wmctrl` and `xdotool` cannot enumerate windows
and GNOME blocks programmatic screenshots, so nothing can record the desktop.
Every stream is therefore taken from its source instead of from the screen:

| Panel | Source |
| --- | --- |
| Perception | `/rgbd_camera/image` and `/detected_parts_named` over ROS |
| Gazebo twin | `/iso_camera`, a camera sensor inside the twin world |
| UR controller | the URSim container's own VNC framebuffer (`172.17.0.2:5900`) |
| Cell telemetry | `/cell/telemetry` and `/safety/state` over ROS |

Capture and render are separate stages. Composing a four panel frame takes longer
than a frame interval, so the capture stage only writes timestamped assets to
disk and the render stage builds the video afterwards. Every asset is named after
its capture time in milliseconds, which is the only index the renderer needs.

## How the twin works

The Gazebo panel is a one way mirror of a cell running elsewhere, not a second
simulated robot. It draws where the real robot's links ARE; it does not work out
where they should be.

- `worlds/sort_cell_twin.sdf` holds only the static scene: the cell geometry
  (matching `scene.py`), the calibrated top down RGB-D camera, and the
  `iso_camera` overview.
- `twin_world.py` generates the robot at launch. It reads
  `description/ur5e_robotiq.urdf.xacro`, the same description that drives
  ros2_control and MoveIt, and emits one `<static>` model per visual link into a
  copy of the scene. There is no second robot description to keep in sync.
- `gz_twin.py` poses those models from TF, the transforms
  `robot_state_publisher` computes from the real robot's `/joint_states`. That is
  the same data RViz draws, which is why the two look identical.

```
/joint_states -> robot_state_publisher -> TF -+-> RViz
                                              +-> gz_twin -> Gazebo model poses
```

Measured on real hardware (URSim over RTDE): stationary agreement with TF of
**0.019 mm / 0.68 mrad** over 450 comparisons, and about **13 ms** of transport
delay while moving. `tools/test_twin_mirror.py` and `tools/test_twin_grasp.py`
check both, plus that a carried part stays within a fraction of a millimetre of a
fixed offset in the gripper frame.

Three things here are load bearing:

1. **Do not simulate the arm.** The first version of this twin was an articulated
   model whose joints were driven by velocity PID through the physics solver,
   chasing the robot. That cannot be made smooth: tracking lag trades against
   wobble, gravity topples an unanchored base (which needed a fake floor plate to
   hide), and a grasped part lags the gripper. Reading a transform has none of
   those failure modes and nothing to tune.
2. **Attach and detach must not wait for a poll.** The parts come from the MoveIt
   planning scene, but the grasp moment is exactly where being half a poll behind
   shows: the gripper lifts by the approach clearance while the part is still
   drawn on the bin, so it appears to jump ~150 mm afterwards. `gz_twin.py`
   subscribes to `/monitored_planning_scene` and re-reads immediately on any
   change; polling is only a safety net. Measured effect: 145 mm of wander down
   to 0.6 mm.
3. **The twin world's lighting must match `perception_cell.sdf`.** `detector.py`
   segments parts with fixed RGB bounds, so an added ambient term or fill light
   washes a saturated primary towards grey and that colour stops being detected.
   A brighter scene buys a nicer Gazebo panel and a broken detector.

## Procedure

Bring up the cell, the twin, then the sorter. With URSim (hardware in the loop):

```bash
# The container is not left running between sessions. Create it WITHOUT --rm,
# or it deletes itself on stop and the next session finds nothing to start.
docker run -d --name ursim -e ROBOT_MODEL=UR5 \
    -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \
    universalrobots/ursim_e-series
# then, on the pendant (http://localhost:6080/vnc.html): confirm the safety
# configuration dialog, and power on + release brakes (dashboard port 29999).

ros2 launch armik_moveit ur5e_gripper_moveit.launch.py launch_rviz:=false \
     use_mock_hardware:=false robot_ip:=172.17.0.2 reverse_ip:=172.17.0.1 \
     headless_mode:=true use_mock_gripper:=true
ros2 launch armik_moveit sort_cell_twin.launch.py
ros2 run    armik_moveit safety_supervisor
ros2 run    armik_moveit color_sort
```

Swap the first launch for `use_mock_hardware:=true` to run without URSim.

**Order matters with real hardware.** Settle the pendant BEFORE starting the
driver. Confirming the safety configuration dialog afterwards kills the URScript
the driver injected in headless mode, and the failure is silent: ros2_control
keeps reporting every trajectory as successful while the robot does not move, and
MoveIt then plans each LIN descent from a stale pose and fails with
`NO_IK_SOLUTION`. If motion stops reaching the robot, restart the driver.

Capture while driving the cell. The pendant capture needs `vncdotool`, which is
not in the ROS Python environment, so it runs from a venv:

```bash
python3 tools/cell_demo_capture.py --out /tmp/session --seconds 105 &
<venv>/bin/python tools/cell_demo_vnc.py --out /tmp/session \
    --host 172.17.0.2::5900 --seconds 105 &

# then drive it: sort colours, and trigger the safety layer partway through
ros2 topic pub --once /target_color std_msgs/String "{data: red}"
ros2 topic pub --once /safety/human_present std_msgs/Bool "{data: true}"
```

Render:

```bash
python3 tools/cell_demo_render.py --session /tmp/session \
        --out cell-demo.mp4 --hardware ursim
```

`--hardware` only sets labels, so pass `mock` for a mock hardware run rather than
letting the video claim RTDE. The renderer drops the pendant column and narrows
the canvas when a session has no pendant frames, so a run without URSim comes out
as a three panel 1960x820 video instead of four panel 2720x820.

Useful flags while iterating on the layout: `--start` and `--duration` to render a
few seconds out of the middle, and `--keep-frames` to inspect a still.
