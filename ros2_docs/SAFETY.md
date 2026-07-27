# Functional safety

The cell has a safety layer that is independent of the motion software and can
veto it, the structure a factory cell needs. It is implemented by the
`safety_supervisor` node and enforced by the sorter, the OPC UA server, and the
dashboard.

> Scope: this is an application-level safety supervisor for a simulated cell. A
> real deployment additionally requires **safety-rated hardware** (a safety PLC /
> safety I/O, category-rated E-stop circuits, safety-rated monitored stop). The
> design is *inspired by* ISO 10218 and ISO/TS 15066; it is not a certified,
> SIL/PL-rated safety function.

## Safety states

| State | Meaning | Motion |
| --- | --- | --- |
| `RUN` | all clear | full speed |
| `REDUCED` | human in the collaborative zone (SSM) | reduced speed |
| `GUARD_STOP` | guard/gate open (protective stop) | inhibited (auto-recovers on close) |
| `ESTOP` | emergency stop, latched | stopped; needs release + reset |
| `FAULT` | robot feedback lost (watchdog) | stopped; needs reset |

## Behaviours

- **Emergency stop (latched)** — trips to a safe state and cancels any motion in
  progress (`cancel-all` on `/move_action`). Clears only when the E-stop is
  released *and* a reset is issued.
- **Guard interlock** — opening the guard forces a protective stop; motion is
  inhibited until the guard closes (auto-recovers, no reset needed).
- **Speed and separation monitoring (ISO/TS 15066)** — a human in the zone drops
  the commanded speed factor (default 0.3); the cell keeps running slower.
- **Watchdog** — stale joint feedback (no robot state) faults the cell to a safe
  state.
- **Interlock** — the sorter refuses to start a motion unless `clear_to_run`.

## Interfaces

Safety inputs (ROS topics; also writable over OPC UA from a safety PLC):

| Topic | OPC UA (`Safety/…`) | Meaning |
| --- | --- | --- |
| `/safety/estop` (Bool) | `EStop` | emergency stop asserted |
| `/safety/guard_closed` (Bool) | `GuardClosed` | guard closed (True = safe) |
| `/safety/human_present` (Bool) | `HumanPresent` | human in the collaborative zone |
| `/safety/reset` (Bool) | `Reset` | clear a latched E-stop / fault |

Safety output: `/safety/state` (JSON) and OPC UA `Safety/SafetyState`,
`Safety/ClearToRun`, `Safety/SpeedScale`. The dashboard shows a colour-coded
safety banner.

## Try it

```bash
ros2 run armik_moveit safety_supervisor
ros2 topic pub --once /safety/estop std_msgs/msg/Bool "{data: true}"        # E-stop -> sorts refused, motion cancelled
ros2 topic pub --once /safety/human_present std_msgs/msg/Bool "{data: true}" # SSM -> reduced speed
ros2 topic pub --once /safety/guard_closed std_msgs/msg/Bool "{data: false}" # guard open -> protective stop
ros2 topic pub --once /safety/reset std_msgs/msg/Bool "{data: true}"         # reset (after release)
```
