# FMEA: the cell's safety supervisor

Failure mode and effects analysis of `armik_moveit/safety_supervisor.py` and the
decision rules it now delegates to `armik_moveit/safety_logic.py`.

**This is not a certification artefact and does not claim to be one.** No
performance level is assigned, no PFH is computed, and nothing here has been
assessed by a notified body. A supervisor written in Python, running on
general-purpose Linux, subscribing over DDS, cannot carry a safety function in
the sense ISO 13849 or IEC 62061 mean. What it can do is be honest about its own
failure modes, and that is what this table is: the analysis a real safety
concept starts from, applied to a simulated cell, so the reasoning is visible
even though the implementation could never be certified.

The severity scale is qualitative on purpose. Assigning RPN numbers to a
simulation would dress a guess up as a measurement.

- **Critical** — the cell can move while a person believes it cannot.
- **Major** — the cell stops when it should not, or a real stop is reported
  wrongly. Nobody is hurt; the cell is untrustworthy or unusable.
- **Minor** — cosmetic or diagnostic only.

---

## 1. E-stop channel

| # | Failure mode | Effect | Cause | Detection today | Severity | Status |
|---|---|---|---|---|---|---|
| 1.1 | E-stop asserted, never received | Cell keeps running with the button pressed | Publisher dead, DDS partition, topic renamed | **None.** `estop` defaults False and there is no heartbeat | Critical | **OPEN** |
| 1.2 | Latch cleared while button held | One operator re-enables a cell another stopped | Reset logic ignoring the live input | `reset_clears_latches` requires `estop_asserted` false; `test_reset_does_nothing_while_the_estop_is_still_pressed` | Critical | Mitigated |
| 1.3 | Latch clears itself on restart | A stop is forgotten across a supervisor crash | State held only in memory | **None.** Latches are not persisted | Critical | **OPEN** |
| 1.4 | E-stop received but motion not cancelled | Arm completes its trajectory after the stop | `/move_action/_action/cancel_goal` unavailable | `_cancel_motion` checks `service_is_ready()` and silently does nothing if not | Critical | **OPEN** |

**1.1 and 1.3 are the two that matter.** Both are the same shape: the supervisor
trusts something it has not verified. 1.1 needs a cyclic safety source (see 5.1).
1.3 needs the latch in non-volatile state, and the correct behaviour on restart
is to come up latched and require a deliberate reset, not to come up clear.

**1.4 is worse than it looks.** The cancel is best-effort by construction: if the
service is not ready the supervisor logs a safe stop it did not actually cause,
so the log says the cell was stopped while the arm is still moving. That is the
same defect class as the AMR project's protective-stop badge over a moving
vehicle: a claim answered by a proxy rather than by the thing claimed. A real
concept does not cancel a goal, it removes drive power.

## 2. Guard interlock

| # | Failure mode | Effect | Cause | Detection today | Severity | Status |
|---|---|---|---|---|---|---|
| 2.1 | Guard state never reported | Cell runs at full speed with the guard state unknown | Safety bridge never started | `guard_closed` starts `None`; unknown is unsafe and holds GUARD_STOP | Critical | **Closed 2026-09-01** |
| 2.2 | Guard opens, message lost | Cell keeps running with the guard open | Dropped DDS sample, publisher death | **None.** No staleness check (see 5.1) | Critical | **OPEN** |
| 2.3 | Guard reported closed by a rogue publisher | Cell runs with the guard open | Any node may publish `/safety/guard_closed` | Partially: OPC UA writes need authentication (`test_opcua_security.py`). The ROS topic itself is unauthenticated | Critical | **OPEN** |
| 2.4 | Guard bounces, cell oscillates | Nuisance stops | Mechanical contact chatter | **None.** No debounce | Minor | Accepted |

**2.1 was real and is fixed.** The supervisor initialised `guard_closed = True`,
so a supervisor whose safety source never came up published RUN at full speed
and looked healthy doing it. Unknown is now a distinct state with its own
reason string, so an operator can tell a dead bus from an open gate.

**2.3 is a property of ROS 2, not of this code.** Any process on the graph can
publish the topic. Real cells solve this with a safety bus that is not the same
network as the control traffic; the OPC UA half is authenticated, the ROS half
is not, and a reader should not be told otherwise.

## 3. Speed and separation monitoring

| # | Failure mode | Effect | Cause | Detection today | Severity | Status |
|---|---|---|---|---|---|---|
| 3.1 | Human present, never reported | Cell runs at full speed beside a person | Detector down, bridge down | `human_present` starts `None`; unknown holds a stop | Critical | **Closed 2026-09-01** |
| 3.2 | Human presence stops being reported mid-cycle | Speed returns to full with the person still there | Detector crash after first message | **None** (see 5.1) | Critical | **OPEN** |
| 3.3 | Safety message missing a field | Consumer falls back to full speed and "safe to run" | `color_sort._on_safety` read `s.get("clear_to_run", True)` and `s.get("speed_scale", 1.0)` | Defaults are now `False` and `0.0`; the node also starts stopped rather than clear | Critical | **Closed 2026-09-01** |
| 3.4 | Safety stream becomes unparseable | Consumer silently keeps the last values it saw | `_on_safety` returned on `JSONDecodeError` without flagging | Now holds a stop, sets FAULT and warns | Critical | **Closed 2026-09-01** |
| 3.5 | Reduced speed applied but never verified | Scaling could silently stop working | Chain is `speed_scale` → `speed_factor` → `max_velocity_scaling_factor` | **Partial.** The chain exists and is readable; no test measures that a commanded velocity actually falls | Major | **OPEN** |
| 3.6 | Separation distance never measured | "Speed and separation" is speed only | Presence is a boolean, not a distance | Documented here | Major | Accepted, by design |

**3.3 and 3.4 are the fail-open defect again, moved downstream.** The first
draft of this table asserted that nothing consumed `speed_scale`. That was
wrong, and checking it rather than asserting it is how the real defect turned
up. The chain is complete and correct: `/safety/state` → `color_sort.speed_scale`
→ `color_sort.speed_factor` → `palletizing.py:278`, which multiplies MoveIt's
`max_velocity_scaling_factor`. What is wrong is the reading, not the plumbing.
`_on_safety` defaults `clear_to_run` to **True** and `speed_scale` to **1.0**,
so a message missing those fields is read as "cell clear, full speed", and an
unparseable message is dropped with no alarm and no record, leaving the last
values in force indefinitely. The supervisor's own inputs were fixed today for
exactly this pattern; the consumer has it too, and the correct defaults here are
`False` and `0.0`.

**3.5 is what remains after 3.3 is fixed.** The scaling is applied, and nothing
measures that it takes effect. A test should command a motion at scale 1.0 and
at scale 0.3 and assert the executed duration changes; without one, the chain
could be broken by an unrelated refactor and every test would still pass.

**3.6 is an honest scope limit.** ISO/TS 15066 speed and separation monitoring
is a function of measured distance and stopping distance. This cell has a
boolean. Calling that SSM is generous, and the code says so.

## 4. Feedback watchdog

| # | Failure mode | Effect | Cause | Detection today | Severity | Status |
|---|---|---|---|---|---|---|
| 4.1 | Joint feedback never arrives at all | Watchdog never fires; a robot silent since boot reads as healthy | `if last_joint and ...` treated "never" as "not late" | `watchdog_expired` measures from node start past `STARTUP_GRACE`; `test_feedback_that_never_arrived_trips_it_too` | Critical | **Closed 2026-09-01** |
| 4.2 | Feedback goes stale mid-run | Detected | Robot or bridge death | `JOINT_TIMEOUT` 1.5 s, latched fault | Critical | Mitigated |
| 4.3 | Feedback fresh but wrong | Supervisor trusts a lying robot | Stuck sensor, replayed bag | **None.** Only arrival is checked, never plausibility | Major | **OPEN** |
| 4.4 | Slow bringup latches a nuisance fault | Operator learns to reset reflexively | Supervisor starts before move_group | `STARTUP_GRACE` 10 s; `test_a_slow_bringup_is_not_a_fault` | Major | Mitigated |

**4.3 is the general form of a lesson this portfolio already learned twice.**
Arrival is a proxy for health. A joint state that never changes while a
trajectory is executing is as broken as one that stopped arriving, and nothing
here would notice.

## 5. Cross-cutting

| # | Failure mode | Effect | Cause | Detection today | Severity | Status |
|---|---|---|---|---|---|---|
| 5.1 | Any safety input goes stale undetected | 1.1, 2.2, 3.2 all follow from this one | Inputs publish **on change only**, so no timeout is possible without false trips | **None**, deliberately | Critical | **OPEN, needs a design change** |
| 5.2 | Supervisor never started | No supervision at all; nothing else notices | No launch file referenced it; it was run by hand | `sort_cell_twin.launch.py` now starts it, and `color_sort` holds a stop until it reports | Critical | **Partly closed 2026-09-01** |
| 5.2b | Supervisor dies mid-run | Its last verdict stands forever | Crash, OOM | **None.** No respawn, no liveness check on `/safety/state` | Critical | **OPEN** |
| 5.3 | Single channel throughout | No redundancy, no diversity | One node, one path, one language | Documented here | Critical | Accepted, by design |

**5.1 is the root of three Critical rows and is not closed, on purpose.**
`opcua_server.py` publishes the safety signals on change specifically so it does
not fight the GUI as a second publisher. A timeout against on-change publishers
trips on a healthy, unchanging cell, and a watchdog that false-trips gets
switched off, after which there is no watchdog. Closing it properly means one
cyclic safety source owning these signals and emitting a heartbeat, the way a
PROFIsafe or FSoE F-host does. That is the next piece of work, and it is a
design change rather than a patch.

**5.2 was embarrassing and cheap to fix, and is now half fixed.** No launch file
referenced the supervisor at all; it was started by hand or forgotten, and
nothing anywhere reported its absence. `sort_cell_twin.launch.py` now brings it
up with the rest of the cell, and the consumer holding a stop until it reports
means a forgotten supervisor is loud instead of silent.

**What is left is 5.2b, and it is the harder half.** A supervisor that starts
and later dies leaves its last verdict standing forever, because `/safety/state`
is read on arrival and never checked for liveness. That is 5.1 again from the
other end: the consumer needs to treat an aged verdict as no verdict.

---

## What this analysis changed

Three rows moved to Closed on 2026-09-01, and all three were found by writing
the table rather than by any test failing:

- **2.1 and 3.1**, the fail-open inputs. Both defaults asserted the safe value
  before any source had spoken.
- **4.1**, the watchdog that could not fire for a robot that never reported.

Each now has a test that was **shown to fail** before it was made to pass.
Seeding the original fail-open back in initially failed only two of the new
tests, and not the one named for the defect: with a tri-state input `not None`
is `True`, so an unreported guard stopped the cell by coincidence while
reporting "guard open". The tests now assert the reason as well as the stop, and
the seeded defect fails four.

## The honest summary

Of 22 identified failure modes, **6 were closed today** (2.1, 3.1, 4.1, 3.3,
3.4, and half of 5.2), **3 are mitigated with tests, 3 are accepted design
limits, and 10 remain open**, of which 8 are Critical. The largest single item
is 5.1; 1.1, 2.2, 3.2 and 5.2b all collapse into it.

The most valuable rows are not the closed ones. They are **3.3 and 3.4**, and
they were found by checking a claim this document had already made. The first
draft asserted that nothing consumed `speed_scale`. Reading the code to confirm
it showed the opposite, that the chain into MoveIt's velocity scaling is intact,
and showed something worse than the imagined defect: the consumer defaults a
missing `clear_to_run` to `True` and a missing `speed_scale` to `1.0`. The exact
fail-open pattern fixed in the supervisor this morning is still present one node
downstream, where a malformed safety message reads as permission to run at full
speed.

That is the argument for writing the table at all. Three of these rows were
closed by tests; two of the most serious were found by a document being made to
justify itself.
