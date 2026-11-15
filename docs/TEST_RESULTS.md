# Test results

> **Release candidate (V3.2, 2026-11-15)** — §V3.1/V3.2 below. §V3 (after the independent review
> of V2, 2026-11-09; detail in `docs/ASTRA_V3_REMEDIATION_REPORT.md`), §0 (V2) and §1–7 (V1) are
> kept for traceability.

## V3.1 / V3.2 results (release candidate)

Two HIGH findings of a further audit, fixed and re-verified:
**F3** — `sectorCoverage()` iterated 2π / `angle_increment` directions (a 720-beam scan with
`angle_increment = 1e-9` passed validation and cost ~6.3e9 lookups, blocking the gate's watchdog
timer); **F4** — a SIGINT between `send_goal_async()` returning and goal registration could leave
a sent goal untracked while shutdown reported success. V3.2 hardens F4 further: unknown-ownership
goals are cancelled by their own goal ID and confirmed through Nav2's status topic.

**Build / tests** (`ROS_DOMAIN_ID=87 colcon build && colcon test`, all 8 packages):
**500 results, 0 errors, 0 failures, 0 skipped** — 489 test cases (C++ GoogleTest 68, vision
pytest 208, mission pytest 187, gazebo 7 / description 7 / navigation 6 / bringup 6) + 11 CTest
wrapper entries.

| New regression tests | Cases | Before the fix |
|---|---|---|
| `inspectloop_safety/test/test_v3_1_scan_workload.cpp` — the audit scan → `BAD_METADATA`; 20 malformed-metadata cases fail closed; ≤ 3·(n+6) lookups; identical to the V3 algorithm on 4 800 plausible cases; watchdog scheduling (scan callback < 5 ms, tick gap < 50 ms) | 9 | V3 took 20.19 s for one sector, `assessScan` did not return in 30 s (`docs/evidence/astra_v3_1_live/before_fix_f3_unbounded_sector_loop.txt`) |
| `inspectloop_mission/test/test_v3_1_nav_send_interrupt.py` — SIGINT injected (sys.settrace) right after `send_goal_async()` returns, and at every bytecode of the send → accept → track → result path | 9 | 7 of 9 failed on V3: "false clean shutdown after SIGINT at run:272" (`…/before_fix_f4_send_registration_gap.txt`) |
| `inspectloop_mission/test/test_v3_2_unknown_goal_fallback.py` — lost goal handle cancelled by ID; fail closed on unknown ID / refused / unanswered cancel / unobservable status / robot still moving; foreign goals never cancelled; opcode sweep: no stranded goal with a cooperative server | 17 | new API (V3.2); behaviour before: live control run below |
| `inspectloop_mission/test/test_v3_2_goal_id_fallback_ros.py` — the same over a **real rclpy action server**: unknown ID → `ERROR_UNKNOWN_GOAL_ID`, finished goal → `ERROR_GOAL_TERMINATED`, other client's goal untouched | 4 | – |

The opcode sweep also exposed and fixed two latent cleanup defects: a second Ctrl-C could escape
`cancel_all()`'s final check, and a SIGINT inside `_track()` left a goal whose termination could
never be confirmed.

**Live integration** (Gazebo Harmonic + Nav2, headless, developer machine):

| Test | Observed | Evidence |
|---|---|---|
| Normal 4-gauge mission (V3.2) | COMPLETED, exit 0; PG-11 4.212, PG-12 6.797 (MOVE_NEW → −45°), PG-13 UNRESOLVED, PG-14 8.699 alarm HIGH; max \|err\| 0.012 bar, 0 incorrect acceptances | `docs/evidence/astra_v3_2/demo/` |
| Malformed scan while driving (720 beams, 1e-9) (V3.1) | gate `STOPPED_SCAN_FAULT (scan BAD_METADATA)`; robot 0.40 → 0.000 m/s; `/cmd_vel_safe` max gap 54 ms; mission ABORTED, goal CANCELED | `docs/evidence/astra_v3_1_live/f3_tiny_inc/` |
| SIGINT 10 ms after goal #1 SENT (V3.1, V3.2) | CANCEL_CONFIRMED; Nav2 status 1× CANCELED; robot 0.000 m/s for 10 s | `…/astra_v3_1_live/f4_dispatch_1/`, `…/astra_v3_2/sigint_dispatch_1/` |
| SIGINT right after goal #3 SENT (V3.1, V3.2) | late acceptance cancelled and confirmed; Nav2 2× SUCCEEDED + 1× CANCELED; robot 0.000 m/s | `…/f4_dispatch_3/`, `…/sigint_dispatch_3/` |
| Lost goal handle, real Nav2 (V3.2 harness) | with fallback: cancelled by ID, Nav2 CANCELED, robot 0.000 m/s, cleanup True. Control without fallback: goal kept EXECUTING, robot up to 0.40 m/s for ~13 s, cleanup False | `docs/evidence/astra_v3_2/live_unknown_goal/` |

CI: the same 500 results pass in GitHub Actions (Release build, `ros:jazzy-ros-base`).
Not verified: gate e-stop service failure and gate restart during shutdown, any hardware.

## V3 results

**Build / tests.** `colcon build` 8 packages, no warnings; `colcon test` → **460 results, 0 errors,
0 failures, 0 skipped** (450 test cases: C++ GoogleTest 59, vision pytest 208, mission pytest 157,
other packages 26; + 10 CTest wrappers), `ROS_DOMAIN_ID=87`. Each V3 regression test was run
against the V2 code first: `docs/evidence/astra_v3/before_fix_*.txt`.

**Synthetic CV evaluation** (`tools/heldout_uncertainty_eval.py`, tolerance 0.10 bar; final set
frozen and committed before any reader change):

| Set | V2 accepted / incorrect | V3 accepted / incorrect | V3 clean · difficult · out-of-scale |
|---|---|---|---|
| dev 20261108 (n=400) | 210 / 1 (#183) | 209 / 0 | 185/208 · 24/156 · 0/36 |
| Astra 20261109 (n=200) | 97 / 1 (#94, clean, 0.1013) | 97 / 1 (same) | 89/95 · 8/90 · 0/15 |
| frozen final 20261201 (n=400) | 208 / 0 | 208 / 0 | 189/204 · 19/158 · 0/38 |

**Gazebo / Nav2 (developer-reported, `docs/evidence/demo_run_v3*`).** Two full four-gauge runs:
PG-11 4.215 / 4.192 (4.20), PG-12 MOVE_NEW → ACCEPT 6.837 / 6.787 (6.80), PG-13 UNRESOLVED,
PG-14 8.679 / 8.710 (8.70, alarm HIGH); max |error| 0.037 / 0.014 bar; 0 incorrect acceptances.

**Fault injection (V3, `docs/evidence/astra_v3/faults/`).** Front-sector NaN (±35°) →
`BLOCKED_UNOBSERVED`, stop ≤ 1.2 s, ABORTED; LiDAR kill / all-NaN / empty scans → stop ≤ 1.5 s,
ABORTED; camera loss → UNRESOLVED `CAMERA_UNAVAILABLE`; frozen / future frames → UNRESOLVED
`IMAGE_INVALID`; world-time reset → observation invalidated (clock epoch), Nav2 does not recover →
`NAVIGATION_FAILURE`, no value accepted; SIGINT → goal cancel confirmed, report ABORTED. One
scan_nan run was invalid (injector stalled) and was repeated.

> **v2 (after the Astra audit fixes, 2026-11-08)** — see §0 below and `docs/ASTRA_FIX_REPORT.md`.
> Sections 1–7 document the v1 baseline (tag `v1.0-final`) and are kept for traceability; where v1
> wording used "95 %" for `value ± 2σ`, that claim has been withdrawn (Astra F6).

## 0. v2 results (superseded by V3)

### 0.1 Automated tests
`colcon build` (8 packages, no warnings) and `colcon test` → **260 results, 0 errors, 0 failures**
(252 test cases + 8 CTest wrapper entries), run with `ROS_DOMAIN_ID=87` (isolated from the running
simulation).

| Package | Cases | New regression tests (Astra) |
|---|---|---|
| inspectloop_safety (C++ gtest) | 32 | `test_astra_safety_existing_api` (5), `test_scan_validation` (9) |
| inspectloop_vision | 79 | `test_astra_regressions` (34), `test_frame_freshness` (10) |
| inspectloop_mission | 115 | `test_astra_policy` (40), `test_nav_goal` (13), `test_nav_goal_ros` (5, real rclpy action server), `test_observation_guard` (12), `test_astra_uncertainty_labels` (4) |
| gazebo / description / navigation / bringup | 7 / 7 / 6 / 6 | – |

Every regression test was run against the v1 code first; failing outputs:
`docs/evidence/astra/*_before_fix.txt`.

### 0.2 Held-out evaluation (synthetic, `tools/heldout_uncertainty_eval.py`, n = 400, seed 20261108)

| | v1 code | v2 code |
|---|---|---|
| accepted (yield) | 324/400 | 210/400 |
| incorrect acceptance (\|err\| > 0.1 bar) / accepted | 21/324 | **1/210** |
| accepted \|err\| mean / max | 0.028 / 0.541 bar | 0.015 / 0.118 bar |
| out-of-scale needles accepted | 27/36 | 0/36 |

σ_h is a heuristic; the within-±2σ_h count (210/210 for v2) is a property of this sample only.

### 0.3 Gazebo / Nav2 integration (v2)
Final four-gauge run (`evidence/demo_run_v2/`, exit 0): PG-11 4.200 (truth 4.20), PG-12 6.794
(6.80; MOVE_NEW → ACCEPT at −45°), PG-13 UNRESOLVED, PG-14 8.702 (8.70, alarm HIGH); max |error|
0.006 bar; 0 incorrect acceptances. Provenance of every observation: own viewpoint epoch, robot
settled, first frame ≥ 0.5 s after the request, capture pose known, 0.0 m motion.

### 0.4 Fault injection (v2, isolated runs, `docs/evidence/astra/faults/`)
| Fault | Result |
|---|---|
| camera killed | NO_IMAGE → RETRY_SAME → UNRESOLVED |
| LiDAR killed while driving | gate `scan timeout`, 0.40 → 0 m/s within 1.0–1.5 s, mission ABORTED |
| LiDAR → all-NaN scans | gate `TOO_MANY_INVALID_BEAMS`, stop ≤ 0.8 s, ABORTED via gate state |
| LiDAR → empty scans | gate `scan EMPTY`, stop ≤ 0.8 s, ABORTED via gate state |
| camera → frozen stream | FROZEN_FRAMES → RETRY_SAME → UNRESOLVED |
| camera → future-stamped frames | 30 future frames rejected → NO_IMAGE → UNRESOLVED |
| SIGINT to the mission while driving | goal cancel confirmed (CANCELED), stop ≤ 1.5 s, report ABORTED "interrupted" |

Details, bugs found by these runs and run conditions: `docs/ASTRA_FIX_REPORT.md`.


All results below were produced on 2026-11-08 on the development machine (WSL2, Ubuntu 24.04,
ROS 2 Jazzy, Gazebo Harmonic, 16 cores). Raw artefacts are in `docs/evidence/`. Nothing in this file
is estimated or copied from elsewhere; where something was not verified it says so.

## 1. Build

```
colcon build --symlink-install      -> Summary: 8 packages finished (no warnings with -Wall -Wextra -Wpedantic on the C++ package)
```

## 2. Automated tests (`colcon test`)

`colcon test-result --all` → **126 results, 0 errors, 0 failures, 0 skipped**
(120 test cases + 6 CTest wrapper entries). Re-verified on a fresh `git clone` of the repository
(clean `colcon build` + `colcon test`).

| Package | Framework | Cases | What is covered |
|---|---|---|---|
| inspectloop_safety | gtest (C++) | 18 | TopicMonitor (rate, stale, degraded, clock jump), velocity gate priorities, clamping, slow/stop zone, corridor clearance |
| inspectloop_vision | pytest | 35 | needle reading on textures and perspective/noise/blur renders, occlusion / hidden tag / off-image / no-frame flags, **σ calibration**, constants ↔ textures, **4 real Gazebo frames** |
| inspectloop_mission | pytest | 41 | decision policy (all 4 decisions, escalation, budgets), viewpoint planner (feasibility on the real map, hints, blocked directions), config validation, report + evaluator |
| inspectloop_gazebo | pytest | 7 | committed world/map/textures match the layout, overhanging occluder absent from map, gauges not buried in walls (regression), texture tag |
| inspectloop_description | pytest | 7 | URDF tree, frames, wheel joints, inertia validity, plugin/geometry consistency |
| inspectloop_navigation | pytest | 6 | AMCL initial pose = spawn, frames, robot radius, velocity limits, map file |
| inspectloop_bringup | pytest | 6 | every launch file builds a LaunchDescription |

### Vision accuracy and σ calibration (synthetic, ground truth exact)

* Undistorted textures: |error| < 0.02 bar for all four gauges.
* Effect of the bezel ellipse refinement (introduced after the first measurements): frontal synthetic
  views at 0.5× scale had errors of up to **−0.63 bar** with the homography alone; after moving the tag
  quiet zone to one full marker cell and adding the bezel fit, the same sweep gives **≤ 0.05 bar**
  except for tiny dials seen very obliquely (scale 0.2, strong skew), where errors up to 0.74 bar
  remain — those observations get σ above the acceptance limit.
* Calibration sweep (`test_sigma_is_calibrated_on_synthetic_views`): 96 observations (4 gauges ×
  4 scales × 3 perspectives × 2 blur levels, noise σ = 3 grey levels), none flagged;
  mean |error| 0.011 bar, max 0.072 bar; **100 % within 1σ, max normalised error 0.81σ** → the σ model
  is *conservative* on these conditions (it over-states uncertainty), which errs on the safe side.

## 3. Simulation: environment and sensors (headless)

| Check | Result |
|---|---|
| Gazebo world loads, robot spawns (`ros_gz_sim create`) | ✔ |
| `/scan` rate | 9.6–9.7 Hz (`ros2 topic hz`) |
| `/odom` frame ids | `odom` → `base_footprint` ✔ |
| `/camera/image_raw` | 640×480 rgb8, `frame_id = camera_optical_link` ✔ |
| TF odom → camera_optical_link | translation (0.255, 0, 0.180), optical rotation ✔ |
| Wheel odometry while driving | 0.40 m/s measured on `/odom` during navigation (fault test) |
| Nav2 lifecycle (localization + navigation incl. route/docking servers) | "Managed nodes are active" ✔ |

## 4. End-to-end gauge missions (ground truth only used afterwards)

Three complete missions with the final code, two headless (g2, g3) and one with the Gazebo GUI and
RViz (gui). Decisions were identical in all three:

| Gauge | Truth | Decision sequence | g2 | g3 (archived) | gui |
|---|---|---|---|---|---|
| PG-11 clear | 4.20 | ACCEPT | 4.187 ± 0.104 | 4.205 ± 0.108 | 4.208 ± 0.105 |
| PG-12 occluded from the front | 6.80 | MOVE_NEW → ACCEPT (at −45°) | 6.769 ± 0.158 | 6.792 ± 0.151 | 6.812 ± 0.162 |
| PG-13 behind crate | 3.10 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED | UNRESOLVED | UNRESOLVED | UNRESOLVED |
| PG-14 above alarm | 8.70 | ACCEPT, alarm HIGH | 8.688 ± 0.109 | 8.701 ± 0.107 | 8.697 ± 0.105 |
| max \|error\| of accepted | | | 0.031 | **0.0085** | 0.012 |
| within reported 2σ | | | 3/3 | 3/3 | 3/3 |

(± values are the reported 95 % half-widths = 2σ.)

Archived run g3 (`docs/evidence/v1_demo_run/`): mission COMPLETED, 204 s sim / 234 s wall, 7 viewpoints
visited, 7/7 navigations succeeded on the first attempt, final position errors 0.037–0.156 m
(mean 0.083 m), yaw errors 0.14–0.19 rad, return home SUCCEEDED, launch exit code 0.

Per-observation evidence (g3):

| Obs | Viewpoint | Flags | Visibility | Needle conf | View angle | Dial radius | Decision |
|---|---|---|---|---|---|---|---|
| PG-11 #1 | V0 frontal 1.5 m | – | 0.997 | 0.98 | 21° | 71 px | ACCEPT |
| PG-12 #1 | V0 frontal 1.5 m | OCCLUDED, LOW_NEEDLE_CONTRAST | 0.008 | 0.005 | 21° | 70 px | MOVE_NEW (different_side) |
| PG-12 #2 | V3 −45° 1.5 m | – | 0.87 | 0.46 | 46° | 57 px | ACCEPT |
| PG-13 #1–6 | V0, V1 (−30°), V2 (+30°) | TAG_NOT_VISIBLE | – | – | – | – | RETRY_SAME/MOVE_NEW → UNRESOLVED |
| PG-14 #1 | V0 frontal 1.5 m | – | 0.995 | 0.96 | 20° | 71 px | ACCEPT |

The frontal "view angle" is ≈ 20° because the camera (0.18 m high, tilted up 14°) looks up at the dial.
The PG-12 acceptance at −45° passed with modest margins (visibility 0.87 vs 0.85 limit) — see
KNOWN_LIMITATIONS.md.

Images: `docs/evidence/v1_demo_run/images/` (raw, annotated, rectified for every observation) and the
Gazebo GUI screenshot `docs/evidence/gui/gazebo_gui_pg12.png` (taken with the `/gui/screenshot`
service during the GUI run).

## 5. Failure handling

| Test | How | Observed result | Evidence |
|---|---|---|---|
| Physical occlusion (hanging conduit box, invisible to LiDAR/map) | PG-12 in every mission | OCCLUDED detected → MOVE_NEW → −45° view → ACCEPT, error ≤ 0.031 bar | `v1_demo_run/` |
| No valid reading possible (crate) | PG-13 in every mission | 3 viewpoints, 6 observations → UNRESOLVED, no value published | `v1_demo_run/` |
| **Camera failure** | `scripts/fault_test.sh camera` kills `gz_camera_bridge` while driving to PG-11 | health_monitor: `/camera/image_raw: STALE`; reader: `NO_IMAGE (no fresh frame within 5.0 s; frames received so far: 111)` — no stale frame reused; RETRY_SAME → **UNRESOLVED** "camera delivers no frames: moving cannot fix a sensor fault"; mission completed, exit 0 | `fault_camera/` |
| **LiDAR failure** while driving | `scripts/fault_test.sh lidar` kills `gz_lidar_bridge` | `cmd_vel_gate -> STOPPED_SCAN_FAULT`; measured speed **0.400 m/s → 0.000 m/s within 0.5–1.0 s**; `/scan: STALE`; Nav2 goal aborted; mission **ABORTED** "LiDAR stream failed – stopping for safety"; report written; exit 0 | `fault_lidar/` (`odom_speed.txt`) |
| System not ready | first integration run: mission subscribed `/amcl_pose` volatile and missed the latched message | mission ABORTED after 180 s with "missing /amcl_pose", report written (bug fixed: transient-local subscription) | log in this file's history |
| Vision node crash | earlier pipeline: an rclpy logging error killed the vision node mid-mission | mission recorded the inspection as failed and continued (bug fixed + service wrapped so exceptions return an error result) | – |

## 6. Bugs found by running the system (and fixed)

1. Panel at x = 11.89 m was inside the 0.2 m east wall (inner face 11.8) → camera saw only wall.
   Fixed in the layout; regression test added.
2. ArUco tag had an 8 px quiet zone; the dark panel border merged with the tag at low resolution,
   shifting corners by ~3.6 px and biasing readings by up to 0.6 bar. Fixed with a one-cell quiet zone.
3. Planner "move closer" sent the robot back along an occluded line of sight (run g1, archived in
   `docs/evidence/run_g1_before_fixes/`). Fixed: failed directions are blocked at every distance.
4. σ model double-counted foreshortening and rejected a correct 45° reading (g1: 6.81 bar, σ 0.17).
   Replaced by the minor-axis resolution term; calibration test added.
5. `gz sim` server orphaned after shutdown; launch exit code 1 from a double shutdown. Fixed with the
   supervised wrapper and a guarded exit handler (all final runs: exit 0, no leftover processes).

## 7. Not verified / limitations of the evidence

* RViz2 and the Gazebo GUI were started and ran the full mission (windows confirmed with `xwininfo`,
  Gazebo screenshot captured), but RViz rendering was **not visually inspected**: WSLg does not
  allow root-window capture, and RViz logged a GLSL link error for the indexed-colour map shader
  (WSL Mesa driver), so the costmap colouring may not render on this machine.
* The GitHub Actions workflow was written but **has not run** (no repository published from here).
* Only the scenarios above were tested; no statistics over many randomised worlds.
