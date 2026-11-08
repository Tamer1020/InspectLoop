# InspectLoop — Uncertainty-Aware Autonomous Gauge Inspection Robot

[![CI](https://github.com/Tamer1020/InspectLoop/actions/workflows/ci.yml/badge.svg)](https://github.com/Tamer1020/InspectLoop/actions/workflows/ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue) ![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange) ![Nav2](https://img.shields.io/badge/Nav2-1.3-green)

A differential-drive mobile robot that patrols a simulated industrial warehouse, **reads analog
pressure gauges with OpenCV, estimates how far each reading can be trusted, and — when an
observation is unreliable — autonomously decides to look again, drive to a better viewpoint with
Nav2, or honestly report that no valid reading is possible.**

```
observe ──► value ± σ + quality flags ──► decision policy
   ▲                                         ├─ ACCEPT      → record value ± 2σ, check alarm limits
   │                                         ├─ RETRY_SAME  → transient problem: observe again
   └──────── Nav2 ◄── viewpoint planner ◄────┼─ MOVE_NEW    → geometric problem: new pose
                                             └─ UNRESOLVED  → no reliable reading within budget
```

| Frontal view: dial hidden by a hanging conduit box → **MOVE_NEW** | Robot drove to −45° → **ACCEPT 6.79 ± 0.15 bar** (truth 6.80) |
|---|---|
| ![occluded](docs/images/pg12_frontal_occluded.png) | ![accepted](docs/images/pg12_moved_accepted.png) |

![warehouse](docs/images/gazebo_warehouse.png)

## Verified demo result (Gazebo Harmonic, headless run, archived in `docs/evidence/demo_run`)

| Gauge | Situation | Decisions | Reported (95 %) | Truth |
|---|---|---|---|---|
| PG-11 | clear | ACCEPT | 4.20 ± 0.11 bar | 4.20 |
| PG-12 | conduit box in front of the dial (invisible to the 2-D LiDAR) | MOVE_NEW → ACCEPT | 6.79 ± 0.15 bar | 6.80 |
| PG-13 | crate stack in front of the gauge | RETRY_SAME → MOVE_NEW → … → **UNRESOLVED** | no value | 3.10 |
| PG-14 | above alarm limit | ACCEPT, **alarm HIGH** | 8.70 ± 0.11 bar | 8.70 |

Max error of accepted readings 0.0085 bar (0.09 % FS); all within the reported 2σ. Same decisions in
three independent full runs (two headless, one with Gazebo GUI + RViz). Camera and LiDAR fault
injection tested: camera loss → UNRESOLVED (no stale frame reused); LiDAR loss → robot stopped from
0.40 m/s within 1 s, mission aborted safely. 116 automated tests pass (C++ gtest + pytest).
Details and limitations: [docs/TEST_RESULTS.md](docs/TEST_RESULTS.md),
[docs/KNOWN_LIMITATIONS.md](docs/KNOWN_LIMITATIONS.md).

## What is inside

| Package | Highlights |
|---|---|
| `inspectloop_description` | Xacro diff-drive robot, computed inertias, 2-D GPU LiDAR, RGB camera, REP-103 frames |
| `inspectloop_gazebo` | Warehouse world, gauge panels, occluders and the Nav2 map **generated from one layout file**; ros_gz bridges (LiDAR/camera in separate processes for fault injection); supervised `gz sim` |
| `inspectloop_navigation` | AMCL + Nav2 (MPPI) parameters, map, RViz config |
| `inspectloop_safety` (C++17) | `cmd_vel_gate`: last-line safety filter (LiDAR loss, stale command, obstacle corridor, e-stop); `health_monitor`: stream rate/age + TF diagnostics |
| `inspectloop_vision` | Gauge reader: ArUco → homography → rectification → **bezel ellipse refinement** → needle angle; σ model + quality flags; ROS service with fresh-frame bursts |
| `inspectloop_mission` | Decision policy, map-aware viewpoint planner, Nav2 action-client mission manager, JSON/Markdown/HTML reports, ground-truth evaluator |
| `inspectloop_interfaces` | `GaugeReading`, `GaugeDecision`, `MissionStatus`, `ReadGauge` |
| `inspectloop_bringup` | One launch file for everything |

Architecture, node graph, TF tree, QoS and design decisions: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Quick start

```bash
# Ubuntu 24.04 + ROS 2 Jazzy + Nav2 + ros_gz (see docs/SETUP.md)
cd ~/InspectLoop
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
colcon test && colcon test-result --verbose

# full demo with Gazebo GUI + RViz
ros2 launch inspectloop_bringup inspectloop.launch.py
# headless, exits when the mission is done
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false exit_on_mission_end:=true
# results
ls ~/inspectloop_runs/ && ros2 run inspectloop_mission evaluate_run ~/inspectloop_runs/<mission_id>
```

More: [docs/RUN_DEMO.md](docs/RUN_DEMO.md) · [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md)

## How the robot decides

* **Reading** – 5 frames stamped after arrival (+0.5 s settle) are read independently and fused:
  value = median, σ² = (1.25·spread/√n)² + σ_res² + 0.03², where σ_res is the value change of a
  0.5 px needle shift along the foreshortened dial axis.
* **Flags** – `OCCLUDED`, `OBLIQUE_VIEW`, `TOO_FAR`, `LOW_NEEDLE_CONTRAST`, `TAG_NOT_VISIBLE`,
  `TAG_INTERMITTENT`, `INCONSISTENT_FRAMES`, `BLURRED`, `NO_IMAGE`, `OUT_OF_SCALE`.
* **Policy** – ACCEPT if σ ≤ 0.15 bar, ≥ 3 valid frames and no blocking flag; transient-looking
  problems → RETRY_SAME (once per pose); geometric problems → MOVE_NEW with a hint
  (`different_side` / `frontal` / `closer`); camera fault or budget spent (3 viewpoints / 8
  observations) → UNRESOLVED.
* **Viewpoints** – 15 candidates (1.2/1.5/1.9 m × 0°/±30°/±45°) filtered for clearance on the
  occupancy map; occluded directions are blocked at every distance. Occlusion is *discovered by
  looking*, not predicted from the 2-D map.

## Honest scope

This is a simulation portfolio project. The dial is located with a printed fiducial, the imagery is
rendered (no glare/blur/distortion), thresholds are hand-set and the σ model is a heuristic that was
checked — not proven — against ground truth. See [KNOWN_LIMITATIONS](docs/KNOWN_LIMITATIONS.md).

## License

Apache-2.0 — see [LICENSE](LICENSE).
