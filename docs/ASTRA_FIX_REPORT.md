# Astra Engineering Audit — Fix Report (F1–F6)

> **Superseded in parts by V3** — the independent review of V2 rejected it for two remaining HIGH
> defects (F3 forward-sector blindness, F4 goal ownership before acceptance). See
> [ASTRA_V3_REMEDIATION_REPORT.md](ASTRA_V3_REMEDIATION_REPORT.md). Errata to this file: the
> statement below that fault-test launch exit codes were 1 "because bridges are killed" is wrong
> (harness bug, see the V3 report); the V2 hand-over message's "13 new + 3 updated test files" is
> wrong (9 added + 2 modified; the file list at the end of this report is correct).

Scope: the six HIGH findings of the independent review. Baseline = git tag `v1.0-final`
(= `InspectLoop_Portfolio_Final.zip`, SHA-256 `f4ae5d4c…4d693`, kept unchanged).
For every finding a regression test was written first and run against the unfixed code; the
failing output is archived in `docs/evidence/astra/`. All numbers below come from runs on
2026-11-08 (WSL2, ROS 2 Jazzy, Gazebo Harmonic). The lower-priority findings F7–F9 were not
addressed.

## Summary

| ID | Finding | Status |
|---|---|---|
| F1 | Wrong gauge readings could be accepted (nearby distractor) | Fixed; residual risk for lines ≲ 3–5° from the needle under heavy blur (1/210 held-out) |
| F2 | Out-of-scale / malformed values could pass | Fixed |
| F3 | LiDAR safety validation insufficient | Fixed (unit + Gazebo fault injection) |
| F4 | Nav2 goal cancellation / lifecycle races | Fixed (fake-client + real rclpy action server tests; Gazebo interrupt test) |
| F5 | Camera timestamps / observation freshness | Fixed (unit tests + live provenance + Gazebo injection) |
| F6 | Unsupported "95 %" uncertainty claims | Fixed (labels removed; held-out evaluation added) |

---

## F1 — Incorrect gauge readings can be accepted

**Original finding.** A dark radial distractor 8° from the needle produced an accepted reading of
4.508 bar (truth 4.2, σ 0.034, no flags).

**Reproduction.** `test_astra_regressions.py` on the v1 reader: the 8° distractor gave a *clean*
4.503 bar reading (σ 0.059); the ±8° and +5° cases and the "equally dark second line" cases failed
(`evidence/astra/f1_f2_before_fix.txt`).

**Root cause.** The needle angle was the arg-max of the darkness profile averaged over all radii.
Near the pivot the needle is ~±11° wide, so a line within that width merged into the same peak and
pulled the estimate; the competing-peak test ignored everything within ±12°, and the visibility
check excluded a fixed ±10° wedge around the *estimated* needle, so the distractor itself never
lowered visibility. Nothing checked that the evidence actually looked like one needle.

**Fix.** (`src/inspectloop_vision/inspectloop_vision/gauge_reader.py`)
* Per-ring analysis on 16 rings (0.15–0.52 R): each ring must show one lobe; ring angles must
  agree (`needle_support` ≥ 0.70, else `NEEDLE_PARTIAL`); the best competing lobe relative to the
  needle (`needle_ambiguity`, limit 0.45) → `AMBIGUOUS_NEEDLE`.
* **Needle-model residual**: a tapered, blurred needle model of the known geometry is fitted within
  ±20° of the estimate; unexplained dark energy / needle energy > 0.075 → `AMBIGUOUS_NEEDLE`.
  Threshold chosen from measurements (clean ≤ 0.071 vs. wrong-reading distractors ≥ 0.064, the
  overlap being 3° offsets) — not by raising an arbitrary confidence threshold.
* Below a minor dial radius of 40 px clean and distractor residuals overlap (measured), so
  `TOO_FAR` now requires ≥ 40 px (was 28 px): the image is not adequate evidence.
* Visibility excludes only the needle's geometric footprint (+ blur + 1°), not a fixed wedge.
* Per-ring angle = darkness-weighted lobe centroid (more robust than arg-max + parabola).
* Policy: `AMBIGUOUS_NEEDLE` / `NEEDLE_PARTIAL` are viewpoint problems → MOVE_NEW, hint
  `different_side` (`decision_policy.py`).

**Regression tests.** `src/inspectloop_vision/test/test_astra_regressions.py` (radial distractors at
±5/±8/12/20/−35°, equally dark second lines, white occluders over the tip and over most of the
needle, a non-radial line across the dial, a shadow sector, clean accuracy at 7 values and 3
perspectives); `test_astra_policy.py::test_needle_evidence_flags_are_viewpoint_problems`.

**Verification.** All pass. Held-out evaluation (§F6): incorrect acceptances 1/210 vs. 21/324 for
the v1 code on the same 400 samples. Live Gazebo: from the −45° view of PG-12 in the run *before*
the world correction, the new reader was pulled toward a dark bar crossing the dial (5.86 / 6.17 bar,
truth 6.8) and **flagged** it — nothing wrong was accepted.

**Finding made while verifying F1 — world modelling error.** That dark bar was the support arm of
the PG-12 conduit box, which in v1 ran from the box straight back to the rack **through the gauge
face** (physically impossible), crossing the dial from every viewpoint. v1 accepted 6.79 bar there
only because its averaged profile happened to favour the needle base. The box (the actual occluder)
was kept unchanged; its support became a thin rod beside the panel to an arm at 1.9 m
(`warehouse_layout.yaml`, `generate_warehouse.py`; the map is unchanged because both are above the
LiDAR plane). Images: `evidence/astra/pg12_bracket/`. The real Gazebo regression frame from that v1
viewpoint is now expected to be rejected (`test_gazebo_frame_oblique_view_with_bracket_over_needle_is_not_accepted`).

**Remaining risk.** A line within ~3–5° of the needle under heavy blur can merge into one lobe that
passes all tests (held-out: 1 incorrect acceptance, error 0.118 bar). Yield is lower (§F6).

## F2 — Invalid numerical values can pass acceptance

**Original finding.** `OUT_OF_SCALE` could not trigger (search bounds ±5°, check at ±0.3 bar);
−0.1 / 10.1 bar passed. The policy accepted None/NaN/Inf/negative σ in some cases.

**Reproduction.** v1 reader: needles rendered at −0.1, −0.3, 10.1, 10.4 bar → no flag; NaN/Inf frame
values leaked into the fusion (`f1_f2_before_fix.txt`). v1 policy: 23 of 38 contract tests failed,
e.g. `has_value='yes'`, missing flags list, values −0.1…25 bar accepted
(`f2_policy_before_fix.txt`).

**Root cause.** The search sweep (scale ± 5°) clamped any needle beyond the scale to ≤ 0.19 bar
outside it, below the 0.3 bar check; fusion trusted `value is not None`; the policy compared numbers
without type/finiteness checks or a range check.

**Fix.** Reader: sweep ±20° beyond the scale; `OUT_OF_SCALE` beyond ±0.05 bar of the printed scale;
needle at the sweep edge → no value + `OUT_OF_SCALE`; only finite values enter the fusion; σ must be
finite and > 0 or no value is reported; values are **never clamped**. Policy:
`contract_violations()` checks `has_value is True`, finite real value inside the scale, finite σ > 0,
integer `frames_valid` ≥ 0, list of flags; malformed → RETRY_SAME then UNRESOLVED (never MOVE_NEW,
never ACCEPT); unknown flags block acceptance.

**Regression tests.** `test_astra_regressions.py::test_f2_*` (8 cases),
`src/inspectloop_mission/test/test_astra_policy.py` (38 cases incl. boundaries 0.0, 10.0, ±0.04/0.06
around the tolerance, σ exactly at / just above the limit).

**Verification.** All pass. Held-out: 0/36 out-of-scale needles accepted (v1: 27/36).

**Remaining risk.** The ±0.05 bar tolerance is a design choice; a true reading within 0.05 bar of a
scale end is reported as measured (possibly slightly outside the scale) without a flag.

## F3 — LiDAR safety validation is insufficient

**Original finding.** Empty or all-NaN scans could permit 0.4 m/s; clock rollback did not invalidate
health; negative command age passed.

**Reproduction.** `test_astra_safety_existing_api.cpp` against the v1 library: 5/5 failed (negative
and NaN command age accepted, NaN clearance allowed forward motion, rollback stayed OK, empty/all-NaN
scan reported +∞ clearance and the gate passed 0.4 m/s) — `evidence/astra/f3_before_fix.txt`.

**Root cause.** Scan *arrival* was the only health criterion; `frontClearance` returned +∞ when no
beam was usable; the gate compared `age > timeout` (negative/NaN pass); `TopicMonitor::evaluate`
computed a negative age after a clock rollback; liveness used ROS (sim) time and a ROS timer.

**Fix.** (`velocity_gate.{hpp,cpp}`, `topic_monitor.{hpp,cpp}`, `cmd_vel_gate_node.cpp`,
`health_monitor_node.cpp`, `config/safety.yaml`)
* `assessScan()`: invalid if empty, non-finite/non-positive increment, `range_max ≤ range_min`,
  size inconsistent with the angular span, or < 50 % usable beams overall or in the forward half
  (REP-117: +∞ clear, −∞ obstacle at the sensor, NaN unusable). `frontClearance` returns NaN when
  nothing usable is ahead; NaN clearance blocks forward motion.
* `checkStamp()`: zero/NaN, > 0.2 s in the future, or > 1 s old → scan rejected.
* Gate: negative/non-finite command age → stop; liveness of scans and commands on the **steady
  clock**; loop on a **wall timer** (keeps publishing zero if sim time freezes); backward ROS-clock
  jump → scan state invalidated until a new scan arrives.
* `TopicMonitor`: `CLOCK_JUMP` (→ diagnostics ERROR) when evaluated before its last message.
* Mission: aborts when the gate stays in `STOPPED_SCAN_FAULT` for 2 s — invalid scans can arrive at
  full rate, which the rate-based health monitor would not flag.

**Regression tests.** `test_astra_safety_existing_api.cpp` (5), `test_scan_validation.cpp` (9).

**Verification.** All 32 C++ test cases pass. Gazebo fault injection (`scripts/fault_test.sh`,
evidence `astra/faults/`), robot speed from `/odom`:

| Fault (while driving at 0.40 m/s) | Gate reason | Speed after the fault | Mission |
|---|---|---|---|
| LiDAR bridge killed | `scan timeout` | 0.40 → 0.013 → 0.000 m/s within 1.0–1.5 s | ABORTED "LiDAR stream failed (health_monitor ERROR)" |
| LiDAR replaced by all-NaN scans at 10 Hz | `scan TOO_MANY_INVALID_BEAMS` | 0.40 → 0.14 → 0.00 within ≤ 0.8 s | ABORTED "LiDAR data invalid (cmd_vel_gate STOPPED_SCAN_FAULT)" |
| LiDAR replaced by empty scans at 10 Hz | `scan EMPTY` | 0.40 → 0.22 → 0.00 within ≤ 0.8 s | ABORTED "LiDAR data invalid (cmd_vel_gate STOPPED_SCAN_FAULT)" |

In the two invalid-data cases scans kept arriving at full rate, so the rate-based health monitor
stayed OK; the stop and the abort came from the content validation in the gate.

**Bug found by these tests and fixed.** In the first clean fault runs all three LiDAR cases aborted
with `internal error: ValueError: Logger severity cannot be changed between calls` instead of the
intended reason: `NavGoalRunner` logged INFO and WARNING through one rclpy call site (the fake-client
tests used no logger). The robot was still stopped by the gate. Fixed (two call sites) with a
regression test using a real rclpy logger (`test_nav_goal.py::test_real_rclpy_logger_with_mixed_severities`).

**Remaining risk.** A simulated guard only — not a certified safety function. One 0.1 s
`STOPPED_SCAN_FAULT (scan stamp FUTURE)` was observed at start-up while `/clock` started (robot idle).

## F4 — Nav2 cancellation and goal lifecycle races

**Original finding.** Late goal acceptance after the client timeout, unverified cancellation and
terminal results, goals left active after exceptions.

**Reproduction.** `test_nav_goal_ros.py::test_v1_client_logic_orphans_a_late_goal` replays the v1
client logic against a real rclpy `NavigateToPose` server whose acceptance is delayed: after the
client gave up, the goal was accepted and **kept executing** (orphan).

**Root cause.** The v1 `_nav_attempt` returned `NO_RESPONSE` and forgot the send future; cancel
futures were awaited only for completion (not the return code, not the terminal result);
`KeyboardInterrupt` bypassed cleanup and rclpy's signal handler shut the context down before a
cancel could be sent.

**Fix.** New `src/inspectloop_mission/inspectloop_mission/nav_goal.py` (`NavGoalRunner`): every goal
is tracked until a terminal result is observed; unanswered sends stay tracked and are cancelled on
late acceptance; cancellation counts only with `ERROR_NONE` (listing the goal) / `GOAL_TERMINATED`
**and** a terminal result; any unresolved goal → `motion_inhibited()`, `run()` returns `INHIBITED`
without sending; exceptions / `KeyboardInterrupt` cancel and confirm before propagating; all waits
bounded and in the mission thread. `mission_manager.py`: uses the runner, gives an unresolved goal a
30 s grace, then **latches the gate e-stop** and aborts; at mission end all goals are cancelled and
confirmed (else e-stop); rclpy signal handling disabled so SIGINT can still cancel; launch gives the
mission 40 s to shut down; goal events are stored in the mission record.

**Regression tests.** `test_nav_goal.py` (11 deterministic fake-client cases: success, rejection,
timeout + confirmed cancel, late acceptance, cancel refused, cancel unanswered, cancel accepted but
no terminal result, exception, KeyboardInterrupt, shutdown with pending late acceptance, goal
finishing while cancel refused); `test_nav_goal_ros.py` (5 cases with a real rclpy action server).

**Verification.** All pass. Gazebo SIGINT test (`scripts/interrupt_test.sh`, evidence
`astra/faults/sigint_interrupt/`): SIGINT to the mission manager while Nav2 drives to PG-11 →
`CANCEL_CONFIRMED … cancel ERROR_NONE, terminal CANCELED` (bt_navigator: "Goal canceled"), robot
0.40 → 0.135 → 0.000 m/s within 1.0–1.5 s, report written with `ABORTED: interrupted (SIGINT)`,
launch exit code 0.

**Bug found by this test and fixed.** The first live run had **no effect**: the mission completed
and the robot kept driving. With rclpy's signal handler disabled the mission relied on Python's
default SIGINT handler, which Python does not install when SIGINT is inherited as ignored — as for
background jobs of a non-interactive shell (`SigIgn` mask showed SIGINT/SIGQUIT ignored). The mission
now installs SIGINT/SIGTERM handlers explicitly (`install_interrupt_handlers`); regression test
`test_nav_goal.py::test_interrupt_handlers_installed_even_if_sigint_was_ignored`.

**Remaining risk.** If Nav2 itself dies, cancellation cannot be confirmed; the mission then latches
the gate e-stop — which only helps if the gate is still running.

## F5 — Camera timestamps and observation freshness

**Original finding.** Future-dated images accepted (only `stamp ≥ request` was checked); buffer not
reset across time epochs; results not re-validated by the mission.

**Reproduction.** By code inspection of v1 `_collect_burst` (`s >= not_before_ns` only, no upper
bound, no epoch); unit tests in `test_frame_freshness.py` / `test_observation_guard.py` failed
(modules absent).

**Fix.** `frame_buffer.py` (new): frames eligible only in [request + settle, now + 0.2 s];
clock reset (node clock or plausible stamps jumping back > 1 s) clears the buffer; a single
implausible future stamp cannot trigger a reset; pixel-identical bursts → `FROZEN_FRAMES`.
`gauge_reader_node.py`: buffer invalidated on a new `viewpoint_epoch`; `CLOCK_RESET` during a burst;
capture pose from TF at each frame stamp → `WRONG_VIEWPOINT` / `ROBOT_MOVING`; reply echoes
`observation_id`, epoch, request and first/last frame stamps. Interfaces (`ReadGauge.srv`,
`GaugeReading.msg`) extended accordingly. Mission: epoch per viewpoint, unique observation ids, waits
until odometry shows the robot at rest for 0.5 s, re-validates every reply
(`observation_guard.py`) — mismatches become `STALE_OBSERVATION` without a value; a late reply to
an abandoned request is never consumed.

**Regression tests.** `test_frame_freshness.py` (10), `test_observation_guard.py` (12),
`test_astra_policy.py` (STALE_OBSERVATION / ROBOT_MOVING are transient).

**Verification.** All pass. Live four-gauge run: all 11 observations carried distinct epochs per
viewpoint, robot settled before every request, first frame 0.55–0.8 s after the request, capture
pose known, motion during the bursts 0.0 m. Gazebo injection (evidence `astra/faults/`):

| Fault | Reader | Decisions |
|---|---|---|
| camera bridge killed | `NO_IMAGE` (no stale frame reused) | RETRY_SAME → UNRESOLVED "camera delivers no frames" |
| camera replaced by a frozen stream (one captured frame re-stamped at 10 Hz) | `FROZEN_FRAMES` (+ `TAG_NOT_VISIBLE` computed from the stale content) | RETRY_SAME → UNRESOLVED "FROZEN_FRAMES persisted: moving cannot fix it" |
| camera replaced by frames stamped 5 s in the future | "30 future-dated frame(s) rejected" → `NO_IMAGE` | RETRY_SAME → UNRESOLVED |

**Policy fix found by these tests.** Before, the frozen case escalated to MOVE_NEW because the
derived `TAG_NOT_VISIBLE` looked movable; sensor/data-level invalidation (`NO_IMAGE`,
`FROZEN_FRAMES`, `CLOCK_RESET`, malformed data) now dominates any flag computed from that data
(`test_astra_policy.py::test_sensor_invalidation_dominates_derived_flags`).

**Remaining risk.** TF lookups at frame stamps depend on AMCL/TF timing; with no TF the observation
is rejected (fail-safe, but lowers yield).

## F6 — The reported 95 % uncertainty is not statistically validated

**Original finding.** `value ± 2σ` was labelled "95 %" without calibration evidence.

**Reproduction.** `test_astra_uncertainty_labels.py` failed 4/4 on the v1 report generator
(`evidence/astra/f6_before_fix.txt`).

**Fix.** All "95 %" / CI wording removed from the mission record (`ci95` → `band_2sigma_heuristic`,
`sigma` → `sigma_heuristic`), logs, RViz markers, Markdown/HTML/JSON reports, README and docs; reports
carry an explicit uncertainty note; the synthetic σ test was renamed to a *consistency check*.
New `tools/heldout_uncertainty_eval.py`: 400 samples from a seed and condition ranges not used for
tuning (random values incl. out-of-scale, scale, perspective, in-plane roll, gain, gamma, noise,
blur; distractor / line / white-occluder / shadow subsets), processed by the reader **and** the
mission's decision policy. `evaluate_run` reports denominators and incorrect acceptances.

**Held-out results** (seed 20261108, n = 400; Wilson 95 % intervals describe the observed proportion
on this sample, not σ):

| | v1 code (same samples) | v2 code |
|---|---|---|
| accepted (yield) | 324/400 = 0.81 [0.77, 0.85] | 210/400 = 0.525 [0.48, 0.57] |
| incorrect acceptance (\|err\| > 0.1 bar) | 21/324 = 6.5 % [4.3, 9.7] | **1/210 = 0.5 % [0.08, 2.7]** |
| accepted \|err\| mean / p95 / max | 0.028 / 0.105 / 0.541 bar | 0.015 / 0.044 / 0.118 bar |
| within ±1σ_h / ±2σ_h of accepted | 292/324, 308/324 | 206/210, 210/210 |
| out-of-scale needles accepted | 27/36 | 0/36 |
| clean subset: accepted / incorrect | 218/230 / 11 | 185/230 / 0 |

Raw data: `evidence/astra/heldout_eval.json`, `heldout_eval_v1_baseline.json`.

**Remaining risk.** σ_h remains a heuristic; 210/210 within ±2σ_h is a property of this synthetic
sample, not a coverage guarantee. No real-camera data.

---

## Integration verification (Gazebo Harmonic + Nav2, headless)

Four-gauge missions with the fixed code (each exit code 0, no leftover processes). Final archived
run (`evidence/demo_run_v2/`, after the last code change):

| Gauge | Decisions | Result | Truth | Error |
|---|---|---|---|---|
| PG-11 | ACCEPT | 4.200 bar (σ_h 0.05) | 4.20 | +0.000 |
| PG-12 | MOVE_NEW (frontal occluded) → ACCEPT (−45°, 1.5 m) | 6.794 bar (σ_h 0.08) | 6.80 | −0.006 |
| PG-13 | RETRY_SAME / MOVE_NEW over 3 viewpoints → UNRESOLVED | no value | 3.10 | – |
| PG-14 | ACCEPT, alarm HIGH | 8.702 bar (σ_h 0.05) | 8.70 | +0.002 |

Two earlier full runs with the corrected world gave the same outcomes (max |error| 0.026 and 0.030
bar). In one of them (`evidence/demo_run_v2_tooFar_path/`) the −45° view at 1.5 m was flagged
`TOO_FAR` and PG-12 was accepted after moving closer (1.2 m) — the minor dial radius at that pose
sits close to the 40 px limit, so either path can occur.

Run conditions worth stating: one intermediate fault batch was invalid because a debugging
simulation of mine was still running on the same ROS graph (two `/clock` sources); it was discarded,
`fault_test.sh` now refuses to start next to another InspectLoop simulation, and all fault results
above come from isolated runs. One run (`image_future`, first attempt) never started: the Nav2
container stalled while loading its components; the mission correctly aborted "system not ready"
without moving. Launch exit codes in the fault tests are 1 in some runs because bridges are killed
on purpose and Nav2's container is SIGKILLed at shutdown; the demo runs exit 0.

## Changed files (vs. `v1.0-final`)

- `README.md` (modified)
- `docs/ARCHITECTURE.md` (modified)
- `docs/ASTRA_FIX_REPORT.md` (added, this file)
- `docs/KNOWN_LIMITATIONS.md` (modified)
- `docs/RUN_DEMO.md` (modified)
- `docs/TEST_RESULTS.md` (modified)
- `scripts/fault_test.sh` (modified)
- `scripts/interrupt_test.sh` (added)
- `scripts/publish_fault_stream.py` (added)
- `scripts/sample_odom.py` (modified)
- `src/inspectloop_bringup/launch/inspectloop.launch.py` (modified)
- `src/inspectloop_gazebo/config/warehouse_layout.yaml` (modified)
- `src/inspectloop_gazebo/scripts/generate_warehouse.py` (modified)
- `src/inspectloop_gazebo/worlds/warehouse.sdf` (modified)
- `src/inspectloop_interfaces/msg/GaugeReading.msg` (modified)
- `src/inspectloop_interfaces/srv/ReadGauge.srv` (modified)
- `src/inspectloop_mission/config/mission_params.yaml` (modified)
- `src/inspectloop_mission/inspectloop_mission/decision_policy.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/evaluate.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/mission_manager.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/nav_goal.py` (added)
- `src/inspectloop_mission/inspectloop_mission/observation_guard.py` (added)
- `src/inspectloop_mission/inspectloop_mission/report.py` (modified)
- `src/inspectloop_mission/package.xml` (modified)
- `src/inspectloop_mission/test/test_astra_policy.py` (added)
- `src/inspectloop_mission/test/test_astra_uncertainty_labels.py` (added)
- `src/inspectloop_mission/test/test_nav_goal.py` (added)
- `src/inspectloop_mission/test/test_nav_goal_ros.py` (added)
- `src/inspectloop_mission/test/test_observation_guard.py` (added)
- `src/inspectloop_mission/test/test_report_and_evaluation.py` (modified)
- `src/inspectloop_safety/CMakeLists.txt` (modified)
- `src/inspectloop_safety/config/safety.yaml` (modified)
- `src/inspectloop_safety/include/inspectloop_safety/topic_monitor.hpp` (modified)
- `src/inspectloop_safety/include/inspectloop_safety/velocity_gate.hpp` (modified)
- `src/inspectloop_safety/src/cmd_vel_gate_node.cpp` (modified)
- `src/inspectloop_safety/src/health_monitor_node.cpp` (modified)
- `src/inspectloop_safety/src/topic_monitor.cpp` (modified)
- `src/inspectloop_safety/src/velocity_gate.cpp` (modified)
- `src/inspectloop_safety/test/test_astra_safety_existing_api.cpp` (added)
- `src/inspectloop_safety/test/test_scan_validation.cpp` (added)
- `src/inspectloop_vision/config/gauge_reader.yaml` (modified)
- `src/inspectloop_vision/inspectloop_vision/frame_buffer.py` (added)
- `src/inspectloop_vision/inspectloop_vision/gauge_reader.py` (modified)
- `src/inspectloop_vision/inspectloop_vision/gauge_reader_node.py` (modified)
- `src/inspectloop_vision/test/test_astra_regressions.py` (added)
- `src/inspectloop_vision/test/test_frame_freshness.py` (added)
- `src/inspectloop_vision/test/test_gauge_reader.py` (modified)
- `tools/heldout_uncertainty_eval.py` (added)

Plus new evidence under `docs/evidence/astra/`, `docs/evidence/demo_run_v2*/`; v1 evidence moved to `docs/evidence/v1_demo_run/`.
