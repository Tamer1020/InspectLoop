# InspectLoop V3 — Remediation of the Independent V2 Review

Input: *InspectLoop_V2_Independent_Review_AR.pdf* (verdict **REJECT V2**: two HIGH defects, F3 and
F4; MEDIUM findings F1-residual, F5, F7, F8, F9). Baseline: git `f38d7dc` (=
`InspectLoop_Portfolio_Reviewed_v2.zip`, kept unchanged, as is the V1 ZIP).
All numbers in this file were produced on 2026-11-08/09 on the development machine (WSL2,
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 1.3, 16 cores).

**Evidence levels.**
* *Independently reproducible without ROS*: the C++ gate core (GoogleTest), the Python policy /
  guard / nav-goal / report / reader tests, and the synthetic CV evaluation
  (`tools/heldout_uncertainty_eval.py`, fixed seeds). The reviewer ran this class of tests.
* *Developer-reported*: everything that needs ROS 2 / Gazebo (the rclpy action-server tests, the
  four-gauge demo, fault injection). The reviewer did not run these; logs, mission records,
  images and the exact commands are archived under `docs/evidence/` so they can be re-run.

For every finding a regression test was written first and run against the V2 code; the failing
output is in `docs/evidence/astra_v3/before_fix_*.txt` (V2 code taken from git `f38d7dc`, run in an
isolated copy where shadowing by the new code was possible).

## Summary

| ID | Severity | Finding | V3 status |
|---|---|---|---|
| F3 | HIGH | Forward-sector LiDAR blindness passed the motion gate | **Fixed** — per-direction sector coverage; reproduced Astra case now `BLOCKED_UNOBSERVED`, (0, 0); Gazebo injection stops the robot |
| F4 | HIGH | Goal ownership lost between send and acceptance | **Fixed** — ownership from `send_goal_async`; exception-safe lifecycle; cleanup cannot report success while unresolved; e-stop fallback |
| F5 | MEDIUM | 23 s-old result, 0.8 s rollback, stale odometry admissible | **Fixed** — result-age limit at consumption, stamp ordering, clock epochs via jump callbacks (no size threshold), stamp-based fresh odometry |
| F1 | MEDIUM | Merged-distractor false acceptance (case #183) | **Fixed for the reviewed mechanism** (outer-lobe width bound); narrower merges remain a documented limitation; 1 clean-view error 0.101 bar on Astra's set unchanged |
| F7 | MEDIUM | Reader exceptions reported as `NO_IMAGE` | **Fixed** — typed `fault_class` per reading, `outcome_class` per gauge |
| F8 | MEDIUM | Viewpoint "feasible" overstated reachability | **Documented + wording fixed** — footprint-clear only; failure discovered by navigating |
| F9 | MEDIUM | `report._rel()` broke links after moving a run; `assert … or True` | **Fixed** — tests fail on broken links; live relocation check passes |
| — | — | Found in V3 verification: GIL contention made the reader 8–12 s slow in the node | **Fixed** — reader runs in a worker process; TF poses captured before processing |
| — | — | Found in V3 verification: fault harness printed the wrong exit code | **Fixed**; V2 statement corrected (see Errata) |

---

## F3 (HIGH) — LiDAR scan blindness and motion safety

**Finding.** Astra: 720 beams, valid 5 m returns outside ±35°, NaN inside ±35°, command
(0.4 m/s, 0.2 rad/s) → `VALID`, clearance `inf`, `PASS`, output (0.4, 0.2).

**Root cause.** `assessScan()` required ≥ 50 % usable beams in the whole forward *half* (±90°);
the blind ±35° wedge leaves 61.2 % (`forward_usable = 0.612188`, reproduced exactly).
`frontClearance()` then simply skipped NaN beams, so an unobserved corridor produced "no obstacle
found" = `+inf`. Absence of evidence was treated as free space. Reverse motion (Nav2 MPPI
`vx_min = -0.2`) was not checked against anything behind the robot, and rotation not at all.

**Fix** (`velocity_gate.{hpp,cpp}`, new `gate_supervisor.{hpp,cpp}`, `cmd_vel_gate_node.cpp`,
`safety.yaml`):
* `sectorCoverage()` samples a sector at the scan's angular increment (wrap-around for 360°
  scans) and reports the observed fraction and the widest unobserved wedge. A beam observes its
  ray only if finite in [range_min, range_max] or ±inf (REP-117); NaN, out-of-range values and
  directions the scanner does not sample at all are unknown.
* Motion-relevant sectors from the robot geometry (overridable): forward corridor
  ±atan2(0.27, 0.15) = ±60.9° (rays outside it cannot enter the corridor ahead of the front
  edge), rear corridor ±atan2(0.27, 0.35) = ±37.6°, and the full circle for in-place rotation.
  Observed ⇔ coverage ≥ 0.95, widest gap ≤ 2°, angular increment ≤ 1° (all parameters).
* `applyGate()` gates each motion component on its own evidence: forward needs the forward
  sector, reverse the rear sector plus a rear corridor clearance (`rearClearance()`), rotation the
  full circle and no obstacle inside the 0.35 m turning radius. An unobserved direction zeroes only
  that component (`BLOCKED_UNOBSERVED`). The new `GateInputs` coverage fields default to
  *not observed* (fail closed).
* All node logic (steady-clock liveness, stamp plausibility, backward ROS-clock jumps of any size,
  command watchdog) moved into the ROS-independent `GateSupervisor`, so it is unit-tested exactly
  as it runs. The independent command watchdog (0.5 s, steady clock) is unchanged.
* Mission: `BLOCKED_UNOBSERVED` for 3 s → `MissionAborted` ("LiDAR does not observe the motion
  corridor").

**Regression tests.** `test_v3_sector_blindness_existing_api.cpp` (Astra's exact scenario through
the V2 API; **failed on V2** with `forward_usable=0.612188 validity=VALID clearance=inf state=PASS
out=(0.40, 0.20)`, now `BLOCKED_UNOBSERVED out=(0.00, 0.00)`) and `test_v3_motion_coverage.cpp`
(26 tests): Astra scenario through the pure gate and through the node logic; all-NaN; empty; valid
clear (inf and finite); obstacle in the stopping corridor; 0.8 s and 10 ms clock rollback (also
scans stamped on the old timeline); stale header stamp and steady-clock scan timeout; future
stamp; negative and NaN command age; watchdog preserved; no scan ever; tolerated 1.5° gap vs.
rejected 3° gap; narrow blind wedge at the corridor edge vs. outside it; 12 % scattered invalid
beams; out-of-range finite values; rear blindness (reverse blocked, forward allowed); obstacle
behind; obstacle inside the turning circle; 360/720/1080/1440 beams; 4° resolution; 270° and ±30°
scanners; wrap-around gap across ±180°; sector geometry. The two V2 test fixtures that built
`GateInputs` by hand now state full coverage explicitly (their obstacle/limit semantics unchanged).

**Verification.** Unit tests: see *Test totals*. Gazebo: see *Live integration* (`scan_front_nan`,
`scan_nan`, `scan_empty`, `lidar`). In the four-gauge demo the real Gazebo scans never triggered
`BLOCKED_UNOBSERVED` (gate states: PASS / SLOWED / STOPPED_NO_COMMAND only).

**Remaining risk.** The gate checks the 2-D scan plane only (overhanging objects are invisible, as
to Nav2); wedges ≤ 2° are tolerated; thresholds are hand-set, not derived from a stopping-distance
analysis; this is a simulated guard, **not a certified safety function**.

## F4 (HIGH) — Nav2 goal ownership and the cancellation race

**Finding.** Astra: the goal was sent at `nav_goal.py:159`, but its future was recorded only after
the acceptance wait timed out, and the protective `try` began only after the tracked goal existed.
An interrupt during the acceptance wait produced `cancel_all("shutdown") = True`,
`unresolved() = []`, `motion_inhibited() = False`, `cancel_calls = 0`, goal not terminal. In
addition `mission_manager.py:690` only logged a cleanup exception (no e-stop fallback), and the
existing KeyboardInterrupt test covered only the post-acceptance window.

**Root cause.** Ownership was tied to *acceptance*, not to *sending*; the send → acceptance window
was outside every exception handler; `cancel_all()` could raise (e.g. a second Ctrl-C) and the
mission swallowed that without inhibiting motion.

**Fix** (`nav_goal.py`, `mission_manager.py`):
* The send future is registered as a pending send immediately when `send_goal_async` returns;
  the whole send → acceptance → result lifecycle (`_run_owned`) runs inside a `BaseException`
  handler that resolves everything owned (bounded waits) before re-raising.
* A late acceptance is tracked, cancelled and confirmed (cancel reply listing the goal **and** a
  terminal result) and now leaves a `TERMINAL` audit event even when it resolves during polling.
* `send_goal_async` raising → ownership unknown → `INHIBITED` permanently for this runner.
* Cleanup (`_confirm_cancel`, `cancel_all`) never raises (also not on `KeyboardInterrupt`) and
  returns True only if nothing is unresolved; failures leave the goal tracked and inhibiting
  (`CLEANUP_EXCEPTION`, `CLEANUP_INCOMPLETE` events). No new goal is sent while anything is
  unresolved.
* Mission end: if cleanup is not confirmed — or raises anyway — the gate e-stop is latched (and a
  failure to latch is recorded as "NOT confirmed - check the robot"); the record carries
  `nav_cleanup_confirmed`, `unresolved_goals`, `cleanup_exception`.

**Regression tests.** `test_v3_nav_goal_ownership.py` (11): interrupt (KeyboardInterrupt and
RuntimeError) inside the acceptance wait with acceptance after / within the cleanup window; never
answered; late goal whose cancel is rejected; no second goal while unresolved; exception thrown
by the cleanup itself; second Ctrl-C during cleanup; `send_goal_async` raising; normal timeout /
rejection / success paths. **On V2: 8 failed**, and the "second Ctrl-C" test's KeyboardInterrupt
escaped `run()` and terminated the pytest session itself. `test_nav_goal_ros.py` gained a test
against a real rclpy NavigateToPose server: Ctrl-C during acceptance, the server accepts 1.5 s
later, the goal is cancelled on the server, confirmed `CANCELED`, and a new goal is refused in
between (`INHIBITED`).

**Verification.** Unit + rclpy tests (see *Test totals*); Gazebo SIGINT test (see *Live
integration*).

**Remaining risk.** "Ownership unknown" cannot be cleared (no cancel-all through the action's
cancel service) → the mission ends with the e-stop latched; bounded cleanup can take up to ~40 s
when Nav2 has died (accept 15 s + cancel 10 s + terminal 15 s).

## F5 (MEDIUM) — camera freshness, epochs and odometry

**Finding.** A 23 s-old result passed `validate_observation()`; a 0.8 s sim-clock rollback left a
frame of the previous epoch selectable (`FrameBuffer.check_clock`, 1 s threshold); no maximum
result age, no `first ≤ last` check; `wait_settled()` could use stale odometry.

**Root cause.** The guard checked relations to the *request* only, never the age at
*consumption*; clock discontinuities were detected by comparing against a 1 s threshold; the odom
samples were stamped with the receive time and their freshness was never checked.

**Fix.**
* `observation_guard.py`: `max_result_age_s` (8 s sim time; measured ages in the V3 demo
  0.2–0.8 s), `request ≤ first ≤ last ≤ now + 0.2 s`, clock epochs at request and consumption,
  mission identity, value-without-frames and value-without-pose rejection. `is_settled()` takes
  `now_s`: newest sample ≤ 0.3 s old, not future-dated, ≥ 3 samples in the window.
* Clock discontinuities: the mission and the reader register an rclpy clock-jump callback
  (`min_backward = -1 ns`); *every* backward step increments the clock epoch / clears the frame
  buffer and marks a burst in progress (`CLOCK_RESET`, also if the jump happens during
  processing). The frame buffer's remaining tolerance is 1 ms (float rounding), replacing the 1 s
  threshold. The gate treats any backward step > 1 ms the same way (F3).
* Odometry samples now carry the message stamp (acquisition time).
* The record stores request time, first/last frame stamps, consumption time, result age, clock
  epochs at request and consumption, mission id, and every clock discontinuity.

**Regression tests.** `test_v3_freshness.py` (11; **8 failed on V2**, incl. the exact 23 s and
0.8 s cases) and `test_v3_clock_rollback.py` (4; **3 failed on V2**, incl. the 0.8 s rollback).

**Verification.** Gazebo `clock_reset` injection (world time reset, −26.7 s): mission, gate and
reader all detected it; no value spanning it was used (see *Live integration*).

**Remaining risk.** A rollback that happens and is undone between two `/clock` messages cannot be
seen by anyone; Nav2/AMCL themselves do not recover from a world-time reset.

## F1 (MEDIUM) — remaining rejection/acceptance trade-off

**Finding.** Case #183 of the seed-20261108 evaluation: truth 2.07644, accepted 1.95804, error
0.11840 bar, no flag.

**Protocol (no tuning on the test set).** Before any reader change, a final test set was frozen
and committed (`docs/evidence/astra_v3/f1_frozen_test_set.json`, git `4785f10`: seed 20261201,
n = 400, tool SHA-256 recorded). Investigation and the design check used only the development set
(seed 20261108, already used for V2) and the reproduced case. Astra's cross-check set (seed
20261109, n = 200) was used once, as a check. The frozen set was evaluated once, for V2 (exported
from `f38d7dc`) and V3, after the change was final. Tolerance 0.10 bar.

**Reproduction and mechanism.** The eval tool now records the generator draws (`--dump-case`):
a radial line 5.92° from the needle, darkness 24/255, 3 px wide in the texture (≈ 1 px in the
image), 0.18–0.66 R, blur 5 px, minor dial radius 53.3 px. All 16 sampling rings (0.15–0.52 R)
lie inside the line's radial span, so on every ring needle and line merge into **one** lobe:
ambiguity 0.01, support 1.00, model residual 0.037–0.070 (< 0.075 — the residual fit absorbs the
extra width by choosing a larger blur). Per-frame angles scatter between the two lines
(−79.7° … −83.4°, needle −78.9°, line −84.9°). On the outer rings, where the needle is narrow, the
merged lobe is wider than any single needle can be: outer lobe ratio 1.03–1.10.

**Fix.** `gauge_reader.py`: median outer-ring (≥ 0.40 R) lobe width / predicted needle width
> 1.0 → `AMBIGUOUS_NEEDLE`. 1.0 is the model bound (the prediction already contains a 1.5 px blur
allowance), not a fitted value; on the development set correct accepted readings reached 0.90.
Other candidate features (radial angle trend, frame spread) overlapped with correct readings and
were not used.

**Regression tests.** `test_v3_merged_distractor.py` (110): case #183 from archived frames
(**failed on V2**: `1.96 +/- 0.07 bar from 5/5 frames`, no flag); 96 parametric merged-line cases
(thin dark line at ±4…8°, blur 3/5, scales 0.30–0.50, three values) must never be silently wrong;
12 clean cases under the same conditions must still be accepted (≤ 0.03 bar). The original F1
tests (`test_astra_regressions.py`) are unchanged and pass.

**Results** (accepted / samples, incorrect = accepted with |err| > 0.10 bar):

| Set | Version | Overall | Clean in-scale | Difficult in-scale | Out-of-scale truth | Incorrect | Max \|err\| |
|---|---|---|---|---|---|---|---|
| Development (20261108, n=400) | V2 | 210/400 | 185/208 | 25/156 | 0/36 | 1 (#183, 0.118) | 0.118 |
| Development (20261108, n=400) | V3 | 209/400 | 185/208 | 24/156 | 0/36 | **0** | 0.084 |
| Astra cross-check (20261109, n=200) | V2 | 97/200 | 89/95 | 8/90 | 0/15 | 1 (#94 clean, 0.1013) | 0.101 |
| Astra cross-check (20261109, n=200) | V3 | 97/200 | 89/95 | 8/90 | 0/15 | 1 (#94 clean, 0.1013) | 0.101 |
| **Frozen final (20261201, n=400)** | V2 | 208/400 | 189/204 | 19/158 | 0/38 | 0 | 0.071 |
| **Frozen final (20261201, n=400)** | V3 | 208/400 | 189/204 | 19/158 | 0/38 | **0** | 0.071 |

Our V2 numbers on seed 20261109 equal Astra's (97 accepted / 1 incorrect; clean 89/95,
difficult 8/90, out-of-scale 0/15), i.e. the same tool and generator. Astra's V1 numbers on that
set (157/200, 9 incorrect; difficult 52/90 with 3 incorrect; 11/15 out-of-scale accepted) are
*reviewer-reported* and were not re-run here.

**Honest reading of the trade-off.** The bound removes the reviewed failure mechanism at a cost of
exactly that one acceptance on the development set and no decision change on the frozen set. It
does not change the large yield loss on difficult views that V2 introduced (clean ≈ 90 %
accepted, difficult ≈ 9–15 %). Astra's remaining incorrect acceptance (#94) is a *clean* dial
under strong perspective (skew −0.24), small dial (46.6 px) and 5 px blur, error 0.1013 bar with
σ_h 0.118: a resolution/bias limit, not a distractor; it was not tuned on. Merges narrower than the
bound (very close, thin lines) remain possible — in the parametric test they stayed within 0.1 bar,
which is not a guarantee. Evidence: `docs/evidence/astra_v3/f1_eval/`.

## F7 (MEDIUM) — system faults vs. image-quality failures

**Root cause.** `GaugeReaderNode._on_read()` mapped every internal exception to `NO_IMAGE`; the
mission mapped service unavailability / timeouts to `NO_IMAGE` too, so software and service faults
looked like camera or visibility problems.

**Fix.** `GaugeReading.fault_class` (`NONE`, `CAMERA_UNAVAILABLE`, `IMAGE_INVALID`,
`PROCESSING_ERROR`, `INSUFFICIENT_EVIDENCE`; mission-side `SERVICE_FAILURE`). New flags
`PROCESSING_ERROR` (reader exception), `SERVICE_FAILURE` (service unavailable / timeout / no
response / client exception), `INVALID_FRAMES` (frames arrived but none admissible — previously
`NO_IMAGE`). The policy retries these once and ends UNRESOLVED with a reason stating "system fault
… not an image-quality problem" / "invalid image data"; they never trigger MOVE_NEW. Each gauge
result has `outcome_class` (`MEASURED`, `INSUFFICIENT_VISUAL_EVIDENCE`, `CAMERA_UNAVAILABLE`,
`IMAGE_INVALID`, `SYSTEM_FAULT`, `NAVIGATION_FAILURE`), shown in the report.

**Tests.** `test_v3_fault_class.py` (14, incl. a functional test of the exception path) and
`test_v3_typed_outcomes.py` (15). **Live:** demo PG-13 → `INSUFFICIENT_VISUAL_EVIDENCE`; camera
loss → `CAMERA_UNAVAILABLE`; clock reset → `IMAGE_INVALID` observations, `NAVIGATION_FAILURE`
gauge (see below).

## F8 (MEDIUM) — viewpoint feasibility claims

Chosen option: **document precisely** (a bounded Nav2 `ComputePathToPose` pre-check would add a
planner dependency and latency for a failure the loop already handles). `viewpoint_planner.py`,
the mission record (`candidate_feasibility`), the report ("Footprint-clear viewpoint candidates
(Nav2 path reachability not pre-checked)"), the UNRESOLVED reason and the docs now say
*footprint-clear in the occupancy map*; unreachable candidates are found by navigating
(`NAV_FAILED` → next candidate, or `NAVIGATION_FAILURE`).

## F9 (MEDIUM) — report portability and an ineffective test

**Root cause.** `_rel()` called `os.path.relpath(path, run_dir)` unconditionally: an absolute path
from the *original* run directory became `../../../orig/...` after a move, and an already-relative
path was resolved against the current working directory. `test_report_and_evaluation.py:54`
contained `assert 'images/o2.png' not in md or True`, which can never fail.

**Fix.** `_rel(path, run_dir, original_run_dir)`: relative paths are kept (relative to the run
directory); absolute paths under the original or current run directory are re-based; otherwise an
`images/<basename>` that exists is used. The ineffective assertion was replaced by real checks
(`(images/o2.png)` in the Markdown, `src='images/o2.png'` in the HTML, no `/run/images` left).

**Tests.** `test_v3_report_portability.py` (4; **3 failed on V2**). **Live:** the V3 demo run
directory was copied to a deeper, different path: all 10 image links of the copied *and* of a
regenerated report resolve (`docs/evidence/astra_v3/f9_relocation_check.txt`).

## Issue found during live verification: reader 8–12 s slow inside the node

The first V3 demo run (`docs/evidence/astra_v3/demo_run_v3_first_attempt_gil_regression/`) left
PG-11 and PG-14 UNRESOLVED with `STALE_OBSERVATION`: results were 8.4–13.5 s old (new F5 limit 8 s)
and, beyond ~10 s, the TF poses at the frame stamps had left the TF buffer (`WRONG_VIEWPOINT`,
"pose at acquisition time unknown"). The readings themselves were correct (4.18–4.19 bar).
Measurements: `read_frame` takes 16–17 ms per frame alone (V2 and V3 identical) and 35 ms under the
running simulation, but `processing_time_ms` in the node was 8–12 s (V2 demo: 1.7–9.2 s, V1:
0.7–1.2 s). Any rclpy node with `use_sim_time` spends ~0.5 core on Gazebo's ~1 kHz `/clock`
(measured with a bare node, independent of the new jump callback); the reader's many OpenCV/NumPy
calls each release and re-acquire the GIL and wait behind the callback threads. A harness under the
live simulation: default 114–534 ms per burst, 0.5 ms switch interval 83–92 ms, worker process
90–96 ms.
**Fix:** frames are read in a dedicated `spawn` worker process (`reader_worker.py`, no rclpy
import; worker restarted and `PROCESSING_ERROR` reported on failure), the node uses a 1 ms GIL
switch interval, and TF poses are looked up *before* processing. Result: 70–385 ms per burst,
result ages 0.18–0.76 s in the archived demo. Test: `test_v3_reader_worker.py`.

## Live integration (Gazebo Harmonic + Nav2, headless, one simulation at a time)

Developer-reported; commands `scripts/fault_test.sh <mode> <dir>` and
`scripts/interrupt_test.sh <dir>`; records, logs and speed traces in
`docs/evidence/astra_v3/faults/<mode>/`. Every run: launch exit code 0 (now the real code), no
processes left (`cleanup_sim.sh` between runs, the harness refuses to start next to another
simulation).

**Four-gauge demo** (`docs/evidence/demo_run_v3/`, repeat `docs/evidence/demo_run_v3_repeat/`):

| Gauge | Decisions | Run 1 | Run 2 | Truth |
|---|---|---|---|---|
| PG-11 | ACCEPT | 4.215 | 4.192 | 4.20 |
| PG-12 | frontal occluded → MOVE_NEW → ACCEPT at −45°, 1.5 m | 6.837 | 6.787 | 6.80 |
| PG-13 | RETRY_SAME / MOVE_NEW over 3 viewpoints → UNRESOLVED (`INSUFFICIENT_VISUAL_EVIDENCE`) | – | – | 3.10 |
| PG-14 | ACCEPT, alarm HIGH | 8.679 | 8.710 | 8.70 |

Max |error| 0.037 / 0.014 bar, 0 incorrect acceptances, mission COMPLETED, goal cleanup
confirmed, no `BLOCKED_UNOBSERVED` on real scans, result ages 0.18–0.76 s. (A first run before
the worker-process fix failed — see above.)

**Fault injection:**

| Mode | What is injected | Observed |
|---|---|---|
| `scan_front_nan` | live scan relayed with all beams in ±35° NaN (rest valid, rate healthy) | gate `BLOCKED_UNOBSERVED (forward sector unobserved, full circle unobserved)`, fwd coverage 0.42, gap 70.1°; 0.40 → 0 m/s within ≤ 1.2 s of the real bridge being killed; mission ABORTED after 3.1 s "LiDAR does not observe the motion corridor" |
| `lidar` | LiDAR bridge killed while driving | gate `scan timeout`; 0.40 → 0.097 m/s in 1.0–1.5 s, 0 from 1.5 s; ABORTED (health ERROR); cleanup confirmed |
| `scan_nan` | all-NaN scans at 10 Hz | gate `TOO_MANY_INVALID_BEAMS`; stop 1.0–1.5 s after the kill; ABORTED. *Run 1 invalid for its purpose:* the injector's sim-time timer stalled, so the gate saw a scan timeout instead (robot still stopped); the injector now publishes on a wall timer and logs its count, run 2 is the valid one (run 1 kept in `scan_nan_run1_injector_stalled/`) |
| `scan_empty` | empty scans at 10 Hz | gate `scan EMPTY`; stop within ≈ 1.0 s; ABORTED |
| `camera` | camera bridge killed | `NO_IMAGE` → RETRY_SAME → UNRESOLVED, outcome `CAMERA_UNAVAILABLE` |
| `image_frozen` | one frame re-published with new stamps | `FROZEN_FRAMES` → RETRY_SAME → UNRESOLVED, outcome `IMAGE_INVALID` (never MOVE_NEW) |
| `image_future` | frames stamped 5 s ahead | 30 future frames rejected → `INVALID_FRAMES` (V2: `NO_IMAGE`) → UNRESOLVED, `IMAGE_INVALID` |
| `clock_reset` | Gazebo world-time reset at the first viewpoint (−26.747 s) | mission clock epoch 0 → 1, gate "clock jumped backwards" → `STOPPED_SCAN_FAULT`, reader buffer cleared; obs #1 `INVALID_FRAMES, CLOCK_RESET, STALE_OBSERVATION` ("clock discontinuity between request and consumption"), obs #2 rejected (pose unknown, not settled); Nav2 then aborted every goal (upstream, TF/AMCL do not recover) → PG-11 UNRESOLVED `NAVIGATION_FAILURE`; no value accepted |
| SIGINT | Ctrl-C to the mission while driving | goal cancel `ERROR_NONE`, terminal `CANCELED` confirmed; 0.33 → 0.045 m/s within 1 s (sampling ended with the launch, full stop not recorded); report ABORTED "interrupted (SIGINT)". The first goal of this run was `REJECTED` because `bt_navigator` was still inactive at mission start (start-up race, retried and accepted 2 s later) |


## Test totals

`colcon build`: 8 packages, no warnings. `colcon test` (`ROS_DOMAIN_ID=87`):
**460 results, 0 errors, 0 failures, 0 skipped** (`docs/evidence/astra_v3/colcon_test_result.txt`,
per-file counts in `test_breakdown.txt`).

| Package | Test cases | New in V3 |
|---|---|---|
| inspectloop_safety (C++ GoogleTest) | 59 | `test_v3_motion_coverage` (26), `test_v3_sector_blindness_existing_api` (1) |
| inspectloop_vision (pytest) | 208 | `test_v3_merged_distractor` (110), `test_v3_fault_class` (14), `test_v3_clock_rollback` (4), `test_v3_reader_worker` (1) |
| inspectloop_mission (pytest) | 157 | `test_v3_typed_outcomes` (15), `test_v3_nav_goal_ownership` (11), `test_v3_freshness` (11), `test_v3_report_portability` (4), +1 in `test_nav_goal_ros` (real action server) |
| bringup / description / gazebo / navigation | 6 / 7 / 7 / 6 | – |

450 test cases + 10 CTest wrapper entries = 460. Test files vs. V2: **10 added**, **4 modified**
(`test_nav_goal_ros.py`, `test_report_and_evaluation.py`, `test_astra_safety_existing_api.cpp`,
`test_velocity_gate.cpp`), plus 6 fixture files (`test/data/astra_v3_case183/`).


## PG-12 world occluder: V1 → V2 (unchanged in V3)

V1 modelled the PG-12 conduit box with a horizontal bracket from the box straight back to rack S2.
That arm passed **through the gauge face** (physically impossible) and appeared as a dark bar
across the dial from every viewpoint. V1's reader accepted views across it by chance; V2's
stricter needle-evidence checks correctly refused them, which made PG-12 UNRESOLVED. V2 therefore
changed the world (`warehouse_layout.yaml`, `generate_warehouse.py`, regenerated
`warehouse.sdf`): the box (same size 0.16 × 0.06 × 0.16 m, same position 0.30 m in front of the
dial, same z) now hangs from a thin rod 0.095 m beside the panel up to an arm at 1.9 m. The box
still hides the dial from the frontal viewpoint; the map is unchanged. **Consequence:** the
PG-12 MOVE_NEW → ACCEPT result of V2/V3 is obtained in a corrected world and is not directly
comparable with V1's. Images: `docs/evidence/astra/pg12_bracket/`.

## Errata to the V2 deliverable

* **Test file counts.** The V2 hand-over message stated "13 new test files plus 3 updated". Correct
  (git `v1.0-final..f38d7dc`): **9 added, 2 modified** test files, as the reviewer found. The file
  list in `ASTRA_FIX_REPORT.md` itself was correct.
* **Fault-test exit codes.** `ASTRA_FIX_REPORT.md` said launch exit codes were 1 in some fault runs
  "because bridges are killed on purpose and Nav2's container is SIGKILLed". Wrong: the harness
  printed `$?` of the preceding `[[ -n "$inj" ]] && kill …` line (1 whenever no injector ran),
  not the launch's exit code. Fixed in V3 (`rc=$?` right after `wait`); V3 fault runs report the
  real code.

## Changed files (vs. V2, git `f38d7dc`)

- `README.md` (modified)
- `docs/ARCHITECTURE.md` (modified)
- `docs/ASTRA_FIX_REPORT.md` (modified)
- `docs/ASTRA_V3_REMEDIATION_REPORT.md` (added)
- `docs/KNOWN_LIMITATIONS.md` (modified)
- `docs/RUN_DEMO.md` (modified)
- `docs/TEST_RESULTS.md` (modified)
- `docs/TROUBLESHOOTING.md` (modified)
- `scripts/fault_test.sh` (modified)
- `scripts/interrupt_test.sh` (modified)
- `scripts/publish_fault_stream.py` (modified)
- `src/inspectloop_interfaces/msg/GaugeReading.msg` (modified)
- `src/inspectloop_mission/config/mission_params.yaml` (modified)
- `src/inspectloop_mission/inspectloop_mission/decision_policy.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/mission_manager.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/nav_goal.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/observation_guard.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/report.py` (modified)
- `src/inspectloop_mission/inspectloop_mission/viewpoint_planner.py` (modified)
- `src/inspectloop_mission/test/test_nav_goal_ros.py` (modified)
- `src/inspectloop_mission/test/test_report_and_evaluation.py` (modified)
- `src/inspectloop_mission/test/test_v3_freshness.py` (added)
- `src/inspectloop_mission/test/test_v3_nav_goal_ownership.py` (added)
- `src/inspectloop_mission/test/test_v3_report_portability.py` (added)
- `src/inspectloop_mission/test/test_v3_typed_outcomes.py` (added)
- `src/inspectloop_safety/CMakeLists.txt` (modified)
- `src/inspectloop_safety/config/safety.yaml` (modified)
- `src/inspectloop_safety/include/inspectloop_safety/gate_supervisor.hpp` (added)
- `src/inspectloop_safety/include/inspectloop_safety/velocity_gate.hpp` (modified)
- `src/inspectloop_safety/src/cmd_vel_gate_node.cpp` (modified)
- `src/inspectloop_safety/src/gate_supervisor.cpp` (added)
- `src/inspectloop_safety/src/velocity_gate.cpp` (modified)
- `src/inspectloop_safety/test/test_astra_safety_existing_api.cpp` (modified)
- `src/inspectloop_safety/test/test_v3_motion_coverage.cpp` (added)
- `src/inspectloop_safety/test/test_v3_sector_blindness_existing_api.cpp` (added)
- `src/inspectloop_safety/test/test_velocity_gate.cpp` (modified)
- `src/inspectloop_vision/config/gauge_reader.yaml` (modified)
- `src/inspectloop_vision/inspectloop_vision/frame_buffer.py` (modified)
- `src/inspectloop_vision/inspectloop_vision/gauge_reader.py` (modified)
- `src/inspectloop_vision/inspectloop_vision/gauge_reader_node.py` (modified)
- `src/inspectloop_vision/inspectloop_vision/reader_worker.py` (added)
- `src/inspectloop_vision/test/test_v3_clock_rollback.py` (added)
- `src/inspectloop_vision/test/test_v3_fault_class.py` (added)
- `src/inspectloop_vision/test/test_v3_merged_distractor.py` (added)
- `src/inspectloop_vision/test/test_v3_reader_worker.py` (added)
- `tools/heldout_uncertainty_eval.py` (modified)
- `src/inspectloop_vision/test/data/astra_v3_case183/` (added: 5 frames + parameters)

Plus evidence: `docs/evidence/astra_v3/`, `docs/evidence/demo_run_v3/`, `docs/evidence/demo_run_v3_repeat/`.
