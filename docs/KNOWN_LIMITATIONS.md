# Known limitations

## Perception
* **Fiducial-assisted reading.** The dial is located through an ArUco tag printed on the same panel.
  Real plants rarely have tags next to every gauge; a production system would need a learned dial
  detector or a registered CAD/asset model. Without the tag the reader reports `TAG_NOT_VISIBLE`.
* **One gauge design.** Scale (0–10 bar, 270° sweep), panel layout and the needle-darkness model are
  fixed constants. Other dials (different sweeps, white needles, multiple pointers, digital displays)
  are not supported.
* **Simulated imagery.** The camera is a Gazebo render with mild Gaussian noise: no glare, no glass
  reflections, no motion blur, no lens distortion, stable lighting. The measured accuracy
  (≤ 0.037 bar in the simulation runs) will not transfer to real cameras without re-calibration.
* **σ is a heuristic indicator** (frame spread + pixel-resolution term + 0.03 bar floor), used only
  as an acceptance threshold. It is not a calibrated probability and value ± k·σ is not a confidence
  interval. On the held-out synthetic set every accepted reading was within ±2σ_h (210/210) — a
  property of that sample, not a coverage guarantee — and σ_h does not model systematic errors that
  are identical in all frames of a burst.
* **Nearby lines below the resolution limit (Astra F1 residual risk).** A dark radial line within
  a few degrees of the needle can merge with it into one lobe. V3 adds a physical bound (on rings
  ≥ 0.40 R the merged lobe is wider than any single needle) that rejects the reviewed case
  (#183 of seed 20261108). Merges narrower than that bound are still possible: in the parametric
  test (thin dark line at ±4–5°) several cases pass unflagged — within 0.1 bar in every tested
  case, but nothing guarantees that for other line darkness/width/blur combinations.
* **Clean-view bias under strong perspective + blur.** Astra's cross-check set (seed 20261109)
  contains one incorrect acceptance of a *clean* dial (#94: skew −0.24, 46.6 px minor radius,
  blur 5 px; error +0.1013 bar with σ_h 0.118, i.e. inside ±1σ_h but beyond the 0.10 bar
  tolerance). V3 does not change it; it was not tuned on that set.
* **Conservatism costs yield, mostly on difficult views.** On the frozen V3 test set (seed
  20261201, never used for tuning) V2 and V3 both accept 189/204 clean but only 19/158 difficult
  in-scale samples (distractors, occluders, shadows), with 0 incorrect acceptances; Astra measured
  8/90 difficult for V2 (V1: 52/90 with 3 incorrect). Partial occlusions, shadows and nearby lines
  are refused even when a human could still read the gauge.
* **Thresholds are hand-set** (visibility 0.85, needle confidence 0.35, σ_h ≤ 0.15 bar, ambiguity
  0.45, model residual 0.075, minor dial radius ≥ 40 px, …). The accepted PG-12 view (−45°, 1.2 m)
  has full visibility and needle confidence 0.98 but a view angle of 48° (limit 55°) and a minor
  dial radius that failed the 40 px limit at 1.5 m and passed at 1.2 m — a different pose
  could need another viewpoint.
* Partial occlusion that leaves the needle visible but covers part of the face lowers visibility and
  is treated as unreliable even if the needle could be read — a deliberate conservative choice.

## Planning / decisions
* **Viewpoint feasibility = footprint-clear only (Astra F8).** No Nav2 path is planned to a
  candidate beforehand; an enclosed or inflation-blocked candidate is only found unreachable by
  trying (NAV_FAILED → next candidate). This costs time but is reported, never hidden.
* **Visibility is not predicted.** The viewpoint planner checks reachability on the 2-D map only; it
  learns about occlusion by observing. It therefore spends observations on views that a 3-D model
  could have ruled out (PG-13 costs 3 viewpoints / 6 observations / ~60 s before UNRESOLVED).
* Fixed candidate set (3 distances × 5 angles) and a greedy, hint-based selection; no information-gain
  planning, no camera pan/tilt.
* RETRY_SAME only waits and re-captures; it does not re-align the robot or adjust exposure.

## Navigation / simulation
* The map is generated from the world description (exact) rather than built with SLAM; AMCL starts
  from a known initial pose. `slam_toolbox` is installed but SLAM mapping was not part of the tested demo.
* Final position errors at viewpoints were 0.04–0.16 m and yaw errors 0.14–0.19 rad (goal tolerance
  0.20 m / 0.20 rad); the reader tolerates this, but a tighter yaw tolerance would centre the dial better.
* `gz_ros2_control` is not installed in this environment; the robot uses Gazebo's DiffDrive system
  (wheel-integrated odometry) instead of ros2_control controllers.
* The camera stream is intermittently reported `DEGRADED` (< 5 Hz in sim time) by the health monitor
  under full load; it did not affect readings (bursts wait for 5 fresh frames).
* On shutdown Nav2's composed container sometimes needs SIGKILL / prints a crash during cleanup
  (upstream behaviour, after the report has been written).
* Gauge reading takes ~0.1–0.4 s per 5-frame burst in the V3 worker process. Inside the ROS node
  process (V2, or `reader_worker_process: false`) GIL contention with rclpy callbacks stretched
  it to 2–12 s; with V3's result-age limit that makes observations `STALE_OBSERVATION`.
* rclpy nodes with `use_sim_time` spend ~0.5 core each just processing Gazebo's ~1 kHz `/clock`
  (measured); the Python nodes are CPU-heavy for that reason, not because of their own work.

## Safety layer
* `cmd_vel_gate` / `health_monitor` are a **simulated safety and health guard**, not a certified
  or industrial safety function (no redundancy, no safety-rated hardware/OS, no SIL/PL analysis).
* A single scan with a stamp > 0.2 s ahead of the gate's ROS clock was observed once at start-up
  (robot idle, `/clock` just starting) and caused a 0.1 s `STOPPED_SCAN_FAULT`; not seen during
  navigation.
* **Coverage is angular, not volumetric (V3).** The sector checks prove that the scan *plane*
  observes the corridor; objects above or below the 2-D scan plane (the PG-12 conduit box) are
  invisible to the gate, exactly as to Nav2. Unobserved wedges ≤ 2° are tolerated (a thin pole can
  hide in one at short range). The 95 % / 2° / 1° thresholds are hand-set, not derived from a
  stopping-distance analysis.
* **Rotation** is blocked by an obstacle inside a 0.35 m radius and by < 95 % whole-circle coverage;
  a robot pressed against an obstacle can therefore only translate away. A 270° scanner would make
  in-place rotation and reverse motion impossible by design (tested in unit tests only).
* **Goal ownership unknown is sticky.** If `send_goal_async` itself raises, the runner cannot know
  whether a goal exists and inhibits motion for the rest of the mission (e-stop latched); there is
  no cancel-all fallback through the action's cancel service.

## Verification gaps
* RViz2 ran during the GUI mission but its rendering was not visually verified (WSLg screen capture
  unavailable; GLSL warning for the costmap colour shader on the WSL Mesa driver).
* CI workflow not executed (repository not published from this environment).
* The independent reviewer could not run ROS 2 / Gazebo; all live results are developer-reported
  from this machine (logs, records and images are archived in `docs/evidence/`).
* Results come from a handful of runs of one world; no randomised or long-duration testing.
