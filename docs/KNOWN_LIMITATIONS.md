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
  (≤ 0.031 bar in the simulation runs) will not transfer to real cameras without re-calibration.
* **σ is a heuristic indicator** (frame spread + pixel-resolution term + 0.03 bar floor), used only
  as an acceptance threshold. It is not a calibrated probability and value ± k·σ is not a confidence
  interval. On the held-out synthetic set every accepted reading was within ±2σ_h (210/210) — a
  property of that sample, not a coverage guarantee — and σ_h does not model systematic errors that
  are identical in all frames of a burst.
* **Nearby lines below the resolution limit (Astra F1 residual risk).** A dark radial line within
  ~3–5° of the needle, especially with heavy blur, can merge with it into one lobe that passes
  every needle-evidence test. On the held-out set this produced 1 incorrect acceptance in 210
  (error 0.118 bar). The minimum dial radius of 40 px bounds, but does not remove, this risk.
* **Conservatism costs yield.** After the audit fixes only 210/400 held-out samples were accepted
  (v1: 324/400): partial occlusions, shadows, small dials and nearby lines are now refused even
  when a human could still read the gauge. In the simulation, PG-12 needs a third viewpoint.
* **Thresholds are hand-set** (visibility 0.85, needle confidence 0.35, σ_h ≤ 0.15 bar, ambiguity
  0.45, model residual 0.075, minor dial radius ≥ 40 px, …). The accepted PG-12 view (−45°, 1.2 m)
  has full visibility and needle confidence 0.98 but a view angle of 48° (limit 55°) and a minor
  dial radius that failed the 40 px limit at 1.5 m and passed at 1.2 m — a different pose
  could need another viewpoint.
* Partial occlusion that leaves the needle visible but covers part of the face lowers visibility and
  is treated as unreliable even if the needle could be read — a deliberate conservative choice.

## Planning / decisions
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
* Gauge reading in Python takes ~0.7–1.2 s per 5-frame burst on the test machine — fine for inspection,
  not optimised.

## Safety layer
* `cmd_vel_gate` / `health_monitor` are a **simulated safety and health guard**, not a certified
  or industrial safety function (no redundancy, no safety-rated hardware/OS, no SIL/PL analysis).
* A single scan with a stamp > 0.2 s ahead of the gate's ROS clock was observed once at start-up
  (robot idle, `/clock` just starting) and caused a 0.1 s `STOPPED_SCAN_FAULT`; not seen during
  navigation.

## Verification gaps
* RViz2 ran during the GUI mission but its rendering was not visually verified (WSLg screen capture
  unavailable; GLSL warning for the costmap colour shader on the WSL Mesa driver).
* CI workflow not executed (repository not published from this environment).
* Results come from a handful of runs of one world; no randomised or long-duration testing.
