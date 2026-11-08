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
* **σ is a heuristic model** (frame spread + pixel-resolution term + 0.03 bar floor). It was checked
  against ground truth on 96 synthetic observations (conservative: all within 1σ) and 9 accepted
  simulation readings (all within 2σ). It is not a statistically validated confidence interval, and it
  does not model systematic errors that are identical in all frames of a burst (e.g. a wrong pivot).
* **Thresholds are hand-set** (visibility 0.85, needle confidence 0.35, σ ≤ 0.15 bar, …). The accepted
  PG-12 view at −45° passed with modest margins (visibility 0.87, confidence 0.46); a slightly
  different occluder or pose could turn it into another MOVE_NEW.
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

## Verification gaps
* RViz2 ran during the GUI mission but its rendering was not visually verified (WSLg screen capture
  unavailable; GLSL warning for the costmap colour shader on the WSL Mesa driver).
* CI workflow not executed (repository not published from this environment).
* Results come from a handful of runs of one world; no randomised or long-duration testing.
