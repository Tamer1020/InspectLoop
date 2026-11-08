# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261108_180827_0687`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-08T18:08:27
- **Duration:** 233.0 s sim time / 246.2 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **4** — accepted **3**, unresolved **1**. Observations: 11, viewpoints visited: 8 (navigation succeeded 8, mean final position error 0.094 m).

Decisions: **ACCEPT** ×3, **MOVE_NEW** ×4, **RETRY_SAME** ×3, **UNRESOLVED** ×1

**Alarm limits exceeded:** PG-14

**Recovered by autonomous viewpoint change:** PG-12

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | 4.19 bar (heuristic σ 0.05), alarm NORMAL | 1 | 1 | ACCEPT |
| PG-12 | 6.77 bar (heuristic σ 0.07), alarm NORMAL | 3 | 3 | MOVE_NEW → MOVE_NEW → ACCEPT |
| PG-13 | UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-14 | 8.69 bar (heuristic σ 0.05), alarm HIGH | 1 | 1 | ACCEPT |

## PG-11 — Rack N1 - compressed-air header

**4.19 bar (heuristic σ 0.05), alarm NORMAL**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 24.3 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 4.193 | 0.052 | - | **ACCEPT** | 5 valid frames, sigma 0.052 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 13.7 s, final error 0.104 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 21.6°, distance 1.25 m, dial radius 73.4 px → ACCEPT

    ![PG-11 obs 1](images/PG-11_V0_t00024.80_annotated.png)


## PG-12 — Rack S2 - hydraulic accumulator

**6.77 bar (heuristic σ 0.07), alarm NORMAL**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 33.5 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | OCCLUDED, LOW_NEEDLE_CONTRAST, AMBIGUOUS_NEEDLE, NEEDLE_PARTIAL | **MOVE_NEW** | viewpoint-dependent problem (AMBIGUOUS_NEEDLE, LOW_NEEDLE_CONTRAST, NEEDLE_PARTIAL, OCCLUDED) (hint: different_side) |
| 49.8 | V3 d=1.50 a=-45deg | OBSERVATION #2 | 6.775 | 0.082 | TOO_FAR | **MOVE_NEW** | viewpoint-dependent problem (TOO_FAR) (hint: closer) |
| 57.6 | V8 d=1.20 a=-45deg | OBSERVATION #3 | 6.774 | 0.074 | - | **ACCEPT** | 5 valid frames, sigma 0.074 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-1.0, -2.49): navigation SUCCEEDED in 28.3 s, final error 0.069 m
  - obs #1: frames 0/5 valid (tag in 5), visibility 0.01, needle conf 0.00, view angle 20.9°, distance 1.32 m, dial radius 68.7 px → MOVE_NEW

    ![PG-12 obs 1](images/PG-12_V0_t00064.10_annotated.png)

- **V3 d=1.50 a=-45deg** at (0.061, -2.929): navigation SUCCEEDED in 10.6 s, final error 0.105 m
  - obs #2: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.97, view angle 49.1°, distance 1.43 m, dial radius 53.0 px → MOVE_NEW

    ![PG-12 obs 2](images/PG-12_V3_t00079.90_annotated.png)

- **V8 d=1.20 a=-45deg** at (-0.151, -3.141): navigation SUCCEEDED in 1.7 s, final error 0.187 m
  - obs #3: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 48.3°, distance 1.31 m, dial radius 58.8 px → ACCEPT

    ![PG-12 obs 3](images/PG-12_V8_t00087.60_annotated.png)


## PG-13 — Rack N3 - sprinkler riser

**UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left**  
Feasible viewpoint candidates: 8, rejected by the map (collision/clearance): 7.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 29.1 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 31.5 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 43.7 | V1 d=1.50 a=-30deg | OBSERVATION #3 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 46.4 | V1 d=1.50 a=-30deg | OBSERVATION #4 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 60.1 | V2 d=1.50 a=+30deg | OBSERVATION #5 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 62.7 | V2 d=1.50 a=+30deg | OBSERVATION #6 | - | - | TAG_NOT_VISIBLE | **UNRESOLVED** | TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (5.0, 2.49): navigation SUCCEEDED in 26.8 s, final error 0.108 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 1](images/PG-13_V0_t00120.30_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 2](images/PG-13_V0_t00123.00_annotated.png)

- **V1 d=1.50 a=-30deg** at (4.25, 2.691): navigation SUCCEEDED in 9.9 s, final error 0.063 m
  - obs #3: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 3](images/PG-13_V1_t00135.10_annotated.png)

  - obs #4: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 4](images/PG-13_V1_t00137.60_annotated.png)

- **V2 d=1.50 a=+30deg** at (5.75, 2.691): navigation SUCCEEDED in 11.7 s, final error 0.039 m
  - obs #5: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 5](images/PG-13_V2_t00151.60_annotated.png)

  - obs #6: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-13 obs 6](images/PG-13_V2_t00154.10_annotated.png)


## PG-14 — East wall - boiler feed line

**8.69 bar (heuristic σ 0.05), alarm HIGH**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 26.0 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 8.685 | 0.053 | - | **ACCEPT** | 5 valid frames, sigma 0.053 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (10.29, -0.0): navigation SUCCEEDED in 21.0 s, final error 0.074 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 21.1°, distance 1.25 m, dial radius 72.9 px → ACCEPT

    ![PG-14 obs 1](images/PG-14_V0_t00177.30_annotated.png)


## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 86.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 87.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 91.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 92.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 105.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 106.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 107.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 109.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 140.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 142.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 173.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 175.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 198.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 199.0 | inspectloop/stream/camera/image_raw | OK | OK |
