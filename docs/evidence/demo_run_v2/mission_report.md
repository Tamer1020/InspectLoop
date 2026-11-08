# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261108_204253_cb8e`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-08T20:42:53
- **Duration:** 217.5 s sim time / 234.0 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **4** — accepted **3**, unresolved **1**. Observations: 10, viewpoints visited: 7 (navigation succeeded 7, mean final position error 0.090 m).

Decisions: **ACCEPT** ×3, **MOVE_NEW** ×3, **RETRY_SAME** ×3, **UNRESOLVED** ×1

**Alarm limits exceeded:** PG-14

**Recovered by autonomous viewpoint change:** PG-12

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | 4.20 bar (heuristic σ 0.05), alarm NORMAL | 1 | 1 | ACCEPT |
| PG-12 | 6.79 bar (heuristic σ 0.08), alarm NORMAL | 2 | 2 | MOVE_NEW → ACCEPT |
| PG-13 | UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-14 | 8.70 bar (heuristic σ 0.05), alarm HIGH | 1 | 1 | ACCEPT |

## PG-11 — Rack N1 - compressed-air header

**4.20 bar (heuristic σ 0.05), alarm NORMAL**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 17.6 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 4.200 | 0.051 | - | **ACCEPT** | 5 valid frames, sigma 0.051 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 13.8 s, final error 0.109 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 22.4°, distance 1.20 m, dial radius 76.7 px → ACCEPT

    ![PG-11 obs 1](images/PG-11_V0_t00026.70_annotated.png)


## PG-12 — Rack S2 - hydraulic accumulator

**6.79 bar (heuristic σ 0.08), alarm NORMAL**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 32.7 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | OCCLUDED, LOW_NEEDLE_CONTRAST, AMBIGUOUS_NEEDLE, NEEDLE_PARTIAL | **MOVE_NEW** | viewpoint-dependent problem (AMBIGUOUS_NEEDLE, LOW_NEEDLE_CONTRAST, NEEDLE_PARTIAL, OCCLUDED) (hint: different_side) |
| 47.7 | V3 d=1.50 a=-45deg | OBSERVATION #2 | 6.794 | 0.081 | - | **ACCEPT** | 5 valid frames, sigma 0.081 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-1.0, -2.49): navigation SUCCEEDED in 28.0 s, final error 0.096 m
  - obs #1: frames 0/5 valid (tag in 5), visibility 0.01, needle conf 0.00, view angle 15.0°, distance 1.38 m, dial radius 68.0 px → MOVE_NEW

    ![PG-12 obs 1](images/PG-12_V0_t00058.10_annotated.png)

- **V3 d=1.50 a=-45deg** at (0.061, -2.929): navigation SUCCEEDED in 10.0 s, final error 0.085 m
  - obs #2: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 48.9°, distance 1.43 m, dial radius 53.8 px → ACCEPT

    ![PG-12 obs 2](images/PG-12_V3_t00072.80_annotated.png)


## PG-13 — Rack N3 - sprinkler riser

**UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left**  
Feasible viewpoint candidates: 8, rejected by the map (collision/clearance): 7.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 28.2 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 30.4 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 42.8 | V1 d=1.50 a=-30deg | OBSERVATION #3 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 45.9 | V1 d=1.50 a=-30deg | OBSERVATION #4 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 60.7 | V2 d=1.50 a=+30deg | OBSERVATION #5 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 63.3 | V2 d=1.50 a=+30deg | OBSERVATION #6 | - | - | TAG_NOT_VISIBLE | **UNRESOLVED** | TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (5.0, 2.49): navigation SUCCEEDED in 25.7 s, final error 0.148 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 1](images/PG-13_V0_t00104.00_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 2](images/PG-13_V0_t00106.50_annotated.png)

- **V1 d=1.50 a=-30deg** at (4.25, 2.691): navigation SUCCEEDED in 10.3 s, final error 0.043 m
  - obs #3: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 3](images/PG-13_V1_t00118.70_annotated.png)

  - obs #4: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 4](images/PG-13_V1_t00122.00_annotated.png)

- **V2 d=1.50 a=+30deg** at (5.75, 2.691): navigation SUCCEEDED in 12.6 s, final error 0.059 m
  - obs #5: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 5](images/PG-13_V2_t00136.50_annotated.png)

  - obs #6: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-13 obs 6](images/PG-13_V2_t00139.20_annotated.png)


## PG-14 — East wall - boiler feed line

**8.70 bar (heuristic σ 0.05), alarm HIGH**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 26.3 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 8.702 | 0.055 | - | **ACCEPT** | 5 valid frames, sigma 0.055 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (10.29, -0.0): navigation SUCCEEDED in 20.1 s, final error 0.093 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 20.0°, distance 1.32 m, dial radius 68.5 px → ACCEPT

    ![PG-14 obs 1](images/PG-14_V0_t00161.60_annotated.png)


## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 13.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 14.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 21.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 22.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 24.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 27.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 28.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 32.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 48.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 49.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 99.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 100.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 109.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 110.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 176.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 177.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 210.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 211.0 | inspectloop/stream/camera/image_raw | OK | OK |
