# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261108_162324_98ef`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-08T16:23:24
- **Duration:** 228.2 s sim time / 244.0 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Acceptance rule:** sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **4** — accepted **2**, unresolved **2**. Observations: 13, viewpoints visited: 8 (navigation succeeded 8, mean final position error 0.093 m).

Decisions: **ACCEPT** ×2, **RETRY_SAME** ×5, **MOVE_NEW** ×4, **UNRESOLVED** ×2

**Alarm limits exceeded:** PG-14

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | 4.19 ± 0.19 bar (95%), alarm NORMAL | 1 | 1 | ACCEPT |
| PG-12 | UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left | 5 | 3 | RETRY_SAME → MOVE_NEW → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-13 | UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-14 | 8.68 ± 0.19 bar (95%), alarm HIGH | 1 | 1 | ACCEPT |

## PG-11 — Rack N1 - compressed-air header

**4.19 ± 0.19 bar (95%), alarm NORMAL**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 17.1 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 4.191 | 0.093 | - | **ACCEPT** | 5 valid frames, sigma 0.093 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 14.5 s, final error 0.121 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.97, view angle 22.2°, distance 1.24 m, dial radius 74.6 px → ACCEPT

    ![PG-11 obs 1](images/PG-11_V0_t00027.50_annotated.png)


## PG-12 — Rack S2 - hydraulic accumulator

**UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 29.5 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 32.0 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 45.1 | V3 d=1.50 a=-45deg | OBSERVATION #3 | 6.809 | 0.167 | - | **MOVE_NEW** | viewpoint-dependent problem (HIGH_UNCERTAINTY) (hint: closer) |
| 56.7 | V5 d=1.20 a=+0deg | OBSERVATION #4 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 59.3 | V5 d=1.20 a=+0deg | OBSERVATION #5 | - | - | TAG_NOT_VISIBLE | **UNRESOLVED** | TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (-1.0, -2.49): navigation SUCCEEDED in 28.1 s, final error 0.151 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-12 obs 1](images/PG-12_V0_t00057.30_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-12 obs 2](images/PG-12_V0_t00059.70_annotated.png)

- **V3 d=1.50 a=-45deg** at (0.061, -2.929): navigation SUCCEEDED in 10.5 s, final error 0.071 m
  - obs #3: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.96, view angle 48.2°, distance 1.41 m, dial radius 55.3 px → MOVE_NEW

    ![PG-12 obs 3](images/PG-12_V3_t00072.30_annotated.png)

- **V5 d=1.20 a=+0deg** at (-1.0, -2.79): navigation SUCCEEDED in 10.1 s, final error 0.076 m
  - obs #4: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-12 obs 4](images/PG-12_V5_t00084.40_annotated.png)

  - obs #5: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-12 obs 5](images/PG-12_V5_t00086.90_annotated.png)


## PG-13 — Rack N3 - sprinkler riser

**UNRESOLVED — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left**  
Feasible viewpoint candidates: 8, rejected by the map (collision/clearance): 7.

| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 31.1 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 33.7 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 45.8 | V1 d=1.50 a=-30deg | OBSERVATION #3 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 48.4 | V1 d=1.50 a=-30deg | OBSERVATION #4 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 61.7 | V2 d=1.50 a=+30deg | OBSERVATION #5 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 64.3 | V2 d=1.50 a=+30deg | OBSERVATION #6 | - | - | TAG_NOT_VISIBLE | **UNRESOLVED** | TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (5.0, 2.49): navigation SUCCEEDED in 29.6 s, final error 0.101 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 1](images/PG-13_V0_t00118.20_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 2](images/PG-13_V0_t00120.70_annotated.png)

- **V1 d=1.50 a=-30deg** at (4.25, 2.691): navigation SUCCEEDED in 10.3 s, final error 0.075 m
  - obs #3: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 3](images/PG-13_V1_t00132.50_annotated.png)

  - obs #4: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 4](images/PG-13_V1_t00135.30_annotated.png)

- **V2 d=1.50 a=+30deg** at (5.75, 2.691): navigation SUCCEEDED in 11.5 s, final error 0.040 m
  - obs #5: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 5](images/PG-13_V2_t00148.60_annotated.png)

  - obs #6: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-13 obs 6](images/PG-13_V2_t00151.30_annotated.png)


## PG-14 — East wall - boiler feed line

**8.68 ± 0.19 bar (95%), alarm HIGH**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 23.1 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 8.677 | 0.094 | - | **ACCEPT** | 5 valid frames, sigma 0.094 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (10.29, -0.0): navigation SUCCEEDED in 20.9 s, final error 0.109 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 0.99, needle conf 0.95, view angle 21.3°, distance 1.24 m, dial radius 73.0 px → ACCEPT

    ![PG-14 obs 1](images/PG-14_V0_t00173.90_annotated.png)


## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 16.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 17.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 42.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 43.0 | inspectloop/stream/camera/image_raw | OK | OK |
| 179.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 180.0 | inspectloop/stream/camera/image_raw | OK | OK |
