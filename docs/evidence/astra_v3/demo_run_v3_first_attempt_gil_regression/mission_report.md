# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261109_001640_0420`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-09T00:16:40
- **Duration:** 382.9 s sim time / 397.0 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **4** — accepted **1**, unresolved **3**. Observations: 20, viewpoints visited: 11 (navigation succeeded 11, mean final position error 0.114 m).

Decisions: **RETRY_SAME** ×9, **MOVE_NEW** ×7, **UNRESOLVED** ×3, **ACCEPT** ×1

**Recovered by autonomous viewpoint change:** PG-12

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | UNRESOLVED [IMAGE_INVALID] — STALE_OBSERVATION persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-12 | 6.82 bar (heuristic σ 0.08), alarm NORMAL | 2 | 2 | MOVE_NEW → ACCEPT |
| PG-13 | UNRESOLVED [INSUFFICIENT_VISUAL_EVIDENCE] — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |
| PG-14 | UNRESOLVED [IMAGE_INVALID] — STALE_OBSERVATION persisted and no viewpoint/observation budget left | 6 | 3 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED |

## PG-11 — Rack N1 - compressed-air header

**UNRESOLVED [IMAGE_INVALID] — STALE_OBSERVATION persisted and no viewpoint/observation budget left**  
Footprint-clear viewpoint candidates: 15 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 28.7 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION, WRONG_VIEWPOINT): observe again from the same pose |
| 40.5 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | STALE_OBSERVATION | **MOVE_NEW** | STALE_OBSERVATION persisted after 1 retry(ies) (hint: any) |
| 51.7 | V5 d=1.20 a=+0deg | OBSERVATION #3 | - | - | STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION): observe again from the same pose |
| 65.1 | V5 d=1.20 a=+0deg | OBSERVATION #4 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **MOVE_NEW** | STALE_OBSERVATION, WRONG_VIEWPOINT persisted after 1 retry(ies) (hint: any) |
| 80.3 | V10 d=1.90 a=+0deg | OBSERVATION #5 | - | - | STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION): observe again from the same pose |
| 90.8 | V10 d=1.90 a=+0deg | OBSERVATION #6 | - | - | STALE_OBSERVATION | **UNRESOLVED** | STALE_OBSERVATION persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 13.9 s, final error 0.093 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 21.5°, distance 1.25 m, dial radius 73.1 px → RETRY_SAME

    ![PG-11 obs 1](images/PG-11_V0_t00025.90_annotated.png)

  - obs #2: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 21.5°, distance 1.25 m, dial radius 73.1 px → MOVE_NEW

    ![PG-11 obs 2](images/PG-11_V0_t00041.00_annotated.png)

- **V5 d=1.20 a=+0deg** at (-7.0, 2.79): navigation SUCCEEDED in 0.6 s, final error 0.194 m
  - obs #3: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 21.4°, distance 1.24 m, dial radius 73.3 px → RETRY_SAME

    ![PG-11 obs 3](images/PG-11_V5_t00052.90_annotated.png)

  - obs #4: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 21.4°, distance 1.24 m, dial radius 73.3 px → MOVE_NEW

    ![PG-11 obs 4](images/PG-11_V5_t00064.00_annotated.png)

- **V10 d=1.90 a=+0deg** at (-7.0, 2.09): navigation SUCCEEDED in 3.0 s, final error 0.180 m
  - obs #5: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 17.1°, distance 1.58 m, dial radius 56.0 px → RETRY_SAME

    ![PG-11 obs 5](images/PG-11_V10_t00080.10_annotated.png)

  - obs #6: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 17.1°, distance 1.58 m, dial radius 56.0 px → UNRESOLVED

    ![PG-11 obs 6](images/PG-11_V10_t00092.50_annotated.png)


## PG-12 — Rack S2 - hydraulic accumulator

**6.82 bar (heuristic σ 0.08), alarm NORMAL**  
Footprint-clear viewpoint candidates: 15 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 37.9 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | OCCLUDED, LOW_NEEDLE_CONTRAST, AMBIGUOUS_NEEDLE, NEEDLE_PARTIAL | **MOVE_NEW** | viewpoint-dependent problem (AMBIGUOUS_NEEDLE, LOW_NEEDLE_CONTRAST, NEEDLE_PARTIAL, OCCLUDED) (hint: different_side) |
| 58.3 | V3 d=1.50 a=-45deg | OBSERVATION #2 | 6.819 | 0.075 | - | **ACCEPT** | 5 valid frames, sigma 0.075 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-1.0, -2.49): navigation SUCCEEDED in 28.4 s, final error 0.093 m
  - obs #1: frames 0/5 valid (tag in 5), visibility 0.00, needle conf 0.00, view angle 19.7°, distance 1.35 m, dial radius 66.4 px → MOVE_NEW

    ![PG-12 obs 1](images/PG-12_V0_t00131.20_annotated.png)

- **V3 d=1.50 a=-45deg** at (0.061, -2.929): navigation SUCCEEDED in 10.8 s, final error 0.080 m
  - obs #2: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 47.1°, distance 1.38 m, dial radius 57.7 px → ACCEPT

    ![PG-12 obs 2](images/PG-12_V3_t00151.60_annotated.png)


## PG-13 — Rack N3 - sprinkler riser

**UNRESOLVED [INSUFFICIENT_VISUAL_EVIDENCE] — TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left**  
Footprint-clear viewpoint candidates: 8 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 7.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 28.6 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 31.2 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 43.4 | V1 d=1.50 a=-30deg | OBSERVATION #3 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 46.3 | V1 d=1.50 a=-30deg | OBSERVATION #4 | - | - | TAG_NOT_VISIBLE | **MOVE_NEW** | TAG_NOT_VISIBLE persisted after 1 retry(ies) (hint: different_side) |
| 61.4 | V2 d=1.50 a=+30deg | OBSERVATION #5 | - | - | TAG_NOT_VISIBLE | **RETRY_SAME** | possibly transient (TAG_NOT_VISIBLE): observe again from the same pose |
| 64.3 | V2 d=1.50 a=+30deg | OBSERVATION #6 | - | - | TAG_NOT_VISIBLE | **UNRESOLVED** | TAG_NOT_VISIBLE persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (5.0, 2.49): navigation SUCCEEDED in 26.3 s, final error 0.086 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 1](images/PG-13_V0_t00187.90_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 2](images/PG-13_V0_t00190.50_annotated.png)

- **V1 d=1.50 a=-30deg** at (4.25, 2.691): navigation SUCCEEDED in 10.2 s, final error 0.050 m
  - obs #3: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 3](images/PG-13_V1_t00202.80_annotated.png)

  - obs #4: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → MOVE_NEW

    ![PG-13 obs 4](images/PG-13_V1_t00205.40_annotated.png)

- **V2 d=1.50 a=+30deg** at (5.75, 2.691): navigation SUCCEEDED in 12.6 s, final error 0.025 m
  - obs #5: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-13 obs 5](images/PG-13_V2_t00220.40_annotated.png)

  - obs #6: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-13 obs 6](images/PG-13_V2_t00223.50_annotated.png)


## PG-14 — East wall - boiler feed line

**UNRESOLVED [IMAGE_INVALID] — STALE_OBSERVATION persisted and no viewpoint/observation budget left**  
Footprint-clear viewpoint candidates: 15 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 32.4 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION): observe again from the same pose |
| 46.8 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **MOVE_NEW** | STALE_OBSERVATION, WRONG_VIEWPOINT persisted after 1 retry(ies) (hint: any) |
| 62.5 | V5 d=1.20 a=+0deg | OBSERVATION #3 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION, WRONG_VIEWPOINT): observe again from the same pose |
| 78.3 | V5 d=1.20 a=+0deg | OBSERVATION #4 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **MOVE_NEW** | STALE_OBSERVATION, WRONG_VIEWPOINT persisted after 1 retry(ies) (hint: any) |
| 96.4 | V10 d=1.90 a=+0deg | OBSERVATION #5 | - | - | WRONG_VIEWPOINT, STALE_OBSERVATION | **RETRY_SAME** | possibly transient (STALE_OBSERVATION, WRONG_VIEWPOINT): observe again from the same pose |
| 107.4 | V10 d=1.90 a=+0deg | OBSERVATION #6 | - | - | STALE_OBSERVATION | **UNRESOLVED** | STALE_OBSERVATION persisted and no viewpoint/observation budget left |

- **V0 d=1.50 a=+0deg** at (10.29, -0.0): navigation SUCCEEDED in 21.0 s, final error 0.082 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 20.8°, distance 1.27 m, dial radius 71.8 px → RETRY_SAME

    ![PG-14 obs 1](images/PG-14_V0_t00247.00_annotated.png)

  - obs #2: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 20.8°, distance 1.27 m, dial radius 71.8 px → MOVE_NEW

    ![PG-14 obs 2](images/PG-14_V0_t00258.90_annotated.png)

- **V5 d=1.20 a=+0deg** at (10.59, -0.0): navigation SUCCEEDED in 0.8 s, final error 0.194 m
  - obs #3: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 21.6°, distance 1.23 m, dial radius 73.1 px → RETRY_SAME

    ![PG-14 obs 3](images/PG-14_V5_t00273.60_annotated.png)

  - obs #4: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 21.6°, distance 1.23 m, dial radius 73.1 px → MOVE_NEW

    ![PG-14 obs 4](images/PG-14_V5_t00288.90_annotated.png)

- **V10 d=1.90 a=+0deg** at (9.89, -0.0): navigation SUCCEEDED in 2.8 s, final error 0.177 m
  - obs #5: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 17.7°, distance 1.57 m, dial radius 55.8 px → RETRY_SAME

    ![PG-14 obs 5](images/PG-14_V10_t00307.30_annotated.png)

  - obs #6: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.98, view angle 17.7°, distance 1.57 m, dial radius 55.8 px → UNRESOLVED

    ![PG-14 obs 6](images/PG-14_V10_t00322.80_annotated.png)


## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

No health level changes were recorded during the mission.
