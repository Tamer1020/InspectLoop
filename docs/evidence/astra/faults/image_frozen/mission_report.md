# InspectLoop Gauge Inspection Report — fault_injection_single_gauge

- **Mission ID:** `20261108_203807_dd7b`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-08T20:38:07
- **Duration:** 36.6 s sim time / 41.5 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **1** — accepted **0**, unresolved **1**. Observations: 2, viewpoints visited: 1 (navigation succeeded 1, mean final position error 0.137 m).

Decisions: **RETRY_SAME** ×1, **UNRESOLVED** ×1

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | UNRESOLVED — FROZEN_FRAMES persisted: moving cannot fix it | 2 | 1 | RETRY_SAME → UNRESOLVED |

## PG-11 — Rack N1 - compressed-air header

**UNRESOLVED — FROZEN_FRAMES persisted: moving cannot fix it**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 15.3 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | TAG_NOT_VISIBLE, FROZEN_FRAMES | **RETRY_SAME** | possibly transient (FROZEN_FRAMES): observe again from the same pose |
| 17.5 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | TAG_NOT_VISIBLE, FROZEN_FRAMES | **UNRESOLVED** | FROZEN_FRAMES persisted: moving cannot fix it |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 13.6 s, final error 0.137 m
  - obs #1: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME

    ![PG-11 obs 1](images/PG-11_V0_t00024.70_annotated.png)

  - obs #2: frames 0/5 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

    ![PG-11 obs 2](images/PG-11_V0_t00026.80_annotated.png)


## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 13.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 13.0 | inspectloop/stream/camera/camera_info | ERROR | STALE |
| 14.0 | inspectloop/stream/camera/image_raw | OK | OK |
