# InspectLoop Gauge Inspection Report — fault_injection_single_gauge

- **Mission ID:** `20261108_163325_c8f5`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-08T16:33:25
- **Duration:** 46.0 s sim time / 52.2 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Acceptance rule:** sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **1** — accepted **0**, unresolved **1**. Observations: 2, viewpoints visited: 1 (navigation succeeded 1, mean final position error 0.125 m).

Decisions: **RETRY_SAME** ×1, **UNRESOLVED** ×1

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | UNRESOLVED — camera delivers no frames: moving cannot fix a sensor fault | 2 | 1 | RETRY_SAME → UNRESOLVED |

## PG-11 — Rack N1 - compressed-air header

**UNRESOLVED — camera delivers no frames: moving cannot fix a sensor fault**  
Feasible viewpoint candidates: 15, rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 19.6 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | NO_IMAGE | **RETRY_SAME** | possibly transient (NO_IMAGE): observe again from the same pose |
| 26.3 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | NO_IMAGE | **UNRESOLVED** | camera delivers no frames: moving cannot fix a sensor fault |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 14.0 s, final error 0.125 m
  - obs #1: frames 0/0 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME
  - obs #2: frames 0/0 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 15.0 | inspectloop/stream/camera/image_raw | ERROR | STALE |
| 15.0 | inspectloop/stream/camera/camera_info | ERROR | STALE |
