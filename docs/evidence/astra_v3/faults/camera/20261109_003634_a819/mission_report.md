# InspectLoop Gauge Inspection Report — fault_injection_single_gauge

- **Mission ID:** `20261109_003634_a819`
- **Mission state:** **COMPLETED**
- **Started (wall clock):** 2026-11-09T00:36:34
- **Duration:** 46.5 s sim time / 51.5 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **1** — accepted **0**, unresolved **1**. Observations: 2, viewpoints visited: 1 (navigation succeeded 1, mean final position error 0.144 m).

Decisions: **RETRY_SAME** ×1, **UNRESOLVED** ×1

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | UNRESOLVED [CAMERA_UNAVAILABLE] — camera delivers no frames: moving cannot fix a sensor fault | 2 | 1 | RETRY_SAME → UNRESOLVED |

## PG-11 — Rack N1 - compressed-air header

**UNRESOLVED [CAMERA_UNAVAILABLE] — camera delivers no frames: moving cannot fix a sensor fault**  
Footprint-clear viewpoint candidates: 15 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 20.4 | V0 d=1.50 a=+0deg | OBSERVATION #1 | - | - | NO_IMAGE | **RETRY_SAME** | possibly transient (NO_IMAGE): observe again from the same pose |
| 27.1 | V0 d=1.50 a=+0deg | OBSERVATION #2 | - | - | NO_IMAGE | **UNRESOLVED** | camera delivers no frames: moving cannot fix a sensor fault |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 14.2 s, final error 0.144 m
  - obs #1: frames 0/0 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → RETRY_SAME
  - obs #2: frames 0/0 valid (tag in 0), visibility 0.00, needle conf 0.00, view angle -°, distance - m, dial radius 0.0 px → UNRESOLVED

## Return to home

SUCCEEDED (1 attempt(s)).

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 15.0 | inspectloop/stream/camera/image_raw | ERROR | STALE |
| 15.0 | inspectloop/stream/camera/camera_info | ERROR | STALE |
