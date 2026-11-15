# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261115_130358_ed42`
- **Mission state:** **ABORTED** — interrupted (SIGINT)
- **Started (wall clock):** 2026-11-15T13:03:58
- **Duration:** 47.8 s sim time / 53.0 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **1** — accepted **1**, unresolved **0**. Observations: 1, viewpoints visited: 1 (navigation succeeded 1, mean final position error 0.094 m).

Decisions: **ACCEPT** ×1

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|
| PG-11 | 4.21 bar (heuristic σ 0.05), alarm NORMAL | 1 | 1 | ACCEPT |

## PG-11 — Rack N1 - compressed-air header

**4.21 bar (heuristic σ 0.05), alarm NORMAL**  
Footprint-clear viewpoint candidates: 15 (Nav2 path reachability not pre-checked), rejected by the map (collision/clearance): 0.

| t (s) | Viewpoint | Event | Value | σ (heuristic) | Quality flags | Decision | Reason |
|---|---|---|---|---|---|---|---|
| 16.2 | V0 d=1.50 a=+0deg | OBSERVATION #1 | 4.206 | 0.054 | - | **ACCEPT** | 5 valid frames, sigma 0.054 <= 0.15 |

- **V0 d=1.50 a=+0deg** at (-7.0, 2.49): navigation SUCCEEDED in 14.0 s, final error 0.094 m
  - obs #1: frames 5/5 valid (tag in 5), visibility 1.00, needle conf 0.99, view angle 20.6°, distance 1.29 m, dial radius 70.5 px → ACCEPT

    ![PG-11 obs 1](images/PG-11_V0_t00026.70_annotated.png)


## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 38.0 | inspectloop/stream/camera/image_raw | WARN | DEGRADED |
| 39.0 | inspectloop/stream/camera/image_raw | OK | OK |
