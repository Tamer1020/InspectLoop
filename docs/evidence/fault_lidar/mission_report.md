# InspectLoop Gauge Inspection Report — fault_injection_single_gauge

- **Mission ID:** `20261108_163540_0c34`
- **Mission state:** **ABORTED** — LiDAR stream failed (health_monitor ERROR) - stopping for safety
- **Started (wall clock):** 2026-11-08T16:35:40
- **Duration:** 8.3 s sim time / 12.1 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Acceptance rule:** sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **0** — accepted **0**, unresolved **0**. Observations: 0, viewpoints visited: 0 (navigation succeeded 0, mean final position error - m).

Decisions: 

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|

## System health events

| Sim time (s) | Component | Level | Message |
|---|---|---|---|
| 20.0 | inspectloop/stream/scan | ERROR | STALE |
