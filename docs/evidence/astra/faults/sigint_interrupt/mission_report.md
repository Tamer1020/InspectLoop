# InspectLoop Gauge Inspection Report — warehouse_pressure_round

- **Mission ID:** `20261108_204207_1059`
- **Mission state:** **ABORTED** — interrupted (SIGINT)
- **Started (wall clock):** 2026-11-08T20:42:07
- **Duration:** 6.4 s sim time / 9.4 s wall time
- **Data source:** Gazebo Harmonic simulation. Every value below was read by OpenCV from simulated camera frames captured after the robot reached the viewpoint; the robot does not know the true gauge values.
- **Uncertainty:** sigma is a HEURISTIC uncertainty model (frame spread + pixel resolution + floor). It is a diagnostic, not a calibrated probability: no confidence level or coverage is claimed for value +/- k*sigma.
- **Acceptance rule:** heuristic sigma ≤ 0.15 , ≥ 3 valid frames of 5, no blocking quality flag; ≤ 1 RETRY_SAME per viewpoint, ≤ 3 viewpoints and ≤ 8 observations per gauge.

## Summary

Gauges: **0** — accepted **0**, unresolved **0**. Observations: 0, viewpoints visited: 0 (navigation succeeded 0, mean final position error - m).

Decisions: 

| Gauge | Result | Observations | Viewpoints | Decision sequence |
|---|---|---|---|---|

## System health events

No health level changes were recorded during the mission.
