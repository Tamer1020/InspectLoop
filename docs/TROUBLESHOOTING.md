# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `package 'inspectloop_…' not found` | workspace not sourced | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Mission aborts: `system not ready … missing /amcl_pose` | AMCL not active / initial pose not set | check `ros2 lifecycle get /amcl`; the initial pose comes from `nav2_params.yaml` (`set_initial_pose: true`). The mission subscribes transient-local, so a late start is fine. |
| `missing /gauge/read service` | `gauge_reader_node` crashed or not started | check its log; it wraps processing errors into `PROCESSING_ERROR` results (V3; V2 said `NO_IMAGE`) instead of dying |
| Observations `PROCESSING_ERROR` | exception in the reader or its worker process | the node log has the traceback; the worker is restarted for the next request |
| Observations `STALE_OBSERVATION`, detail "result too old when consumed" | reading took longer than `max_result_age_s` (8 s sim) | check `processing_time_ms` in `gauge_readings.jsonl`; with `reader_worker_process: false` the reader competes for the GIL with rclpy callbacks and can take 8–12 s (observed in V3) |
| Observations `STALE_OBSERVATION` + `WRONG_VIEWPOINT`, "pose at acquisition time unknown" | TF lookup at the frame stamps failed (TF buffer holds 10 s) | same cause as above if processing was slow; otherwise check `map → base_footprint` |
| Robot stops, gate `BLOCKED_UNOBSERVED (forward sector unobserved)` | the LiDAR does not observe the forward corridor (NaN / out-of-range beams, narrow FOV, coarse resolution) | `ros2 topic echo /scan --field ranges`; thresholds `sector_*` in `safety.yaml`; the mission aborts after 3 s |
| Gate `BLOCKED_OBSTACLE` while only turning (output w = 0) | obstacle inside the 0.35 m turning radius | intended: in-place rotation would sweep the footprint corners into it |
| Every observation `CLOCK_RESET`, mission log "ROS clock discontinuity" | the simulation clock went backwards (world reset, bag replay) | intended: results spanning a clock discontinuity are never used; restart the mission after a reset |
| Every observation `NO_IMAGE` | camera bridge not running / no rendering | `ros2 topic hz /camera/image_raw`; headless mode needs `--headless-rendering` (set by the launch) and a working EGL/GPU driver |
| Gauge never visible (`TAG_NOT_VISIBLE`) where it should be | world/texture not found (`model://gauge_panel_*`) | `GZ_SIM_RESOURCE_PATH` is appended by `sim.launch.py`; after editing the layout regenerate the world (SETUP.md §4) |
| Readings biased / many `LOW_NEEDLE_CONTRAST` after editing textures | texture constants out of sync | `colcon test --packages-select inspectloop_vision inspectloop_gazebo` (constant ↔ texture tests) |
| Robot does not move, gate log `STOPPED_SCAN_FAULT` | no `/scan` for 0.5 s | `ros2 topic hz /scan`; LiDAR bridge `gz_lidar_bridge` must be running |
| Robot does not move, gate `STOPPED_ESTOP` | e-stop latched | `ros2 service call /cmd_vel_gate/set_estop std_srvs/srv/SetBool "{data: false}"` |
| Nav2 goals fail near racks | viewpoint inside inflation / dynamic obstacle | the planner already checks 0.45 m clearance; the mission retries, clears costmaps and then moves to another viewpoint |
| Processes left after Ctrl-C | crash during shutdown | `scripts/cleanup_sim.sh` (only matches InspectLoop command lines) |
| `mission_manager` takes up to ~40 s to exit after Nav2 was killed first | bounded goal cleanup (accept 15 s, cancel 10 s, terminal 15 s) before the e-stop is latched | intended fail-safe; the launch's `sigterm_timeout` is 40 s |
| GLSL link error `indexed_8bit_image` in the RViz log | RViz map shader vs. WSL Mesa driver | observed on the test machine; the impact on rendering was not verified (WSLg screen capture unavailable) |
| Want a SLAM map instead of the generated one | | `ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true` with the sim running, drive with `teleop_twist_keyboard` (remap `cmd_vel`), save with `ros2 run nav2_map_server map_saver_cli -f mymap`, pass `map:=` to `navigation.launch.py` (not part of the tested demo) |

Debug commands:

```bash
ros2 topic echo /diagnostics --field status         # health of streams and TF links
ros2 topic echo /cmd_vel_gate/state                 # gate state changes
ros2 run tf2_tools view_frames                      # TF tree PDF
ros2 topic echo /mission/decisions                  # decisions with reasons
ros2 run inspectloop_mission evaluate_run <run_dir> # compare with ground truth
```
