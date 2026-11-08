# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `package 'inspectloop_…' not found` | workspace not sourced | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Mission aborts: `system not ready … missing /amcl_pose` | AMCL not active / initial pose not set | check `ros2 lifecycle get /amcl`; the initial pose comes from `nav2_params.yaml` (`set_initial_pose: true`). The mission subscribes transient-local, so a late start is fine. |
| `missing /gauge/read service` | `gauge_reader_node` crashed or not started | check its log; it wraps processing errors into `NO_IMAGE` results instead of dying |
| Every observation `NO_IMAGE` | camera bridge not running / no rendering | `ros2 topic hz /camera/image_raw`; headless mode needs `--headless-rendering` (set by the launch) and a working EGL/GPU driver |
| Gauge never visible (`TAG_NOT_VISIBLE`) where it should be | world/texture not found (`model://gauge_panel_*`) | `GZ_SIM_RESOURCE_PATH` is appended by `sim.launch.py`; after editing the layout regenerate the world (SETUP.md §4) |
| Readings biased / many `LOW_NEEDLE_CONTRAST` after editing textures | texture constants out of sync | `colcon test --packages-select inspectloop_vision inspectloop_gazebo` (constant ↔ texture tests) |
| Robot does not move, gate log `STOPPED_SCAN_FAULT` | no `/scan` for 0.5 s | `ros2 topic hz /scan`; LiDAR bridge `gz_lidar_bridge` must be running |
| Robot does not move, gate `STOPPED_ESTOP` | e-stop latched | `ros2 service call /cmd_vel_gate/set_estop std_srvs/srv/SetBool "{data: false}"` |
| Nav2 goals fail near racks | viewpoint inside inflation / dynamic obstacle | the planner already checks 0.45 m clearance; the mission retries, clears costmaps and then moves to another viewpoint |
| Processes left after Ctrl-C | crash during shutdown | `scripts/cleanup_sim.sh` (only matches InspectLoop command lines) |
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
