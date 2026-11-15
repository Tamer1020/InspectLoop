V3.2 F4 hardening - live evidence (2026-11-15, headless Gazebo Harmonic gz-sim 8.15.0 + Nav2,
ROS 2 Jazzy, WSL2). Code: V3.1 + goal-id cancellation fallback for unknown-ownership goals.

live_unknown_goal/   tools/live_unknown_goal.sh (stack with mission:=false + harness)
  action_info.txt    /navigate_to_pose clients: bt_navigator, waypoint_follower, docking_server
                     -> the server is shared; cancel-all would cancel other clients' goals
  fallback.txt       SIGINT injected after send_goal_async returned, before its future was
                     stored (ownership UNKNOWN). Events OWNERSHIP_UNKNOWN -> UNKNOWN_GOAL_ACTIVE
                     (Nav2 listed our id EXECUTING) -> UNKNOWN_RESOLVED (cancel-by-id ERROR_NONE,
                     server status CANCELED). Robot 0.000 m/s throughout; cancel_all() True.
  nofallback.txt     Control, V3.1 behaviour: same injection, no fallback. The orphan goal stayed
                     EXECUTING and drove the robot at up to 0.40 m/s for ~13 s until Nav2 reported
                     SUCCEEDED; cancel_all() False at every attempt (fail closed, not stopped).
sigint_dispatch_1/   tools/sigint_at_dispatch.sh: SIGINT right after goal #1 SENT. CANCEL_CONFIRMED
                     (ERROR_NONE, CANCELED); Nav2 status list: 1x CANCELED; robot 0.000 m/s 10 s.
sigint_dispatch_3/   SIGINT right after goal #3 SENT (PG-12/V3): LATE_ACCEPTANCE -> CANCEL_CONFIRMED;
                     Nav2: 2x SUCCEEDED, 1x CANCELED; robot 0.000 m/s 10 s.
demo/                Normal mission: COMPLETED, launch exit 0, nav_cleanup_confirmed=true
                     (evaluation.json from `evaluate_run`: max |err| 0.012 bar, 0 incorrect).
                     PG-11 4.21 bar, PG-12 6.80 bar, PG-13 UNRESOLVED, PG-14 8.70 bar alarm HIGH.

Notes
* Harness teardown: the backgrounded `ros2 launch` did not exit on SIGINT within 30 s and was
  SIGTERMed; 0 leftover processes in every run. Nav2's container aborts (-6) during
  launch shutdown after the report is written (pre-existing, see KNOWN_LIMITATIONS.md).
* The exact send/registration window cannot be hit with a real signal; the harness uses the
  same deterministic sys.settrace injection as the unit tests.
* Not exercised live: gate e-stop service failure, gate restart, request lost before reaching
  Nav2 (unit-tested only: stays unresolved / inhibited).

colcon_test_result.txt  all 8 packages: 500 results, 0 errors, 0 failures, 0 skipped.
tools/                  harness scripts used for every live run above.
