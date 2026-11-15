V3.1 live evidence (2026-11-15, headless Gazebo Harmonic + Nav2, ROS 2 Jazzy).
Produced on the V3.1 working tree (F3 bounded sector coverage + F4 register-before-send),
BEFORE the V3.2 goal-id cancellation fallback. Preserved from a temporary directory.
  demo/           normal mission: COMPLETED, launch exit 0 (PG-11 4.19, PG-12 6.81, PG-13 UNRESOLVED,
                  PG-14 8.71 alarm HIGH)
  f3_tiny_inc/    720-beam scan, angle_increment 1e-9, injected while driving: gate
                  STOPPED_SCAN_FAULT (scan BAD_METADATA), robot 0 m/s, /cmd_vel_safe max gap 54 ms
  f4_dispatch_1/  SIGINT right after goal #1 SENT: CANCEL_CONFIRMED, Nav2 status 5, robot 0 m/s
  f4_dispatch_3/  SIGINT right after goal #3 SENT: CANCEL_CONFIRMED, Nav2 statuses 4,4,5, robot 0 m/s
Harness scripts: ../astra_v3_2/tools/ (tiny_inc_fault.sh, sigint_at_dispatch.sh).

Before-fix reproductions (regenerated 2026-11-15 against git HEAD e9a12e6 = V3):
  before_fix_f4_send_registration_gap.txt  V3.1 F4 tests on V3 nav_goal.py: 7 of 9 fail
                                           ("false clean shutdown after SIGINT at run:272")
  before_fix_f3_unbounded_sector_loop.txt  V3 sectorCoverage on the 1e-9 scan: 20.19 s for one
                                           sector; assessScan did not return within 30 s
  before_fix_f3_probe.cpp                  the probe program used for the F3 timing
V3.2 before-fix equivalent: ../astra_v3_2/live_unknown_goal/nofallback.txt (live control run).
