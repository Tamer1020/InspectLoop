# Periodic mission progress published by inspectloop_mission.
builtin_interfaces/Time stamp
string mission_id
string state                 # IDLE | WAITING_FOR_SYSTEM | NAVIGATING | OBSERVING | RETURNING | COMPLETED | ABORTED
string current_gauge
string current_viewpoint
uint32 current_observation
uint32 gauges_total
uint32 gauges_done
uint32 gauges_accepted
uint32 gauges_unresolved
string last_decision
string detail
