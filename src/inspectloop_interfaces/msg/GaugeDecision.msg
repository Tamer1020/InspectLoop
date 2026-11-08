# Decision taken by the mission manager after one gauge observation.
builtin_interfaces/Time stamp
string gauge_id
uint32 observation_index                # 1-based, per gauge
string viewpoint_label

uint8 ACCEPT = 0                        # reading is reliable -> record it
uint8 RETRY_SAME = 1                    # likely transient problem -> observe again from the same pose
uint8 MOVE_NEW = 2                      # geometric problem (occlusion/angle/range) -> new viewpoint via Nav2
uint8 UNRESOLVED = 3                    # no reliable reading possible within the budget -> report honestly
uint8 decision
string decision_label
string reason

bool has_value
float32 value
float32 sigma
string[] quality_flags
