# One observation of an analog gauge: a burst of camera frames taken from one viewpoint,
# each read independently, then fused. Produced by inspectloop_vision from real simulated frames.

builtin_interfaces/Time stamp           # stamp of the last frame used (sim time)
string gauge_id
int32 tag_id                            # ArUco id expected on the gauge panel
string viewpoint_label                  # e.g. "V0 d=1.50 a=+0deg"
string observation_id                   # copied from the request (Astra F5)
uint32 viewpoint_epoch                  # copied from the request
builtin_interfaces/Time request_stamp   # node time when the request was received
builtin_interfaces/Time first_frame_stamp
# robot pose (map -> base_footprint) at the frames' capture times, from TF
bool pose_known
float32 acq_x
float32 acq_y
float32 acq_yaw
float32 pose_motion_m                   # max displacement of the robot during the burst
float32 pose_motion_rad

# --- fused measurement ----------------------------------------------------------
bool has_value                          # false -> value/sigma are NaN
float32 value                           # median of the valid per-frame readings
float32 sigma                           # HEURISTIC 1-sigma uncertainty (not a calibrated probability)
string unit

# --- evidence / quality metrics ----------------------------------------------
uint32 frames_requested
uint32 frames_received
uint32 frames_tag_detected
uint32 frames_valid                     # frames that produced a needle reading
float32[] frame_values                  # per-frame readings (valid frames only)
float32 frame_spread                    # robust spread (1.4826*MAD, or |a-b| for 2) of frame_values
float32 needle_confidence               # median needle peak quality [0..1]
float32 dial_visibility                 # median fraction of the dial face that is unobstructed [0..1]
float32 view_angle_deg                  # angle between camera ray and dial normal (solvePnP)
float32 distance_m                      # camera -> tag distance (solvePnP)
float32 dial_radius_px                  # apparent dial radius in the image (resolution)
float32 sharpness                       # variance of Laplacian over the dial region
float32 processing_time_ms

# Quality flags (any subset): NO_IMAGE, TAG_NOT_VISIBLE, TAG_INTERMITTENT, OCCLUDED,
# OBLIQUE_VIEW, TOO_FAR, LOW_NEEDLE_CONTRAST, AMBIGUOUS_NEEDLE, NEEDLE_PARTIAL,
# INCONSISTENT_FRAMES, BLURRED, OUT_OF_SCALE, FROZEN_FRAMES, CLOCK_RESET, ROBOT_MOVING,
# WRONG_VIEWPOINT, STALE_OBSERVATION
string[] quality_flags

string raw_image_path
string annotated_image_path
string rectified_image_path
string detail
