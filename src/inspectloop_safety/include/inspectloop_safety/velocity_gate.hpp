// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace inspectloop_safety
{

struct Velocity
{
  double linear{0.0};
  double angular{0.0};
};

struct GateLimits
{
  double max_linear{0.5};          ///< |v| limit [m/s]
  double max_angular{1.5};         ///< |w| limit [rad/s]
  double cmd_timeout_s{0.5};       ///< deadman: zero output when the last command is older
  double stop_distance_m{0.30};    ///< block forward motion if an obstacle is closer (from LiDAR)
  double slow_distance_m{0.60};    ///< linearly scale forward speed down inside this range
};

/// Inputs evaluated on every gate cycle.
///
/// V3 (Astra V2 review F3): every motion direction needs positive evidence that the region it
/// sweeps is observed. The *_observed flags default to false (fail closed): a caller that does
/// not establish coverage cannot obtain motion in that direction.
struct GateInputs
{
  Velocity command;
  double command_age_s{1e9};
  bool scan_healthy{false};
  double min_front_range_m{1e9};   ///< closest obstacle in the forward corridor
  bool forward_observed{false};    ///< forward stopping-corridor sector sufficiently covered
  double min_rear_range_m{std::numeric_limits<double>::quiet_NaN()};  ///< rear corridor
  bool rear_observed{false};       ///< rear corridor sector sufficiently covered
  bool rotation_observed{false};   ///< full circle sufficiently covered for an in-place turn
  bool rotation_obstacle{false};   ///< an obstacle inside the rotation sweep circle
  bool estop{false};
};

enum class GateState { kPass, kLimited, kSlowed, kBlockedObstacle, kStoppedNoCommand,
  kStoppedScanFault, kStoppedEstop, kBlockedUnobserved };

std::string toString(GateState s);

struct GateOutput
{
  Velocity velocity;
  GateState state{GateState::kStoppedNoCommand};
};

/// Pure function implementing the safety gate policy (priority order):
/// e-stop > LiDAR fault > stale command > per-direction coverage (unobserved -> that motion
/// component is zeroed) > obstacle in the swept corridor > speed limiting.
GateOutput applyGate(const GateInputs & in, const GateLimits & limits);

/// Minimal LaserScan view (ROS independent).
struct ScanData
{
  std::vector<float> ranges;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

enum class ScanValidity { kValid, kEmpty, kBadMetadata, kInconsistentSize, kTooManyInvalid };
std::string toString(ScanValidity v);

/// Coverage of one angular sector of the scan.
/// A beam "observes" its ray if it is a finite reading in [range_min, range_max] or +/-inf
/// (REP-117: +inf = no return up to range_max, i.e. observed free up to range_max; -inf = object
/// closer than range_min). NaN, out-of-range finite values, and directions the scan does not
/// sample at all (narrow field of view) are UNOBSERVED - never free space.
struct SectorCoverage
{
  double coverage{0.0};    ///< observed sample directions / all sample directions in the sector
  double max_gap_rad{std::numeric_limits<double>::infinity()};  ///< widest unobserved run
  bool observed{false};    ///< coverage >= min and max gap <= limit (and resolution ok)
};

/// Geometry and coverage thresholds of the motion-relevant sectors (all configurable).
struct CoverageParams
{
  double corridor_half_width_m{0.27};  ///< half robot width + margin
  double front_offset_m{0.15};         ///< sensor origin -> front edge (along +x)
  double rear_offset_m{0.35};          ///< sensor origin -> rear edge (along -x)
  double sensor_x_m{0.10};             ///< sensor x in the base frame (rotation centre at 0)
  double rotation_radius_m{0.35};      ///< swept radius of the footprint when turning in place
  /// half-angles of the forward / rear sectors; <= 0 -> derived from the corridor geometry
  /// (atan2(half width, edge offset): rays outside it cannot enter the corridor ahead of the edge)
  double forward_half_angle_rad{0.0};
  double rear_half_angle_rad{0.0};
  double min_usable_fraction{0.5};     ///< whole-scan structural plausibility
  double min_sector_coverage{0.95};    ///< per motion sector
  double max_sector_gap_rad{0.03490658503988659};      ///< 2 deg: widest tolerated unobserved run
  double max_angle_increment_rad{0.017453292519943295};  ///< 1 deg: coarser scans cannot protect
  double forwardHalfAngle() const;
  double rearHalfAngle() const;
};

struct ScanAssessment
{
  ScanValidity validity{ScanValidity::kEmpty};
  double front_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double rear_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double usable_fraction{0.0};          ///< whole scan
  double forward_usable_fraction{0.0};  ///< beams with |angle| < 90 deg (V2 metric, kept)
  SectorCoverage forward;               ///< forward stopping-corridor sector
  SectorCoverage rear;                  ///< rear corridor sector (reverse motion)
  SectorCoverage full;                  ///< whole circle (in-place rotation)
  bool rotation_obstacle{false};        ///< obstacle inside rotation_radius_m of the base centre
};

/// Coverage of the sector [centre - half_angle, centre + half_angle] (radians, scan frame).
/// The sector is sampled at the scan's angular increment; wrap-around is handled for 360 deg
/// scans. Returns observed=false for structurally unusable scans.
SectorCoverage sectorCoverage(const ScanData & scan, double centre, double half_angle,
  double min_coverage, double max_gap_rad, double max_increment_rad);

/// Full V3 assessment: structure, whole-scan usable fraction, per-sector coverage, forward and
/// rear corridor clearance and the rotation-sweep obstacle check.
ScanAssessment assessScan(const ScanData & scan, const CoverageParams & params);

/// Validate a scan and compute the forward clearance (Astra F3). Fail-closed:
///  * empty ranges, non-finite / non-positive increment, range_max <= range_min, or a ranges
///    count inconsistent with the angular span -> invalid;
///  * per REP-117: +inf = no return within range (clear), -inf = object closer than range_min
///    (obstacle at the sensor), NaN or out-of-[range_min, range_max] finite values = unusable;
///  * fewer than min_usable_fraction usable beams overall or in the forward half -> invalid.
/// V2-compatible overload: default CoverageParams with the given corridor / offset / fraction.
ScanAssessment assessScan(const ScanData & scan, double corridor_half_width, double front_offset_m,
  double min_usable_fraction = 0.5);

enum class StampCheck { kOk, kZero, kFuture, kStale };
std::string toString(StampCheck c);

/// Plausibility of a message header stamp against the node clock (both seconds).
StampCheck checkStamp(double stamp_s, double now_s, double max_age_s, double max_future_s);

/// Free distance behind the robot's rear edge (mirror of frontClearance for reverse motion).
double rearClearance(const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double corridor_half_width, double rear_offset_m);

/// Free distance ahead of the robot's front edge, from a laser scan.
///
/// Only points inside the corridor swept by the robot when driving straight
/// (|y| <= corridor_half_width, x > 0 in the sensor frame) are considered.
/// The sensor is assumed to look along +x of the robot; front_offset_m is the
/// distance from the sensor origin to the front edge. Invalid readings (NaN,
/// inf, outside [range_min, range_max]) are ignored. Returns +inf when clear and NaN when the
/// forward half contains no usable beam at all (unknown is never reported as free space).
double frontClearance(const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double corridor_half_width, double front_offset_m);

}  // namespace inspectloop_safety
