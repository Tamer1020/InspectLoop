// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#pragma once

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
struct GateInputs
{
  Velocity command;
  double command_age_s{1e9};
  bool scan_healthy{false};
  double min_front_range_m{1e9};   ///< closest obstacle in the forward sector
  bool estop{false};
};

enum class GateState { kPass, kLimited, kSlowed, kBlockedObstacle, kStoppedNoCommand,
  kStoppedScanFault, kStoppedEstop };

std::string toString(GateState s);

struct GateOutput
{
  Velocity velocity;
  GateState state{GateState::kStoppedNoCommand};
};

/// Pure function implementing the safety gate policy (priority order):
/// e-stop > LiDAR fault > stale command > forward obstacle > speed limiting.
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

struct ScanAssessment
{
  ScanValidity validity{ScanValidity::kEmpty};
  double front_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double usable_fraction{0.0};          ///< whole scan
  double forward_usable_fraction{0.0};  ///< beams with |angle| < 90 deg
};

/// Validate a scan and compute the forward clearance (Astra F3). Fail-closed:
///  * empty ranges, non-finite / non-positive increment, range_max <= range_min, or a ranges
///    count inconsistent with the angular span -> invalid;
///  * per REP-117: +inf = no return within range (clear), -inf = object closer than range_min
///    (obstacle at the sensor), NaN or out-of-[range_min, range_max] finite values = unusable;
///  * fewer than min_usable_fraction usable beams overall or in the forward half -> invalid.
ScanAssessment assessScan(const ScanData & scan, double corridor_half_width, double front_offset_m,
  double min_usable_fraction = 0.5);

enum class StampCheck { kOk, kZero, kFuture, kStale };
std::string toString(StampCheck c);

/// Plausibility of a message header stamp against the node clock (both seconds).
StampCheck checkStamp(double stamp_s, double now_s, double max_age_s, double max_future_s);

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
