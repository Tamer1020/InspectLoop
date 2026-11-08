// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#pragma once

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

/// Free distance ahead of the robot's front edge, from a laser scan.
///
/// Only points inside the corridor swept by the robot when driving straight
/// (|y| <= corridor_half_width, x > 0 in the sensor frame) are considered.
/// The sensor is assumed to look along +x of the robot; front_offset_m is the
/// distance from the sensor origin to the front edge. Invalid readings (NaN,
/// inf, outside [range_min, range_max]) are ignored. Returns +inf when clear.
double frontClearance(const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double corridor_half_width, double front_offset_m);

}  // namespace inspectloop_safety
