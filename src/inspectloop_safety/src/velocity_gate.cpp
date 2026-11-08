// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#include "inspectloop_safety/velocity_gate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace inspectloop_safety
{

std::string toString(GateState s)
{
  switch (s) {
    case GateState::kPass: return "PASS";
    case GateState::kLimited: return "LIMITED";
    case GateState::kSlowed: return "SLOWED";
    case GateState::kBlockedObstacle: return "BLOCKED_OBSTACLE";
    case GateState::kStoppedNoCommand: return "STOPPED_NO_COMMAND";
    case GateState::kStoppedScanFault: return "STOPPED_SCAN_FAULT";
    case GateState::kStoppedEstop: return "STOPPED_ESTOP";
  }
  return "UNKNOWN";
}

GateOutput applyGate(const GateInputs & in, const GateLimits & lim)
{
  GateOutput out;
  if (in.estop) {
    out.state = GateState::kStoppedEstop;
    return out;
  }
  if (!in.scan_healthy) {
    out.state = GateState::kStoppedScanFault;
    return out;
  }
  if (in.command_age_s > lim.cmd_timeout_s || !std::isfinite(in.command.linear) ||
    !std::isfinite(in.command.angular))
  {
    out.state = GateState::kStoppedNoCommand;
    return out;
  }

  out.state = GateState::kPass;
  double v = std::clamp(in.command.linear, -lim.max_linear, lim.max_linear);
  double w = std::clamp(in.command.angular, -lim.max_angular, lim.max_angular);
  if (v != in.command.linear || w != in.command.angular) {
    out.state = GateState::kLimited;
  }

  if (v > 0.0) {
    if (in.min_front_range_m <= lim.stop_distance_m) {
      v = 0.0;  // rotation stays allowed so the planner can turn away
      out.state = GateState::kBlockedObstacle;
    } else if (in.min_front_range_m < lim.slow_distance_m &&
      lim.slow_distance_m > lim.stop_distance_m)
    {
      const double scale = (in.min_front_range_m - lim.stop_distance_m) /
        (lim.slow_distance_m - lim.stop_distance_m);
      v *= std::clamp(scale, 0.0, 1.0);
      out.state = GateState::kSlowed;
    }
  }
  out.velocity = {v, w};
  return out;
}

double frontClearance(const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double corridor_half_width, double front_offset_m)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    const double x = r * std::cos(a);
    const double y = r * std::sin(a);
    if (x <= 0.0 || std::fabs(y) > corridor_half_width) {
      continue;
    }
    best = std::min(best, std::max(0.0, x - front_offset_m));
  }
  return best;
}

}  // namespace inspectloop_safety
