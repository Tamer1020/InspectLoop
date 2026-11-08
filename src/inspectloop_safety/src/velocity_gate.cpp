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
  // fail closed: a negative or non-finite age means the freshness is unknown
  if (!std::isfinite(in.command_age_s) || in.command_age_s < 0.0 ||
    in.command_age_s > lim.cmd_timeout_s || !std::isfinite(in.command.linear) ||
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
    if (std::isnan(in.min_front_range_m) || in.min_front_range_m <= lim.stop_distance_m) {
      // unknown clearance is treated like an obstacle
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
  std::size_t forward_usable = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    if (!std::isfinite(a) || std::cos(a) <= 0.0) {
      continue;                                     // only the forward half matters here
    }
    const double r = ranges[i];
    if (std::isnan(r)) {
      continue;
    }
    if (std::isinf(r)) {
      ++forward_usable;
      if (r < 0.0) {
        best = 0.0;                                  // -inf: object closer than range_min
      }
      continue;                                      // +inf: no return, clear
    }
    if (r < range_min || r > range_max) {
      continue;
    }
    ++forward_usable;
    const double x = r * std::cos(a);
    const double y = r * std::sin(a);
    if (std::fabs(y) > corridor_half_width) {
      continue;
    }
    best = std::min(best, std::max(0.0, x - front_offset_m));
  }
  if (forward_usable == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best;
}

std::string toString(ScanValidity v)
{
  switch (v) {
    case ScanValidity::kValid: return "VALID";
    case ScanValidity::kEmpty: return "EMPTY";
    case ScanValidity::kBadMetadata: return "BAD_METADATA";
    case ScanValidity::kInconsistentSize: return "INCONSISTENT_SIZE";
    case ScanValidity::kTooManyInvalid: return "TOO_MANY_INVALID_BEAMS";
  }
  return "UNKNOWN";
}

ScanAssessment assessScan(const ScanData & s, double corridor_half_width, double front_offset_m,
  double min_usable_fraction)
{
  ScanAssessment out;
  if (s.ranges.empty()) {
    out.validity = ScanValidity::kEmpty;
    return out;
  }
  if (!std::isfinite(s.angle_min) || !std::isfinite(s.angle_max) ||
    !std::isfinite(s.angle_increment) || s.angle_increment <= 0.0 ||
    !std::isfinite(s.range_min) || !std::isfinite(s.range_max) || s.range_min < 0.0 ||
    s.range_max <= s.range_min || s.angle_max < s.angle_min)
  {
    out.validity = ScanValidity::kBadMetadata;
    return out;
  }
  const double expected = (s.angle_max - s.angle_min) / s.angle_increment + 1.0;
  if (std::fabs(expected - static_cast<double>(s.ranges.size())) > 1.5) {
    out.validity = ScanValidity::kInconsistentSize;
    return out;
  }
  std::size_t usable = 0, fwd = 0, fwd_usable = 0;
  for (std::size_t i = 0; i < s.ranges.size(); ++i) {
    const double r = s.ranges[i];
    const bool ok = std::isinf(r) || (std::isfinite(r) && r >= s.range_min && r <= s.range_max);
    usable += ok;
    const double a = s.angle_min + static_cast<double>(i) * s.angle_increment;
    if (std::cos(a) > 0.0) {
      ++fwd;
      fwd_usable += ok;
    }
  }
  out.usable_fraction = static_cast<double>(usable) / static_cast<double>(s.ranges.size());
  out.forward_usable_fraction = fwd ? static_cast<double>(fwd_usable) / static_cast<double>(fwd) :
    0.0;
  if (out.usable_fraction < min_usable_fraction || out.forward_usable_fraction < min_usable_fraction) {
    out.validity = ScanValidity::kTooManyInvalid;
    return out;
  }
  out.front_clearance_m = frontClearance(s.ranges, s.angle_min, s.angle_increment, s.range_min,
      s.range_max, corridor_half_width, front_offset_m);
  out.validity = std::isnan(out.front_clearance_m) ? ScanValidity::kTooManyInvalid :
    ScanValidity::kValid;
  return out;
}

std::string toString(StampCheck c)
{
  switch (c) {
    case StampCheck::kOk: return "OK";
    case StampCheck::kZero: return "ZERO_OR_INVALID";
    case StampCheck::kFuture: return "FUTURE";
    case StampCheck::kStale: return "STALE";
  }
  return "UNKNOWN";
}

StampCheck checkStamp(double stamp_s, double now_s, double max_age_s, double max_future_s)
{
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || !std::isfinite(now_s)) {
    return StampCheck::kZero;
  }
  if (stamp_s > now_s + max_future_s) {
    return StampCheck::kFuture;
  }
  if (now_s - stamp_s > max_age_s) {
    return StampCheck::kStale;
  }
  return StampCheck::kOk;
}

}  // namespace inspectloop_safety
