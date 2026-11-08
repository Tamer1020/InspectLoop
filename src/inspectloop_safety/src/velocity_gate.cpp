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
    case GateState::kBlockedUnobserved: return "BLOCKED_UNOBSERVED";
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

  bool limited = false, slowed = false, blocked = false, unobserved = false;
  double v = std::clamp(in.command.linear, -lim.max_linear, lim.max_linear);
  double w = std::clamp(in.command.angular, -lim.max_angular, lim.max_angular);
  limited = v != in.command.linear || w != in.command.angular;

  // Linear motion: the corridor swept in the direction of travel must be observed (V3, F3) and
  // free. Unknown clearance (NaN) is treated like an obstacle.
  auto corridor = [&](bool observed, double clearance) {
      if (!observed) {
        unobserved = true;
        return 0.0;
      }
      if (std::isnan(clearance) || clearance <= lim.stop_distance_m) {
        blocked = true;
        return 0.0;
      }
      if (clearance < lim.slow_distance_m && lim.slow_distance_m > lim.stop_distance_m) {
        slowed = true;
        return std::clamp((clearance - lim.stop_distance_m) /
                 (lim.slow_distance_m - lim.stop_distance_m), 0.0, 1.0);
      }
      return 1.0;
    };
  if (v > 0.0) {
    v *= corridor(in.forward_observed, in.min_front_range_m);
  } else if (v < 0.0) {
    v *= corridor(in.rear_observed, in.min_rear_range_m);
  }
  // Rotation sweeps the whole footprint circle: it needs whole-circle coverage and no obstacle
  // inside the sweep radius. Linear motion that is still allowed is not affected.
  if (w != 0.0) {
    if (!in.rotation_observed) {
      unobserved = true;
      w = 0.0;
    } else if (in.rotation_obstacle) {
      blocked = true;
      w = 0.0;
    }
  }
  out.state = unobserved ? GateState::kBlockedUnobserved :
    blocked ? GateState::kBlockedObstacle :
    slowed ? GateState::kSlowed :
    limited ? GateState::kLimited : GateState::kPass;
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

double rearClearance(const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double corridor_half_width, double rear_offset_m)
{
  double best = std::numeric_limits<double>::infinity();
  std::size_t rear_usable = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    if (!std::isfinite(a) || std::cos(a) >= 0.0) {
      continue;                                     // only the rear half
    }
    const double r = ranges[i];
    if (std::isnan(r)) {
      continue;
    }
    if (std::isinf(r)) {
      ++rear_usable;
      if (r < 0.0) {
        best = 0.0;
      }
      continue;
    }
    if (r < range_min || r > range_max) {
      continue;
    }
    ++rear_usable;
    const double x = r * std::cos(a);
    const double y = r * std::sin(a);
    if (std::fabs(y) > corridor_half_width) {
      continue;
    }
    best = std::min(best, std::max(0.0, -x - rear_offset_m));
  }
  if (rear_usable == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return best;
}

double CoverageParams::forwardHalfAngle() const
{
  return forward_half_angle_rad > 0.0 ? forward_half_angle_rad :
         std::atan2(corridor_half_width_m, std::max(front_offset_m, 1e-3));
}

double CoverageParams::rearHalfAngle() const
{
  return rear_half_angle_rad > 0.0 ? rear_half_angle_rad :
         std::atan2(corridor_half_width_m, std::max(rear_offset_m, 1e-3));
}

namespace
{
bool beamObserved(double r, double range_min, double range_max)
{
  return std::isinf(r) || (std::isfinite(r) && r >= range_min && r <= range_max);
}

bool structurallyValid(const ScanData & s)
{
  if (s.ranges.empty() || !std::isfinite(s.angle_min) || !std::isfinite(s.angle_max) ||
    !std::isfinite(s.angle_increment) || s.angle_increment <= 0.0 ||
    !std::isfinite(s.range_min) || !std::isfinite(s.range_max) || s.range_min < 0.0 ||
    s.range_max <= s.range_min || s.angle_max < s.angle_min)
  {
    return false;
  }
  const double expected = (s.angle_max - s.angle_min) / s.angle_increment + 1.0;
  return std::fabs(expected - static_cast<double>(s.ranges.size())) <= 1.5;
}

/// index of the beam sampling direction theta, or -1 if the scan does not sample it
long beamIndex(const ScanData & s, double theta)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  const long n = static_cast<long>(s.ranges.size());
  for (const double wrap : {0.0, kTwoPi, -kTwoPi}) {
    const double t = theta + wrap;
    const long idx = std::lround((t - s.angle_min) / s.angle_increment);
    if (idx >= 0 && idx < n &&
      std::fabs(s.angle_min + static_cast<double>(idx) * s.angle_increment - t) <=
      0.5 * s.angle_increment + 1e-9)
    {
      return idx;
    }
  }
  return -1;
}
}  // namespace

SectorCoverage sectorCoverage(const ScanData & s, double centre, double half_angle,
  double min_coverage, double max_gap_rad, double max_increment_rad)
{
  SectorCoverage out;
  if (!structurallyValid(s) || !std::isfinite(centre) || !std::isfinite(half_angle) ||
    half_angle <= 0.0)
  {
    return out;
  }
  const double inc = s.angle_increment;
  const bool full_circle = half_angle >= M_PI - 1e-9;
  const long samples = full_circle ?
    std::max(1L, std::lround(2.0 * M_PI / inc)) :
    2L * static_cast<long>(std::floor(half_angle / inc)) + 1L;
  const double start = full_circle ? centre - M_PI : centre - inc * static_cast<double>(
    (samples - 1) / 2);
  long observed = 0, run = 0, longest = 0, leading = -1;
  for (long k = 0; k < samples; ++k) {
    const long idx = beamIndex(s, start + static_cast<double>(k) * inc);
    const bool ok = idx >= 0 && beamObserved(s.ranges[static_cast<std::size_t>(idx)],
        s.range_min, s.range_max);
    if (ok) {
      ++observed;
      if (leading < 0) {
        leading = run;          // unobserved run at the start (joins the end on a full circle)
      }
      longest = std::max(longest, run);
      run = 0;
    } else {
      ++run;
    }
  }
  if (leading < 0) {            // nothing observed at all
    longest = samples;
  } else if (full_circle) {
    longest = std::max(longest, run + leading);
  } else {
    longest = std::max(longest, run);
  }
  out.coverage = static_cast<double>(observed) / static_cast<double>(samples);
  out.max_gap_rad = static_cast<double>(longest) * inc;
  out.observed = inc <= max_increment_rad + 1e-12 && out.coverage >= min_coverage &&
    out.max_gap_rad <= max_gap_rad + 1e-9;
  return out;
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

ScanAssessment assessScan(const ScanData & s, const CoverageParams & p)
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
  if (!structurallyValid(s)) {
    out.validity = ScanValidity::kInconsistentSize;
    return out;
  }
  std::size_t usable = 0, fwd = 0, fwd_usable = 0;
  for (std::size_t i = 0; i < s.ranges.size(); ++i) {
    const double r = s.ranges[i];
    const bool ok = beamObserved(r, s.range_min, s.range_max);
    usable += ok;
    const double a = s.angle_min + static_cast<double>(i) * s.angle_increment;
    if (std::cos(a) > 0.0) {
      ++fwd;
      fwd_usable += ok;
    }
    // rotation sweep: anything inside the footprint's turning circle
    if (ok && !(std::isinf(r) && r > 0.0)) {
      const double rr = std::isinf(r) ? 0.0 : r;     // -inf: object at the sensor
      const double bx = p.sensor_x_m + rr * std::cos(a);
      const double by = rr * std::sin(a);
      if (std::hypot(bx, by) < p.rotation_radius_m) {
        out.rotation_obstacle = true;
      }
    }
  }
  out.usable_fraction = static_cast<double>(usable) / static_cast<double>(s.ranges.size());
  out.forward_usable_fraction = fwd ? static_cast<double>(fwd_usable) / static_cast<double>(fwd) :
    0.0;
  out.forward = sectorCoverage(s, 0.0, p.forwardHalfAngle(), p.min_sector_coverage,
      p.max_sector_gap_rad, p.max_angle_increment_rad);
  out.rear = sectorCoverage(s, M_PI, p.rearHalfAngle(), p.min_sector_coverage,
      p.max_sector_gap_rad, p.max_angle_increment_rad);
  out.full = sectorCoverage(s, 0.0, M_PI, p.min_sector_coverage, p.max_sector_gap_rad,
      p.max_angle_increment_rad);
  if (out.usable_fraction < p.min_usable_fraction ||
    out.forward_usable_fraction < p.min_usable_fraction)
  {
    out.validity = ScanValidity::kTooManyInvalid;
    return out;
  }
  out.front_clearance_m = frontClearance(s.ranges, s.angle_min, s.angle_increment, s.range_min,
      s.range_max, p.corridor_half_width_m, p.front_offset_m);
  out.rear_clearance_m = rearClearance(s.ranges, s.angle_min, s.angle_increment, s.range_min,
      s.range_max, p.corridor_half_width_m, p.rear_offset_m);
  out.validity = std::isnan(out.front_clearance_m) ? ScanValidity::kTooManyInvalid :
    ScanValidity::kValid;
  return out;
}

ScanAssessment assessScan(const ScanData & s, double corridor_half_width, double front_offset_m,
  double min_usable_fraction)
{
  CoverageParams p;
  p.corridor_half_width_m = corridor_half_width;
  p.front_offset_m = front_offset_m;
  p.min_usable_fraction = min_usable_fraction;
  return assessScan(s, p);
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
