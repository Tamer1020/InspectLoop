// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#include "inspectloop_safety/gate_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace inspectloop_safety
{

GateSupervisor::GateSupervisor(const SupervisorParams & params)
: p_(params) {}

void GateSupervisor::onCommand(const Velocity & cmd, double steady_s)
{
  cmd_ = cmd;
  cmd_steady_ = steady_s;
}

bool GateSupervisor::detectClockJump(double ros_now_s)
{
  // any backward step of the ROS clock (simulation reset / replay) is a discontinuity: there is
  // no "small enough to ignore" rollback threshold, only a 1 ms tolerance for float noise
  const bool jump = last_ros_ && std::isfinite(ros_now_s) &&
    ros_now_s < *last_ros_ - p_.clock_jump_tolerance_s;
  if (std::isfinite(ros_now_s)) {
    last_ros_ = (!last_ros_ || jump) ? ros_now_s : std::max(*last_ros_, ros_now_s);
  }
  return jump;
}

void GateSupervisor::invalidateScan(const std::string & why)
{
  scan_ok_ = false;
  scan_reason_ = why;
  scan_steady_.reset();
  assessment_ = ScanAssessment{};
}

void GateSupervisor::onScan(const ScanData & scan, double stamp_s, double ros_now_s,
  double steady_s)
{
  if (detectClockJump(ros_now_s)) {
    pending_jump_ = true;
    invalidateScan("clock jumped backwards");
  }
  assessment_ = assessScan(scan, p_.coverage);
  const StampCheck sc = checkStamp(stamp_s, ros_now_s, p_.scan_max_stamp_age_s,
      p_.scan_max_future_s);
  scan_steady_ = steady_s;
  scan_ok_ = assessment_.validity == ScanValidity::kValid && sc == StampCheck::kOk;
  scan_reason_ = assessment_.validity != ScanValidity::kValid ?
    "scan " + toString(assessment_.validity) :
    (sc != StampCheck::kOk ? "scan stamp " + toString(sc) : "");
  if (!scan_ok_) {
    // nothing from an unhealthy scan may grant motion
    const ScanValidity v = assessment_.validity;
    assessment_ = ScanAssessment{};
    assessment_.validity = v;
  }
}

GateSupervisor::Result GateSupervisor::tick(double ros_now_s, double steady_s)
{
  Result res;
  if (detectClockJump(ros_now_s)) {
    invalidateScan("clock jumped backwards");
    res.clock_jump = true;
  }
  if (pending_jump_) {
    res.clock_jump = true;
    pending_jump_ = false;
  }
  std::string reason = scan_reason_;
  bool healthy = scan_ok_ && scan_steady_.has_value();
  if (!scan_steady_) {
    reason = reason.empty() ? "no scan" : reason;
  } else {
    const double age = steady_s - *scan_steady_;
    if (!std::isfinite(age) || age < 0.0 || age > p_.scan_timeout_s) {
      healthy = false;
      reason = "scan timeout";
    }
  }

  GateInputs in;
  in.command = cmd_;
  in.command_age_s = cmd_steady_ ? steady_s - *cmd_steady_ :
    std::numeric_limits<double>::infinity();
  in.scan_healthy = healthy;
  in.estop = estop_;
  if (healthy) {
    in.min_front_range_m = assessment_.front_clearance_m;
    in.forward_observed = assessment_.forward.observed;
    in.min_rear_range_m = assessment_.rear_clearance_m;
    in.rear_observed = assessment_.rear.observed;
    in.rotation_observed = assessment_.full.observed;
    in.rotation_obstacle = assessment_.rotation_obstacle;
  } else {
    in.min_front_range_m = std::numeric_limits<double>::quiet_NaN();
  }
  res.output = applyGate(in, p_.limits);
  if (res.output.state == GateState::kBlockedUnobserved) {
    std::string r;
    if (!in.forward_observed) {
      r += "forward sector unobserved";
    }
    if (!in.rear_observed) {
      r += std::string(r.empty() ? "" : ", ") + "rear sector unobserved";
    }
    if (!in.rotation_observed) {
      r += std::string(r.empty() ? "" : ", ") + "full circle unobserved";
    }
    reason = r;
  }
  res.reason = reason;
  return res;
}

}  // namespace inspectloop_safety
