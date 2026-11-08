// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>

#include "inspectloop_safety/velocity_gate.hpp"

namespace inspectloop_safety
{

struct SupervisorParams
{
  GateLimits limits;
  CoverageParams coverage;
  double scan_timeout_s{0.5};          ///< steady (monotonic) time since the last scan
  double scan_max_stamp_age_s{1.0};    ///< ROS-time age of the scan header stamp
  double scan_max_future_s{0.2};       ///< header stamp ahead of the ROS clock
  double clock_jump_tolerance_s{1e-3};  ///< any larger backward ROS-clock step = discontinuity
};

/// The cmd_vel_gate decision logic without ROS (V3, Astra V2 review F3): the node only forwards
/// messages and clock readings, so liveness, stamp plausibility, clock discontinuities, coverage
/// and the command watchdog are unit tested exactly as they run.
///
/// Time bases: `steady_s` (monotonic) for liveness and the command deadman - immune to sim-clock
/// pauses and resets; `ros_now_s` (ROS / sim time) only for header-stamp plausibility.
class GateSupervisor
{
public:
  explicit GateSupervisor(const SupervisorParams & params);

  void onCommand(const Velocity & cmd, double steady_s);
  void onScan(const ScanData & scan, double stamp_s, double ros_now_s, double steady_s);
  void setEstop(bool on) {estop_ = on;}
  bool estop() const {return estop_;}

  struct Result
  {
    GateOutput output;
    std::string reason;       ///< why the scan is unhealthy / which motion is unobserved
    bool clock_jump{false};   ///< a backward ROS-clock step was detected on this tick
  };
  Result tick(double ros_now_s, double steady_s);

  const ScanAssessment & lastAssessment() const {return assessment_;}
  const SupervisorParams & params() const {return p_;}

private:
  bool detectClockJump(double ros_now_s);
  void invalidateScan(const std::string & why);

  SupervisorParams p_;
  Velocity cmd_;
  std::optional<double> cmd_steady_;
  std::optional<double> scan_steady_;
  std::optional<double> last_ros_;
  ScanAssessment assessment_;
  bool scan_ok_{false};
  bool pending_jump_{false};
  std::string scan_reason_{"no scan"};
  bool estop_{false};
};

}  // namespace inspectloop_safety
