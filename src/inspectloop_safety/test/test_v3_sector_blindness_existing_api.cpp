// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
// Astra V2 review, F3 (HIGH): forward-sector blindness. Written against the V2 API before the
// fix; it must fail on V2 (forward command passes) and pass on V3.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "inspectloop_safety/velocity_gate.hpp"

using namespace inspectloop_safety;  // NOLINT

TEST(AstraV3F3, ForwardSectorNaNBlindnessDoesNotPassForwardMotion)
{
  // exact review scenario: 720 beams, valid 5 m returns outside +-35 deg, NaN inside
  ScanData s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / 720.0;
  s.angle_max = s.angle_min + s.angle_increment * 719.0;
  s.range_min = 0.12;
  s.range_max = 12.0;
  s.ranges.assign(720, 5.0f);
  for (std::size_t i = 0; i < 720; ++i) {
    const double a = s.angle_min + static_cast<double>(i) * s.angle_increment;
    if (std::fabs(a) <= 35.0 * M_PI / 180.0) {
      s.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  const ScanAssessment a = assessScan(s, 0.27, 0.15, 0.5);
  GateInputs in;
  in.command = {0.4, 0.2};
  in.command_age_s = 0.05;
  in.scan_healthy = a.validity == ScanValidity::kValid;
  in.min_front_range_m = a.front_clearance_m;
  const GateOutput out = applyGate(in, GateLimits{});
  std::printf("forward_usable=%.6f validity=%s clearance=%f state=%s out=(%.2f, %.2f)\n",
    a.forward_usable_fraction, toString(a.validity).c_str(), a.front_clearance_m,
    toString(out.state).c_str(), out.velocity.linear, out.velocity.angular);
  EXPECT_LE(out.velocity.linear, 0.0) << "forward motion passed with an unobserved front sector";
}
