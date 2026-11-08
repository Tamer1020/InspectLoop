// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "inspectloop_safety/velocity_gate.hpp"

using inspectloop_safety::applyGate;
using inspectloop_safety::frontClearance;
using inspectloop_safety::GateInputs;
using inspectloop_safety::GateLimits;
using inspectloop_safety::GateState;

namespace
{
GateInputs nominal(double v, double w)
{
  GateInputs in;
  in.command = {v, w};
  in.command_age_s = 0.05;
  in.scan_healthy = true;
  in.min_front_range_m = 5.0;
  // V3: coverage is explicit (fail-closed defaults); these tests assume a fully observed scan
  in.forward_observed = true;
  in.min_rear_range_m = 5.0;
  in.rear_observed = true;
  in.rotation_observed = true;
  return in;
}
}  // namespace

TEST(VelocityGate, PassesNominalCommand)
{
  const auto out = applyGate(nominal(0.3, 0.5), GateLimits{});
  EXPECT_EQ(out.state, GateState::kPass);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.3);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.5);
}

TEST(VelocityGate, ClampsToLimits)
{
  GateLimits lim;
  lim.max_linear = 0.4;
  lim.max_angular = 1.0;
  const auto out = applyGate(nominal(2.0, -3.0), lim);
  EXPECT_EQ(out.state, GateState::kLimited);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.4);
  EXPECT_DOUBLE_EQ(out.velocity.angular, -1.0);
}

TEST(VelocityGate, StopsOnScanFault)
{
  auto in = nominal(0.3, 0.2);
  in.scan_healthy = false;
  const auto out = applyGate(in, GateLimits{});
  EXPECT_EQ(out.state, GateState::kStoppedScanFault);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.0);
}

TEST(VelocityGate, StopsOnStaleCommand)
{
  auto in = nominal(0.3, 0.2);
  in.command_age_s = 2.0;
  EXPECT_EQ(applyGate(in, GateLimits{}).state, GateState::kStoppedNoCommand);
}

TEST(VelocityGate, RejectsNonFiniteCommand)
{
  auto in = nominal(std::numeric_limits<double>::quiet_NaN(), 0.0);
  const auto out = applyGate(in, GateLimits{});
  EXPECT_EQ(out.state, GateState::kStoppedNoCommand);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
}

TEST(VelocityGate, EstopHasHighestPriority)
{
  auto in = nominal(0.3, 0.2);
  in.estop = true;
  in.scan_healthy = false;
  EXPECT_EQ(applyGate(in, GateLimits{}).state, GateState::kStoppedEstop);
}

TEST(VelocityGate, BlocksForwardButAllowsRotationAndReverse)
{
  GateLimits lim;
  auto in = nominal(0.3, 0.6);
  in.min_front_range_m = 0.1;
  auto out = applyGate(in, lim);
  EXPECT_EQ(out.state, GateState::kBlockedObstacle);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.6);

  in.command.linear = -0.1;
  out = applyGate(in, lim);
  EXPECT_DOUBLE_EQ(out.velocity.linear, -0.1);
}

TEST(VelocityGate, SlowsLinearlyInSlowZone)
{
  GateLimits lim;
  lim.stop_distance_m = 0.2;
  lim.slow_distance_m = 0.6;
  auto in = nominal(0.4, 0.0);
  in.min_front_range_m = 0.4;  // halfway
  const auto out = applyGate(in, lim);
  EXPECT_EQ(out.state, GateState::kSlowed);
  EXPECT_NEAR(out.velocity.linear, 0.2, 1e-9);
}

TEST(FrontClearance, IgnoresPointsOutsideCorridorAndInvalid)
{
  // 4 beams: -90, 0, +90 deg, and one NaN
  const double inc = M_PI / 2.0;
  std::vector<float> ranges = {0.3f, 2.0f, 0.3f, std::numeric_limits<float>::quiet_NaN()};
  const double c = frontClearance(ranges, -M_PI / 2.0, inc, 0.1, 10.0, 0.27, 0.15);
  EXPECT_NEAR(c, 1.85, 1e-6);  // only the straight-ahead beam counts
}

TEST(FrontClearance, ClearReturnsInfinity)
{
  std::vector<float> ranges(10, std::numeric_limits<float>::infinity());
  EXPECT_TRUE(std::isinf(frontClearance(ranges, -1.0, 0.2, 0.1, 10.0, 0.27, 0.15)));
}

TEST(FrontClearance, ObstacleInsideFrontEdgeClampsToZero)
{
  std::vector<float> ranges = {0.13f};
  EXPECT_DOUBLE_EQ(frontClearance(ranges, 0.0, 0.1, 0.12, 10.0, 0.27, 0.15), 0.0);
}
