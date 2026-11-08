// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
// Astra F3 probes expressed against the v1 API (these failed before the fix).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "inspectloop_safety/topic_monitor.hpp"
#include "inspectloop_safety/velocity_gate.hpp"

using inspectloop_safety::applyGate;
using inspectloop_safety::GateInputs;
using inspectloop_safety::GateLimits;
using inspectloop_safety::GateState;
using inspectloop_safety::StreamHealth;
using inspectloop_safety::TopicMonitor;

namespace
{
GateInputs moving(double age)
{
  GateInputs in;
  in.command = {0.4, 0.0};
  in.command_age_s = age;
  in.scan_healthy = true;
  in.min_front_range_m = 5.0;
  return in;
}
}  // namespace

TEST(AstraF3, NegativeCommandAgeIsNotFresh)
{
  const auto out = applyGate(moving(-0.2), GateLimits{});
  EXPECT_EQ(out.state, GateState::kStoppedNoCommand);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
}

TEST(AstraF3, NonFiniteCommandAgeIsNotFresh)
{
  EXPECT_EQ(applyGate(moving(std::numeric_limits<double>::quiet_NaN()), GateLimits{}).state,
    GateState::kStoppedNoCommand);
}

TEST(AstraF3, NonFiniteClearanceBlocksForwardMotion)
{
  auto in = moving(0.05);
  in.min_front_range_m = std::numeric_limits<double>::quiet_NaN();
  const auto out = applyGate(in, GateLimits{});
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
}

TEST(AstraF3, ClockRollbackWithoutNewDataIsNotHealthy)
{
  TopicMonitor m(0.5, 0.0);
  for (int i = 0; i < 10; ++i) {
    m.recordMessage(100.0 + 0.1 * i);
  }
  EXPECT_EQ(m.evaluate(100.95), StreamHealth::kOk);
  // simulation reset: time jumps back to 2 s, no message since
  EXPECT_NE(m.evaluate(2.0), StreamHealth::kOk);
}

TEST(AstraF3, EmptyOrAllNaNScanDoesNotReportFreeSpace)
{
  using inspectloop_safety::frontClearance;
  const std::vector<float> empty;
  EXPECT_FALSE(std::isinf(frontClearance(empty, -1.0, 0.1, 0.1, 10.0, 0.27, 0.15)));
  const std::vector<float> nans(720, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(std::isinf(frontClearance(nans, -M_PI, M_PI / 360.0, 0.1, 10.0, 0.27, 0.15)));
  // and the gate must not move forward on such a "clearance"
  auto in = moving(0.05);
  in.min_front_range_m = frontClearance(empty, -1.0, 0.1, 0.1, 10.0, 0.27, 0.15);
  EXPECT_DOUBLE_EQ(applyGate(in, GateLimits{}).velocity.linear, 0.0);
}
