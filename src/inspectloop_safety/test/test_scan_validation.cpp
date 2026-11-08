// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
// Astra F3: LaserScan structural / content validation (fail closed).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "inspectloop_safety/velocity_gate.hpp"

using inspectloop_safety::applyGate;
using inspectloop_safety::assessScan;
using inspectloop_safety::checkStamp;
using inspectloop_safety::GateInputs;
using inspectloop_safety::GateLimits;
using inspectloop_safety::GateState;
using inspectloop_safety::ScanData;
using inspectloop_safety::ScanValidity;
using inspectloop_safety::StampCheck;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

ScanData scan(std::size_t n, float value)
{
  ScanData s;
  s.angle_min = -M_PI;
  s.angle_increment = 2.0 * M_PI / static_cast<double>(n);
  s.angle_max = s.angle_min + s.angle_increment * static_cast<double>(n - 1);
  s.range_min = 0.12;
  s.range_max = 12.0;
  s.ranges.assign(n, value);
  return s;
}

const double kHalfWidth = 0.27;
const double kFront = 0.15;
}  // namespace

TEST(ScanValidation, NominalClearScanIsValid)
{
  const auto a = assessScan(scan(720, std::numeric_limits<float>::infinity()), kHalfWidth, kFront);
  EXPECT_EQ(a.validity, ScanValidity::kValid);
  EXPECT_TRUE(std::isinf(a.front_clearance_m));
}

TEST(ScanValidation, EmptyScanIsInvalid)
{
  ScanData s = scan(720, 5.0f);
  s.ranges.clear();
  const auto a = assessScan(s, kHalfWidth, kFront);
  EXPECT_NE(a.validity, ScanValidity::kValid);
}

TEST(ScanValidation, AllNaNScanIsInvalid)
{
  const auto a = assessScan(scan(720, std::numeric_limits<float>::quiet_NaN()), kHalfWidth, kFront);
  EXPECT_NE(a.validity, ScanValidity::kValid);
}

TEST(ScanValidation, ForwardHalfMostlyNaNIsInvalid)
{
  ScanData s = scan(720, 5.0f);
  for (std::size_t i = 180; i < 540; ++i) {      // -90..+90 deg = forward half
    if (i % 4 != 0) {
      s.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  EXPECT_NE(assessScan(s, kHalfWidth, kFront).validity, ScanValidity::kValid);
}

TEST(ScanValidation, InvalidMetadataIsInvalid)
{
  ScanData s = scan(720, 5.0f);
  s.angle_increment = 0.0;
  EXPECT_NE(assessScan(s, kHalfWidth, kFront).validity, ScanValidity::kValid);
  s = scan(720, 5.0f);
  s.angle_increment = kNaN;
  EXPECT_NE(assessScan(s, kHalfWidth, kFront).validity, ScanValidity::kValid);
  s = scan(720, 5.0f);
  s.range_max = s.range_min;
  EXPECT_NE(assessScan(s, kHalfWidth, kFront).validity, ScanValidity::kValid);
  s = scan(720, 5.0f);
  s.ranges.resize(100);                          // inconsistent with the angle span
  EXPECT_NE(assessScan(s, kHalfWidth, kFront).validity, ScanValidity::kValid);
}

TEST(ScanValidation, NegativeInfinityMeansObstacleAtTheSensor)
{
  ScanData s = scan(720, std::numeric_limits<float>::infinity());
  s.ranges[360] = -std::numeric_limits<float>::infinity();   // straight ahead (angle 0)
  const auto a = assessScan(s, kHalfWidth, kFront);
  EXPECT_EQ(a.validity, ScanValidity::kValid);
  EXPECT_DOUBLE_EQ(a.front_clearance_m, 0.0);
}

TEST(ScanValidation, ObstacleAheadMeasured)
{
  ScanData s = scan(720, std::numeric_limits<float>::infinity());
  s.ranges[360] = 1.15f;
  const auto a = assessScan(s, kHalfWidth, kFront);
  EXPECT_NEAR(a.front_clearance_m, 1.0, 1e-5);
}

TEST(StampCheck, RejectsZeroFutureAndStaleStamps)
{
  EXPECT_EQ(checkStamp(0.0, 10.0, 0.5, 0.2), StampCheck::kZero);
  EXPECT_EQ(checkStamp(10.5, 10.0, 0.5, 0.2), StampCheck::kFuture);
  EXPECT_EQ(checkStamp(9.0, 10.0, 0.5, 0.2), StampCheck::kStale);
  EXPECT_EQ(checkStamp(9.9, 10.0, 0.5, 0.2), StampCheck::kOk);
  EXPECT_EQ(checkStamp(kNaN, 10.0, 0.5, 0.2), StampCheck::kZero);
  EXPECT_EQ(checkStamp(10.1, 10.0, 0.5, 0.2), StampCheck::kOk);   // within the future tolerance
}

TEST(GateWithScanValidity, InvalidScanStopsTheRobot)
{
  GateInputs in;
  in.command = {0.4, 0.3};
  in.command_age_s = 0.05;
  in.scan_healthy = false;          // the node sets this from assessScan + liveness + stamp
  in.min_front_range_m = kInf;
  const auto out = applyGate(in, GateLimits{});
  EXPECT_EQ(out.state, GateState::kStoppedScanFault);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.0);
}
