// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
// V3 (Astra V2 review, F3 HIGH): motion-relevant sector coverage, per-direction gating and the
// node logic (GateSupervisor: liveness, stamps, clock rollback, command watchdog).
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "inspectloop_safety/gate_supervisor.hpp"
#include "inspectloop_safety/velocity_gate.hpp"

using namespace inspectloop_safety;  // NOLINT

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kDeg = M_PI / 180.0;

/// n beams spanning [amin, amax] (inclusive), all set to `value`
ScanData makeScan(std::size_t n, float value, double amin = -M_PI, double amax_excl = M_PI,
  bool full_circle = true)
{
  ScanData s;
  s.angle_min = amin;
  s.angle_increment = full_circle ? (amax_excl - amin) / static_cast<double>(n) :
    (amax_excl - amin) / static_cast<double>(n - 1);
  s.angle_max = s.angle_min + s.angle_increment * static_cast<double>(n - 1);
  s.range_min = 0.12;
  s.range_max = 12.0;
  s.ranges.assign(n, value);
  return s;
}

double angleOf(const ScanData & s, std::size_t i)
{
  return s.angle_min + static_cast<double>(i) * s.angle_increment;
}

void setSector(ScanData & s, double lo_deg, double hi_deg, float value)
{
  for (std::size_t i = 0; i < s.ranges.size(); ++i) {
    const double a = std::atan2(std::sin(angleOf(s, i)), std::cos(angleOf(s, i))) / kDeg;
    if (a >= lo_deg && a <= hi_deg) {
      s.ranges[i] = value;
    }
  }
}

/// The exact Astra V2 review scenario: 720 beams, 5 m outside +-35 deg, NaN inside.
ScanData astraScan()
{
  ScanData s = makeScan(720, 5.0f);
  for (std::size_t i = 0; i < 720; ++i) {
    if (std::fabs(angleOf(s, i)) <= 35.0 * kDeg) {
      s.ranges[i] = kNaN;
    }
  }
  return s;
}

GateOutput gateFor(const ScanData & s, double v, double w)
{
  const ScanAssessment a = assessScan(s, CoverageParams{});
  GateInputs in;
  in.command = {v, w};
  in.command_age_s = 0.05;
  in.scan_healthy = a.validity == ScanValidity::kValid;
  in.min_front_range_m = a.front_clearance_m;
  in.forward_observed = a.forward.observed;
  in.min_rear_range_m = a.rear_clearance_m;
  in.rear_observed = a.rear.observed;
  in.rotation_observed = a.full.observed;
  in.rotation_obstacle = a.rotation_obstacle;
  return applyGate(in, GateLimits{});
}

/// supervisor fed like the node: stamp == ros time, steady clock in seconds
struct Rig
{
  GateSupervisor sup{SupervisorParams{}};
  double ros{100.0};
  double steady{1000.0};
  void scan(const ScanData & s, double stamp_offset = 0.0)
  {
    sup.onScan(s, ros + stamp_offset, ros, steady);
  }
  void cmd(double v, double w) {sup.onCommand({v, w}, steady);}
  GateSupervisor::Result tick() {return sup.tick(ros, steady);}
  void advance(double dt) {ros += dt; steady += dt;}
};
}  // namespace

// ---------------------------------------------------------------- the mandatory regression
TEST(V3Coverage, AstraExactScenarioForwardSectorIsUnobserved)
{
  const ScanAssessment a = assessScan(astraScan(), CoverageParams{});
  EXPECT_EQ(a.validity, ScanValidity::kValid);           // structurally fine, as in V2 ...
  EXPECT_NEAR(a.forward_usable_fraction, 0.612188, 1e-5);
  EXPECT_FALSE(a.forward.observed);                      // ... but the corridor is not observed
  EXPECT_GE(a.forward.max_gap_rad, 70.0 * kDeg - 1e-6);
}

TEST(V3Coverage, AstraExactScenarioForwardCommandDoesNotPassThePureGate)
{
  const GateOutput out = gateFor(astraScan(), 0.4, 0.2);
  EXPECT_EQ(out.state, GateState::kBlockedUnobserved);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
  // only 90 % of the circle is observed -> an in-place turn is not covered either
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.0);
}

TEST(V3Coverage, AstraExactScenarioThroughTheNodeLogic)
{
  Rig r;
  r.cmd(0.4, 0.2);
  r.scan(astraScan());
  const auto res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kBlockedUnobserved);
  EXPECT_DOUBLE_EQ(res.output.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(res.output.velocity.angular, 0.0);
  EXPECT_NE(res.reason.find("forward sector unobserved"), std::string::npos) << res.reason;
}

// ---------------------------------------------------------------- required companion cases
TEST(V3Coverage, AllNaNScanStopsEverything)
{
  Rig r;
  r.cmd(0.4, 0.2);
  r.scan(makeScan(720, kNaN));
  const auto res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kStoppedScanFault);
  EXPECT_DOUBLE_EQ(res.output.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(res.output.velocity.angular, 0.0);
}

TEST(V3Coverage, EmptyScanStopsEverything)
{
  Rig r;
  r.cmd(0.4, 0.2);
  ScanData s = makeScan(720, 5.0f);
  s.ranges.clear();
  r.scan(s);
  const auto res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kStoppedScanFault);
  EXPECT_NE(res.reason.find("EMPTY"), std::string::npos);
}

TEST(V3Coverage, ValidClearScanPassesAllMotion)
{
  for (float v : {kInf, 5.0f}) {
    Rig r;
    r.cmd(0.4, 0.2);
    r.scan(makeScan(720, v));
    const auto res = r.tick();
    EXPECT_EQ(res.output.state, GateState::kPass);
    EXPECT_DOUBLE_EQ(res.output.velocity.linear, 0.4);
    EXPECT_DOUBLE_EQ(res.output.velocity.angular, 0.2);
  }
}

TEST(V3Coverage, ObstacleInStoppingCorridorBlocksForwardOnly)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, -1.0, 1.0, 0.30f);       // 0.15 m ahead of the front edge (< stop 0.30)
  const GateOutput out = gateFor(s, 0.4, 0.2);
  EXPECT_EQ(out.state, GateState::kBlockedObstacle);
  EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.2);
  EXPECT_DOUBLE_EQ(gateFor(s, -0.1, 0.0).velocity.linear, -0.1);   // reverse away is fine
}

TEST(V3Coverage, ClockRollbackInvalidatesScanUntilANewScanArrives)
{
  Rig r;
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f));
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
  r.ros -= 0.8;                          // simulation clock rolled back by 0.8 s
  r.steady += 0.05;
  r.cmd(0.4, 0.0);
  const auto res = r.tick();
  EXPECT_TRUE(res.clock_jump);
  EXPECT_EQ(res.output.state, GateState::kStoppedScanFault);
  EXPECT_DOUBLE_EQ(res.output.velocity.linear, 0.0);
  // a scan stamped on the OLD timeline is now in the future -> still rejected
  r.sup.onScan(makeScan(720, 5.0f), r.ros + 0.8, r.ros, r.steady);
  EXPECT_EQ(r.tick().output.state, GateState::kStoppedScanFault);
  // a scan from the new timeline restores motion
  r.advance(0.05);
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f));
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
}

TEST(V3Coverage, RollbackDetectedInTheScanCallbackToo)
{
  Rig r;
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f));
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
  r.ros -= 0.01;                         // even a 10 ms rollback is a discontinuity
  r.scan(makeScan(720, 5.0f), +0.01);    // stamped on the old timeline
  EXPECT_TRUE(r.tick().clock_jump);
}

TEST(V3Coverage, StaleScanStampAndScanTimeoutStop)
{
  Rig r;
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f), -1.5);     // header 1.5 s old (limit 1.0)
  auto res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kStoppedScanFault);
  EXPECT_NE(res.reason.find("STALE"), std::string::npos) << res.reason;
  r.scan(makeScan(720, 5.0f));
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
  r.steady += 0.6;                       // no new scan for 0.6 s steady time (limit 0.5)
  r.cmd(0.4, 0.0);
  res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kStoppedScanFault);
  EXPECT_EQ(res.reason, "scan timeout");
}

TEST(V3Coverage, FutureScanStampStops)
{
  Rig r;
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f), +0.5);
  EXPECT_EQ(r.tick().output.state, GateState::kStoppedScanFault);
}

TEST(V3Coverage, NegativeAndNaNCommandAgeStop)
{
  Rig r;
  r.scan(makeScan(720, 5.0f));
  r.sup.onCommand({0.4, 0.0}, r.steady + 5.0);         // command "from the future"
  EXPECT_EQ(r.tick().output.state, GateState::kStoppedNoCommand);
  r.sup.onCommand({0.4, 0.0}, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(r.tick().output.state, GateState::kStoppedNoCommand);
  r.cmd(0.4, 0.0);
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
}

TEST(V3Coverage, CommandWatchdogIsPreserved)
{
  Rig r;
  r.cmd(0.4, 0.0);
  r.scan(makeScan(720, 5.0f));
  r.steady += 0.45;
  r.ros += 0.45;
  r.scan(makeScan(720, 5.0f));
  EXPECT_EQ(r.tick().output.state, GateState::kPass);
  r.steady += 0.1;                       // command now 0.55 s old (deadman 0.5 s)
  r.ros += 0.1;
  r.scan(makeScan(720, 5.0f));
  const auto res = r.tick();
  EXPECT_EQ(res.output.state, GateState::kStoppedNoCommand);
  EXPECT_DOUBLE_EQ(res.output.velocity.linear, 0.0);
}

TEST(V3Coverage, NoScanEverStopsEverything)
{
  Rig r;
  r.cmd(0.4, 0.2);
  EXPECT_EQ(r.tick().output.state, GateState::kStoppedScanFault);
}

// ---------------------------------------------------------------- partial / mixed blindness
TEST(V3Coverage, SmallGapIsToleratedLargerGapIsNot)
{
  ScanData s = makeScan(720, 5.0f);       // 0.5 deg per beam
  setSector(s, -0.6, 0.6, kNaN);          // 3 beams = 1.5 deg
  EXPECT_TRUE(assessScan(s, CoverageParams{}).forward.observed);
  setSector(s, -1.4, 1.4, kNaN);          // 6 beams = 3 deg > 2 deg
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_FALSE(a.forward.observed);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.0);
}

TEST(V3Coverage, NarrowBlindWedgeAtTheCorridorEdgeBlocks)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, 50.0, 56.0, kNaN);         // inside the +-60.9 deg corridor sector
  EXPECT_FALSE(assessScan(s, CoverageParams{}).forward.observed);
  ScanData t = makeScan(720, 5.0f);
  setSector(t, 65.0, 80.0, kNaN);         // outside it: the forward corridor is still observed
  EXPECT_TRUE(assessScan(t, CoverageParams{}).forward.observed);
}

TEST(V3Coverage, ScatteredInvalidBeamsLowerCoverage)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  ScanData s = makeScan(720, 5.0f);
  for (std::size_t i = 0; i < s.ranges.size(); ++i) {
    if (u(rng) < 0.12) {
      s.ranges[i] = (i % 2) ? kNaN : 50.0f;         // NaN or beyond range_max
    }
  }
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_EQ(a.validity, ScanValidity::kValid);
  EXPECT_LT(a.forward.coverage, 0.95);
  EXPECT_FALSE(a.forward.observed);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.0);
}

TEST(V3Coverage, OutOfRangeFiniteValuesAreUnobservedNotFree)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, -20.0, 20.0, 0.05f);       // below range_min, not flagged as -inf
  EXPECT_FALSE(assessScan(s, CoverageParams{}).forward.observed);
  setSector(s, -20.0, 20.0, 99.0f);       // beyond range_max
  EXPECT_FALSE(assessScan(s, CoverageParams{}).forward.observed);
}

TEST(V3Coverage, RearBlindnessBlocksReverseButNotForward)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, 150.0, 180.0, kNaN);
  setSector(s, -180.0, -150.0, kNaN);
  EXPECT_DOUBLE_EQ(gateFor(s, -0.15, 0.0).velocity.linear, 0.0);
  EXPECT_EQ(gateFor(s, -0.15, 0.0).state, GateState::kBlockedUnobserved);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.3);
}

TEST(V3Coverage, ObstacleBehindBlocksReverse)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, 178.0, 180.0, 0.45f);
  setSector(s, -180.0, -178.0, 0.45f);    // 0.10 m behind the rear edge (< stop 0.30)
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_NEAR(a.rear_clearance_m, 0.10, 1e-3);
  EXPECT_DOUBLE_EQ(gateFor(s, -0.15, 0.0).velocity.linear, 0.0);
  EXPECT_EQ(gateFor(s, -0.15, 0.0).state, GateState::kBlockedObstacle);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.3);
}

TEST(V3Coverage, ObstacleInsideTurningCircleBlocksRotation)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, 109.0, 111.0, 0.30f);      // base frame ~(0.0, 0.28): beside the robot, outside
                                          // the corridor, 0.28 m from the turning centre
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_TRUE(a.rotation_obstacle);
  const auto out = gateFor(s, 0.0, 0.5);
  EXPECT_DOUBLE_EQ(out.velocity.angular, 0.0);
  EXPECT_EQ(out.state, GateState::kBlockedObstacle);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.3);  // corridor ahead still clear
}

// ---------------------------------------------------------------- beam counts / resolutions
TEST(V3Coverage, DifferentBeamCountsGiveConsistentResults)
{
  for (std::size_t n : {360u, 720u, 1080u, 1440u}) {
    ScanData clear = makeScan(n, 5.0f);
    EXPECT_TRUE(assessScan(clear, CoverageParams{}).forward.observed) << n;
    EXPECT_DOUBLE_EQ(gateFor(clear, 0.4, 0.2).velocity.linear, 0.4) << n;
    ScanData blind = makeScan(n, 5.0f);
    setSector(blind, -35.0, 35.0, kNaN);
    EXPECT_FALSE(assessScan(blind, CoverageParams{}).forward.observed) << n;
    EXPECT_DOUBLE_EQ(gateFor(blind, 0.4, 0.2).velocity.linear, 0.0) << n;
    ScanData obstacle = makeScan(n, 5.0f);
    setSector(obstacle, -2.0, 2.0, 0.30f);
    EXPECT_DOUBLE_EQ(gateFor(obstacle, 0.4, 0.0).velocity.linear, 0.0) << n;
  }
}

TEST(V3Coverage, TooCoarseResolutionCannotProtect)
{
  ScanData s = makeScan(90, 5.0f);        // 4 deg per beam > 1 deg limit
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_EQ(a.validity, ScanValidity::kValid);
  EXPECT_FALSE(a.forward.observed);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.0);
}

TEST(V3Coverage, LimitedFieldOfViewScannerCoversOnlyWhatItSees)
{
  // 270 deg scanner (-135..+135): forward observed, rear corridor never sampled
  ScanData s = makeScan(1081, 5.0f, -135.0 * kDeg, 135.0 * kDeg, false);
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_TRUE(a.forward.observed);
  EXPECT_FALSE(a.rear.observed);
  EXPECT_FALSE(a.full.observed);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.3, 0.0).velocity.linear, 0.3);
  EXPECT_DOUBLE_EQ(gateFor(s, -0.1, 0.0).velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(gateFor(s, 0.0, 0.5).velocity.angular, 0.0);
  // +-30 deg scanner: does not even sample the +-60.9 deg forward corridor sector
  ScanData narrow = makeScan(121, 5.0f, -30.0 * kDeg, 30.0 * kDeg, false);
  EXPECT_FALSE(assessScan(narrow, CoverageParams{}).forward.observed);
  EXPECT_DOUBLE_EQ(gateFor(narrow, 0.3, 0.0).velocity.linear, 0.0);
}

TEST(V3Coverage, SectorAnglesFollowTheCorridorGeometry)
{
  CoverageParams p;
  EXPECT_NEAR(p.forwardHalfAngle() / kDeg, std::atan2(0.27, 0.15) / kDeg, 1e-9);   // ~60.9
  EXPECT_NEAR(p.rearHalfAngle() / kDeg, std::atan2(0.27, 0.35) / kDeg, 1e-9);      // ~37.6
  p.forward_half_angle_rad = 80.0 * kDeg;
  EXPECT_NEAR(p.forwardHalfAngle() / kDeg, 80.0, 1e-9);
}

TEST(V3Coverage, WrapAroundGapAcrossPiIsMeasuredAsOneGap)
{
  ScanData s = makeScan(720, 5.0f);
  setSector(s, 178.6, 180.0, kNaN);
  setSector(s, -180.0, -178.6, kNaN);     // 2.8 deg gap straddling +-180 deg
  const auto a = assessScan(s, CoverageParams{});
  EXPECT_GT(a.full.max_gap_rad, 2.0 * kDeg);
  EXPECT_FALSE(a.rear.observed);
}
