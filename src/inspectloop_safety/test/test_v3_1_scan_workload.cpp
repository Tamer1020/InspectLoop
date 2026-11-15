// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
// V3.1 (audit F3 HIGH): bounded sector-coverage workload and metadata plausibility.
//
// V3's sectorCoverage() iterated over 2*pi / angle_increment sample directions. A 720-beam
// scan with angle_increment = 1e-9 (and a matching 719e-9 rad span, so the size-consistency
// check passed) cost ~6.3e9 beam lookups for the full circle alone: the scan callback blocked
// the single-threaded executor and with it the gate's watchdog timer.
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include "inspectloop_safety/gate_supervisor.hpp"
#include "inspectloop_safety/velocity_gate.hpp"

using namespace inspectloop_safety;  // NOLINT

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kDeg = M_PI / 180.0;
using Clock = std::chrono::steady_clock;

/// Gate publish period at the configured 20 Hz, and the per-callback budget we allow a scan
/// callback on the shared single-threaded executor (10 % of one watchdog period).
constexpr double kTickPeriodS = 1.0 / 20.0;
constexpr double kScanCallbackBudgetS = 0.1 * kTickPeriodS;

/// The audit's exact input: 720 beams, angle_increment = 1e-9, metadata self-consistent.
ScanData degenerateScan(double amin = 0.0, float value = 5.0f)
{
  ScanData s;
  s.angle_min = amin;
  s.angle_increment = 1e-9;
  s.angle_max = s.angle_min + 719.0 * s.angle_increment;
  s.range_min = 0.12;
  s.range_max = 12.0;
  s.ranges.assign(720, value);
  return s;
}

ScanData fullScan(std::size_t n = 720, float value = 5.0f)
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

std::size_t lookupBound(const ScanData & s) {return 3 * (s.ranges.size() + 6);}

/// Run fn on a worker thread; if it does not finish within timeout_s the process exits with a
/// failure instead of hanging the test run (the V3 code takes minutes on these inputs).
double runBounded(const std::function<void()> & fn, double timeout_s, const char * what)
{
  struct Shared
  {
    std::mutex m;
    std::condition_variable cv;
    bool done{false};
    double secs{0.0};
  };
  auto sh = std::make_shared<Shared>();
  std::thread t([sh, fn]() {
      const auto t0 = Clock::now();
      fn();
      const double secs = std::chrono::duration<double>(Clock::now() - t0).count();
      std::lock_guard<std::mutex> lk(sh->m);
      sh->secs = secs;
      sh->done = true;
      sh->cv.notify_all();
    });
  std::unique_lock<std::mutex> lk(sh->m);
  if (!sh->cv.wait_for(lk, std::chrono::duration<double>(timeout_s), [&] {return sh->done;})) {
    std::fprintf(stderr, "[  FATAL   ] %s did not return within %.1f s (unbounded workload)\n",
      what, timeout_s);
    std::fflush(stderr);
    std::_Exit(1);
  }
  lk.unlock();
  t.join();
  return sh->secs;
}

void expectNoMotion(const ScanAssessment & a)
{
  GateInputs in;
  in.command = {0.3, 0.5};
  in.command_age_s = 0.05;
  in.scan_healthy = a.validity == ScanValidity::kValid;
  in.min_front_range_m = a.front_clearance_m;
  in.forward_observed = a.forward.observed;
  in.min_rear_range_m = a.rear_clearance_m;
  in.rear_observed = a.rear.observed;
  in.rotation_observed = a.full.observed;
  in.rotation_obstacle = a.rotation_obstacle;
  for (const Velocity cmd : {Velocity{0.3, 0.0}, Velocity{-0.2, 0.0}, Velocity{0.0, 0.8},
      Velocity{0.3, 0.5}})
  {
    in.command = cmd;
    const GateOutput out = applyGate(in, GateLimits{});
    EXPECT_DOUBLE_EQ(out.velocity.linear, 0.0);
    EXPECT_DOUBLE_EQ(out.velocity.angular, 0.0);
  }
}

// ------------------------------------------------ reference: the V3 brute-force algorithm
// Kept verbatim (minus the unbounded sample count) as an oracle: on plausible scans the
// bounded implementation must return exactly the same coverage / gap / verdict.
long refBeamIndex(const ScanData & s, double theta)
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

SectorCoverage refSectorCoverage(const ScanData & s, double centre, double half_angle,
  double min_coverage, double max_gap_rad, double max_increment_rad)
{
  SectorCoverage out;
  const double inc = s.angle_increment;
  const bool full_circle = half_angle >= M_PI - 1e-9;
  const long samples = full_circle ? std::max(1L, std::lround(2.0 * M_PI / inc)) :
    2L * static_cast<long>(std::floor(half_angle / inc)) + 1L;
  const double start = full_circle ? centre - M_PI : centre - inc * static_cast<double>(
    (samples - 1) / 2);
  long observed = 0, run = 0, longest = 0, leading = -1;
  for (long k = 0; k < samples; ++k) {
    const long idx = refBeamIndex(s, start + static_cast<double>(k) * inc);
    const double r = idx >= 0 ? s.ranges[static_cast<std::size_t>(idx)] : 0.0;
    const bool ok = idx >= 0 &&
      (std::isinf(r) || (std::isfinite(r) && r >= s.range_min && r <= s.range_max));
    if (ok) {
      ++observed;
      if (leading < 0) {
        leading = run;
      }
      longest = std::max(longest, run);
      run = 0;
    } else {
      ++run;
    }
  }
  if (leading < 0) {
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
}  // namespace

// ------------------------------------------------------------- the mandatory regressions
TEST(V31ScanWorkload, AuditScan720BeamsIncrement1e9IsBadMetadataAndBounded)
{
  const ScanData s = degenerateScan();
  ScanAssessment a;
  const double secs = runBounded([&] {a = assessScan(s, CoverageParams{});},
      2.0, "assessScan(720 beams, inc=1e-9)");
  EXPECT_EQ(a.validity, ScanValidity::kBadMetadata);
  EXPECT_FALSE(a.forward.observed);
  EXPECT_FALSE(a.rear.observed);
  EXPECT_FALSE(a.full.observed);
  EXPECT_EQ(a.forward.lookups + a.rear.lookups + a.full.lookups, 0u);
  EXPECT_LT(secs, kScanCallbackBudgetS);
  expectNoMotion(a);
}

TEST(V31ScanWorkload, AuditScanDirectSectorCoverageCallsAreBounded)
{
  const ScanData s = degenerateScan(-M_PI);
  const CoverageParams p;
  for (const double half : {p.forwardHalfAngle(), p.rearHalfAngle(), M_PI}) {
    SectorCoverage c;
    const double secs = runBounded([&] {
          c = sectorCoverage(s, 0.0, half, p.min_sector_coverage, p.max_sector_gap_rad,
            p.max_angle_increment_rad);
        }, 2.0, "sectorCoverage(720 beams, inc=1e-9)");
    EXPECT_FALSE(c.observed);
    EXPECT_LE(c.lookups, lookupBound(s));
    EXPECT_LT(secs, kScanCallbackBudgetS);
  }
}

TEST(V31ScanWorkload, AuditScanThroughTheSupervisorStopsAfterAValidScan)
{
  GateSupervisor sup{SupervisorParams{}};
  double ros = 100.0, steady = 1000.0;
  sup.onCommand({0.3, 0.0}, steady);
  sup.onScan(fullScan(), ros, ros, steady);
  ASSERT_EQ(sup.tick(ros, steady).output.state, GateState::kPass);
  ros += 0.05;
  steady += 0.05;
  sup.onCommand({0.3, 0.4}, steady);
  runBounded([&] {sup.onScan(degenerateScan(), ros, ros, steady);}, 2.0,
    "GateSupervisor::onScan(inc=1e-9)");
  const auto r = sup.tick(ros, steady);
  EXPECT_EQ(r.output.state, GateState::kStoppedScanFault);  // the earlier valid scan is gone
  EXPECT_DOUBLE_EQ(r.output.velocity.linear, 0.0);
  EXPECT_DOUBLE_EQ(r.output.velocity.angular, 0.0);
  EXPECT_NE(r.reason.find("BAD_METADATA"), std::string::npos) << r.reason;
}

// ----------------------------------------------------------------- metadata validation
TEST(V31ScanWorkload, ImplausibleOrDegenerateMetadataFailsClosed)
{
  const double dmax = std::numeric_limits<double>::max();
  const double dinf = std::numeric_limits<double>::infinity();
  const double dnan = std::numeric_limits<double>::quiet_NaN();
  std::vector<std::pair<const char *, std::function<void(ScanData &)>>> cases = {
    {"inc=1e-9", [](ScanData & s) {s = degenerateScan();}},
    {"inc=1e-300 consistent", [](ScanData & s) {
        s.angle_increment = 1e-300; s.angle_min = 0.0; s.angle_max = 719e-300;
      }},
    {"inc=denormal", [](ScanData & s) {
        s.angle_increment = 4.9e-324; s.angle_min = 0.0; s.angle_max = 0.0;
      }},
    {"inc just below minimum", [](ScanData & s) {
        s.angle_increment = 0.99 * kMinScanAngleIncrementRad; s.angle_min = 0.0;
        s.angle_max = 719.0 * s.angle_increment;
      }},
    {"inc=0", [](ScanData & s) {s.angle_increment = 0.0;}},
    {"inc<0", [](ScanData & s) {s.angle_increment = -s.angle_increment;}},
    {"inc=NaN", [&](ScanData & s) {s.angle_increment = dnan;}},
    {"inc=inf", [&](ScanData & s) {s.angle_increment = dinf;}},
    {"inc>2pi", [](ScanData & s) {s.angle_increment = 7.0; s.angle_max = s.angle_min + 719 * 7.0;}},
    {"three revolutions", [](ScanData & s) {
        s.angle_increment = 6.0 * M_PI / 719.0; s.angle_min = -3.0 * M_PI;
        s.angle_max = 3.0 * M_PI;
      }},
    {"angle_min=-inf", [&](ScanData & s) {s.angle_min = -dinf;}},
    {"angle_max=NaN", [&](ScanData & s) {s.angle_max = dnan;}},
    {"angle_max<angle_min", [](ScanData & s) {std::swap(s.angle_min, s.angle_max);}},
    {"huge angles", [&](ScanData & s) {s.angle_min = -dmax; s.angle_max = dmax;}},
    {"range_min<0", [](ScanData & s) {s.range_min = -1.0;}},
    {"range_max=NaN", [&](ScanData & s) {s.range_max = dnan;}},
    {"range_max=inf", [&](ScanData & s) {s.range_max = dinf;}},
    {"range_max<=range_min", [](ScanData & s) {s.range_max = s.range_min;}},
    {"size inconsistent", [](ScanData & s) {s.ranges.resize(100, 5.0f);}},
    {"empty", [](ScanData & s) {s.ranges.clear();}},
  };
  for (auto & [name, mutate] : cases) {
    SCOPED_TRACE(name);
    ScanData s = fullScan();
    mutate(s);
    ScanAssessment a;
    const double secs = runBounded([&] {a = assessScan(s, CoverageParams{});}, 2.0, name);
    EXPECT_NE(a.validity, ScanValidity::kValid);
    EXPECT_FALSE(a.forward.observed || a.rear.observed || a.full.observed);
    EXPECT_LT(secs, kScanCallbackBudgetS);
    expectNoMotion(a);
  }
}

TEST(V31ScanWorkload, PlausibleHighResolutionAndDuplicatedEndpointScansStayValid)
{
  // 0.05 deg (7200 beams) full circle, and the Gazebo convention angle_max = angle_min + 2*pi
  // (first beam repeated at +2*pi) - both are real drivers' metadata and must not be rejected.
  ScanData hi = fullScan(7200);
  EXPECT_EQ(assessScan(hi, CoverageParams{}).validity, ScanValidity::kValid);
  EXPECT_TRUE(assessScan(hi, CoverageParams{}).forward.observed);
  ScanData gz;
  gz.angle_min = -M_PI;
  gz.angle_max = M_PI;
  gz.angle_increment = 2.0 * M_PI / 719.0;
  gz.range_min = 0.12;
  gz.range_max = 12.0;
  gz.ranges.assign(720, 5.0f);
  const auto a = assessScan(gz, CoverageParams{});
  EXPECT_EQ(a.validity, ScanValidity::kValid);
  EXPECT_TRUE(a.forward.observed && a.rear.observed && a.full.observed);
}

// --------------------------------------------------------------------- the work bound
TEST(V31ScanWorkload, LookupsAreBoundedByScanSizeNotSectorSize)
{
  // Plausible metadata at the finest accepted increment but only 720 beams (a 0.4 deg wide
  // scan): V3 looked up ~628k full-circle directions; the bounded version <= 3 * (n + 6).
  ScanData s;
  s.angle_increment = 1.0001 * kMinScanAngleIncrementRad;
  s.angle_min = -359.5 * s.angle_increment;
  s.angle_max = s.angle_min + 719.0 * s.angle_increment;
  s.range_min = 0.12;
  s.range_max = 12.0;
  s.ranges.assign(720, 5.0f);
  const CoverageParams p;
  const ScanAssessment a = assessScan(s, p);
  for (const SectorCoverage * c : {&a.forward, &a.rear, &a.full}) {
    EXPECT_FALSE(c->observed);
    EXPECT_LE(c->lookups, lookupBound(s));
  }
  const SectorCoverage ref = refSectorCoverage(s, 0.0, M_PI, p.min_sector_coverage,
      p.max_sector_gap_rad, p.max_angle_increment_rad);
  EXPECT_DOUBLE_EQ(a.full.coverage, ref.coverage);
  EXPECT_DOUBLE_EQ(a.full.max_gap_rad, ref.max_gap_rad);
  EXPECT_GT(std::lround(2.0 * M_PI / s.angle_increment), 600000L);   // what V3 iterated
}

TEST(V31ScanWorkload, BoundedImplementationMatchesTheV3AlgorithmOnPlausibleScans)
{
  std::mt19937 rng(20261115);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const CoverageParams p;
  int compared = 0;
  for (int trial = 0; trial < 300; ++trial) {
    // random FOV (30..360 deg), resolution (0.1..1.2 deg), offset, NaN / out-of-range holes
    const double fov = (30.0 + 330.0 * u(rng)) * kDeg;
    const double inc = (0.1 + 1.1 * u(rng)) * kDeg;
    const std::size_t n = std::max<std::size_t>(2, static_cast<std::size_t>(fov / inc));
    ScanData s;
    s.angle_increment = inc;
    s.angle_min = -M_PI + 2.0 * M_PI * u(rng);
    s.angle_max = s.angle_min + inc * static_cast<double>(n - 1);
    s.range_min = 0.12;
    s.range_max = 12.0;
    s.ranges.assign(n, 5.0f);
    const double hole_p = trial % 3 == 0 ? 0.0 : 0.15 * u(rng);
    for (auto & r : s.ranges) {
      const double x = u(rng);
      r = x < hole_p ? kNaN : x < 1.5 * hole_p ? 50.0f : x < 1.6 * hole_p ? kInf : 5.0f;
    }
    if (trial % 5 == 0) {   // a contiguous blind wedge
      const std::size_t a0 = static_cast<std::size_t>(u(rng) * static_cast<double>(n));
      for (std::size_t i = a0; i < std::min(n, a0 + n / 8); ++i) {
        s.ranges[i] = kNaN;
      }
    }
    for (const double centre : {0.0, M_PI, -2.0, 2.5}) {
      for (const double half : {p.forwardHalfAngle(), p.rearHalfAngle(), 10.0 * kDeg, M_PI}) {
        const SectorCoverage got = sectorCoverage(s, centre, half, p.min_sector_coverage,
            p.max_sector_gap_rad, p.max_angle_increment_rad);
        const SectorCoverage ref = refSectorCoverage(s, centre, half, p.min_sector_coverage,
            p.max_sector_gap_rad, p.max_angle_increment_rad);
        ASSERT_DOUBLE_EQ(got.coverage, ref.coverage) << "trial " << trial;
        ASSERT_DOUBLE_EQ(got.max_gap_rad, ref.max_gap_rad) << "trial " << trial;
        ASSERT_EQ(got.observed, ref.observed) << "trial " << trial;
        ASSERT_LE(got.lookups, lookupBound(s));
        ++compared;
      }
    }
  }
  EXPECT_EQ(compared, 300 * 16);
}

// ------------------------------------------------------------ watchdog scheduling
// The node runs the scan callback and the 20 Hz gate timer on one SingleThreadedExecutor: the
// timer (deadman, scan timeout, zero-velocity output) cannot fire while a scan callback runs.
// Model that executor: a 10 Hz stream of adversarial scans interleaved with timer ticks. The
// timer is starved by at most the longest scan callback, which must stay a small fraction of
// the tick period, and every tick must command zero velocity.
TEST(V31ScanWorkload, WatchdogTimerIsNotStarvedByAdversarialScans)
{
  std::vector<ScanData> adversarial = {degenerateScan(), degenerateScan(-M_PI, kNaN)};
  {
    ScanData fine;    // plausible metadata, finest increment, 0.4 deg wide
    fine.angle_increment = kMinScanAngleIncrementRad;
    fine.angle_min = -0.0036;
    fine.angle_max = fine.angle_min + 719.0 * fine.angle_increment;
    fine.range_min = 0.12;
    fine.range_max = 12.0;
    fine.ranges.assign(720, 5.0f);
    adversarial.push_back(fine);
  }
  ScanData big = fullScan(720);
  big.angle_increment = 1e-9;   // valid size, absurd increment, inconsistent span
  adversarial.push_back(big);

  GateSupervisor sup{SupervisorParams{}};
  double ros = 100.0, steady = 1000.0;
  double worst_cb = 0.0, worst_tick_gap = 0.0;
  runBounded([&] {
      auto last_tick = Clock::now();
      for (int i = 0; i < 400; ++i) {
        sup.onCommand({0.3, 0.5}, steady);
        if (i % 2 == 0) {     // scans at half the tick rate
          const auto t0 = Clock::now();
          sup.onScan(adversarial[static_cast<std::size_t>(i / 2) % adversarial.size()], ros,
          ros, steady);
          worst_cb = std::max(worst_cb,
          std::chrono::duration<double>(Clock::now() - t0).count());
        }
        const auto now = Clock::now();
        worst_tick_gap = std::max(worst_tick_gap,
        std::chrono::duration<double>(now - last_tick).count());
        last_tick = now;
        const auto r = sup.tick(ros, steady);
        ASSERT_DOUBLE_EQ(r.output.velocity.linear, 0.0) << "tick " << i;
        ASSERT_DOUBLE_EQ(r.output.velocity.angular, 0.0) << "tick " << i;
        ros += kTickPeriodS;
        steady += kTickPeriodS;
      }
    }, 5.0, "adversarial scan stream");
  RecordProperty("worst_scan_callback_us", static_cast<int>(worst_cb * 1e6));
  RecordProperty("worst_tick_gap_us", static_cast<int>(worst_tick_gap * 1e6));
  EXPECT_LT(worst_cb, kScanCallbackBudgetS);
  EXPECT_LT(worst_tick_gap, kTickPeriodS);
}

TEST(V31ScanWorkload, ScanTimeoutStillStopsWhenOnlyMalformedScansArrive)
{
  GateSupervisor sup{SupervisorParams{}};
  double ros = 100.0, steady = 1000.0;
  sup.onScan(fullScan(), ros, ros, steady);
  sup.onCommand({0.3, 0.0}, steady);
  ASSERT_EQ(sup.tick(ros, steady).output.state, GateState::kPass);
  sup.onScan(degenerateScan(), ros, ros, steady);
  for (int i = 0; i < 20; ++i) {
    ros += 0.1;
    steady += 0.1;
    sup.onCommand({0.3, 0.0}, steady);
    const auto r = sup.tick(ros, steady);
    EXPECT_NE(r.output.state, GateState::kPass);
    EXPECT_DOUBLE_EQ(r.output.velocity.linear, 0.0);
  }
}
