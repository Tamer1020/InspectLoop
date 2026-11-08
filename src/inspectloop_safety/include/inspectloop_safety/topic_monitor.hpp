// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>

namespace inspectloop_safety
{

/// Health level of a monitored data stream.
/// kClockJump: the evaluation time lies before the last message (simulation reset / backward
/// clock jump) - the stream must deliver a new message before it is trusted again.
enum class StreamHealth { kOk, kDegraded, kStale, kNeverReceived, kClockJump };

std::string toString(StreamHealth h);

/// Tracks message arrivals of one stream and classifies its health.
///
/// Time is passed explicitly (seconds, any monotonic clock such as ROS sim time)
/// so the logic is deterministic and unit-testable without ROS.
class TopicMonitor
{
public:
  /// @param timeout_s  stream is STALE when no message arrived for this long
  /// @param min_rate_hz stream is DEGRADED when the measured rate is below this (0 disables)
  /// @param window_s   averaging window for the rate estimate
  TopicMonitor(double timeout_s, double min_rate_hz, double window_s = 2.0);

  void recordMessage(double now_s);
  StreamHealth evaluate(double now_s) const;

  /// Messages per second over the window ending at now_s.
  double rateHz(double now_s) const;
  /// Seconds since the last message, empty if none received.
  std::optional<double> ageS(double now_s) const;
  std::size_t totalCount() const {return total_;}

  /// Restart the startup grace period (e.g. after the clock jumps backwards).
  void reset();

private:
  void prune(double now_s) const;

  double timeout_s_;
  double min_rate_hz_;
  double window_s_;
  mutable std::deque<double> stamps_;
  std::optional<double> last_;
  std::size_t total_{0};
};

}  // namespace inspectloop_safety
