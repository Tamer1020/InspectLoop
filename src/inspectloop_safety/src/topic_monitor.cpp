// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#include "inspectloop_safety/topic_monitor.hpp"

#include <cmath>
#include <stdexcept>

namespace inspectloop_safety
{

std::string toString(StreamHealth h)
{
  switch (h) {
    case StreamHealth::kOk: return "OK";
    case StreamHealth::kDegraded: return "DEGRADED";
    case StreamHealth::kStale: return "STALE";
    case StreamHealth::kNeverReceived: return "NEVER_RECEIVED";
    case StreamHealth::kClockJump: return "CLOCK_JUMP";
  }
  return "UNKNOWN";
}

TopicMonitor::TopicMonitor(double timeout_s, double min_rate_hz, double window_s)
: timeout_s_(timeout_s), min_rate_hz_(min_rate_hz), window_s_(window_s)
{
  if (timeout_s <= 0.0 || window_s <= 0.0 || min_rate_hz < 0.0) {
    throw std::invalid_argument("TopicMonitor: timeout and window must be > 0, min rate >= 0");
  }
}

void TopicMonitor::recordMessage(double now_s)
{
  if (last_ && now_s < *last_) {
    // Time went backwards (simulation reset): discard history.
    reset();
  }
  stamps_.push_back(now_s);
  last_ = now_s;
  ++total_;
  prune(now_s);
}

void TopicMonitor::prune(double now_s) const
{
  while (!stamps_.empty() && stamps_.front() < now_s - window_s_) {
    stamps_.pop_front();
  }
}

double TopicMonitor::rateHz(double now_s) const
{
  prune(now_s);
  if (stamps_.size() < 2) {
    return 0.0;
  }
  const double span = stamps_.back() - stamps_.front();
  if (span <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(stamps_.size() - 1) / span;
}

std::optional<double> TopicMonitor::ageS(double now_s) const
{
  if (!last_) {
    return std::nullopt;
  }
  return now_s - *last_;
}

StreamHealth TopicMonitor::evaluate(double now_s) const
{
  if (!last_) {
    return StreamHealth::kNeverReceived;
  }
  if (!std::isfinite(now_s) || now_s < *last_ - 1e-6) {
    return StreamHealth::kClockJump;               // fail closed until fresh data arrives
  }
  if (now_s - *last_ > timeout_s_) {
    return StreamHealth::kStale;
  }
  // Need at least a few samples before judging the rate.
  if (min_rate_hz_ > 0.0 && stamps_.size() >= 3 && rateHz(now_s) < min_rate_hz_) {
    return StreamHealth::kDegraded;
  }
  return StreamHealth::kOk;
}

void TopicMonitor::reset()
{
  stamps_.clear();
  last_.reset();
}

}  // namespace inspectloop_safety
