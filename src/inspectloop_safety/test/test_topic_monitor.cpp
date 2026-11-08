// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <stdexcept>

#include "inspectloop_safety/topic_monitor.hpp"

using inspectloop_safety::StreamHealth;
using inspectloop_safety::TopicMonitor;

TEST(TopicMonitor, NeverReceivedInitially)
{
  TopicMonitor m(0.5, 5.0);
  EXPECT_EQ(m.evaluate(10.0), StreamHealth::kNeverReceived);
  EXPECT_FALSE(m.ageS(10.0).has_value());
}

TEST(TopicMonitor, HealthyAtNominalRate)
{
  TopicMonitor m(0.5, 5.0);
  for (int i = 0; i <= 20; ++i) {
    m.recordMessage(i * 0.1);
  }
  EXPECT_NEAR(m.rateHz(2.0), 10.0, 1e-6);
  EXPECT_EQ(m.evaluate(2.05), StreamHealth::kOk);
}

TEST(TopicMonitor, StaleAfterTimeout)
{
  TopicMonitor m(0.5, 5.0);
  for (int i = 0; i <= 10; ++i) {
    m.recordMessage(i * 0.1);
  }
  EXPECT_EQ(m.evaluate(1.4), StreamHealth::kOk);
  EXPECT_EQ(m.evaluate(1.6), StreamHealth::kStale);
  EXPECT_NEAR(*m.ageS(1.6), 0.6, 1e-9);
}

TEST(TopicMonitor, DegradedWhenRateTooLow)
{
  TopicMonitor m(1.0, 5.0);
  for (int i = 0; i <= 10; ++i) {
    m.recordMessage(i * 0.4);  // 2.5 Hz, below the 5 Hz minimum
  }
  EXPECT_EQ(m.evaluate(4.05), StreamHealth::kDegraded);
}

TEST(TopicMonitor, RecoversAfterStale)
{
  TopicMonitor m(0.5, 0.0);
  m.recordMessage(0.0);
  EXPECT_EQ(m.evaluate(1.0), StreamHealth::kStale);
  m.recordMessage(1.0);
  EXPECT_EQ(m.evaluate(1.1), StreamHealth::kOk);
}

TEST(TopicMonitor, ClockJumpBackResetsHistory)
{
  TopicMonitor m(0.5, 5.0);
  for (int i = 0; i < 10; ++i) {
    m.recordMessage(100.0 + i * 0.1);
  }
  m.recordMessage(1.0);  // simulation restarted
  EXPECT_EQ(m.evaluate(1.05), StreamHealth::kOk);
  EXPECT_NEAR(*m.ageS(1.05), 0.05, 1e-9);
  EXPECT_EQ(m.totalCount(), 11u);
}

TEST(TopicMonitor, RejectsInvalidConfig)
{
  EXPECT_THROW(TopicMonitor(0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(TopicMonitor(1.0, -1.0), std::invalid_argument);
}
