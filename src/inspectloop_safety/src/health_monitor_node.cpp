// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
//
// health_monitor: watches sensor streams (type-agnostic) and TF links and
// publishes standard diagnostic_msgs on /diagnostics once per second.
// Levels: OK, WARN (rate below minimum), ERROR (stale / never received / TF missing).

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "inspectloop_safety/topic_monitor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace inspectloop_safety
{

using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue p;
  p.key = k;
  p.value = v;
  return p;
}

std::string fmt(double v, int prec = 2)
{
  std::ostringstream o;
  o.setf(std::ios::fixed);
  o.precision(prec);
  o << v;
  return o.str();
}
}  // namespace

class HealthMonitorNode : public rclcpp::Node
{
  struct Stream
  {
    std::string topic;
    std::string type;
    std::unique_ptr<TopicMonitor> monitor;
    rclcpp::GenericSubscription::SharedPtr sub;
    StreamHealth last{StreamHealth::kNeverReceived};
  };

  struct TfCheck
  {
    std::string parent;
    std::string child;
    bool last_ok{false};
  };

public:
  HealthMonitorNode()
  : Node("health_monitor")
  {
    const auto topics = declare_parameter<std::vector<std::string>>("topics",
        {"/scan", "/odom", "/camera/image_raw"});
    const auto types = declare_parameter<std::vector<std::string>>("types",
        {"sensor_msgs/msg/LaserScan", "nav_msgs/msg/Odometry", "sensor_msgs/msg/Image"});
    const auto timeouts = declare_parameter<std::vector<double>>("timeouts_s", {0.5, 0.5, 1.0});
    const auto min_rates = declare_parameter<std::vector<double>>("min_rates_hz", {5.0, 10.0, 3.0});
    const auto tf_pairs = declare_parameter<std::vector<std::string>>("tf_checks",
        {"odom:base_footprint", "base_link:lidar_link", "base_link:camera_optical_link"});
    tf_max_age_s_ = declare_parameter("tf_max_age_s", 1.0);
    startup_grace_s_ = declare_parameter("startup_grace_s", 5.0);
    const double period = declare_parameter("publish_period_s", 1.0);

    if (topics.size() != types.size() || topics.size() != timeouts.size() ||
      topics.size() != min_rates.size())
    {
      throw std::invalid_argument("topics, types, timeouts_s and min_rates_hz must have equal size");
    }

    for (std::size_t i = 0; i < topics.size(); ++i) {
      auto s = std::make_shared<Stream>();
      s->topic = topics[i];
      s->type = types[i];
      s->monitor = std::make_unique<TopicMonitor>(timeouts[i], min_rates[i]);
      auto * raw = s.get();
      // SensorDataQoS (best effort) is compatible with both reliable and best-effort publishers.
      s->sub = create_generic_subscription(s->topic, s->type, rclcpp::SensorDataQoS(),
          [this, raw](std::shared_ptr<const rclcpp::SerializedMessage>) {
            raw->monitor->recordMessage(now().seconds());
          });
      streams_.push_back(s);
    }
    for (const auto & pair : tf_pairs) {
      const auto sep = pair.find(':');
      if (sep == std::string::npos) {
        throw std::invalid_argument("tf_checks entries must be 'parent:child', got " + pair);
      }
      tf_checks_.push_back({pair.substr(0, sep), pair.substr(sep + 1)});
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    start_time_ = now().seconds();
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(period),
        [this]() {publish();});
    RCLCPP_INFO(get_logger(), "monitoring %zu streams and %zu TF links", streams_.size(),
      tf_checks_.size());
  }

private:
  void publish()
  {
    const double t = now().seconds();
    if (t < start_time_) {  // clock reset
      start_time_ = t;
    }
    const bool in_grace = (t - start_time_) < startup_grace_s_;
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();

    for (auto & s : streams_) {
      DiagnosticStatus st;
      st.name = "inspectloop/stream" + s->topic;
      st.hardware_id = "inspectloop";
      const auto h = s->monitor->evaluate(t);
      const auto age = s->monitor->ageS(t);
      switch (h) {
        case StreamHealth::kOk: st.level = DiagnosticStatus::OK; break;
        case StreamHealth::kDegraded: st.level = DiagnosticStatus::WARN; break;
        case StreamHealth::kClockJump: st.level = DiagnosticStatus::ERROR; break;
        default: st.level = in_grace ? DiagnosticStatus::WARN : DiagnosticStatus::ERROR; break;
      }
      st.message = toString(h);
      st.values = {kv("type", s->type), kv("rate_hz", fmt(s->monitor->rateHz(t))),
        kv("age_s", age ? fmt(*age, 3) : "n/a"), kv("count", std::to_string(s->monitor->totalCount()))};
      if (h != s->last) {
        if (h == StreamHealth::kOk) {
          RCLCPP_INFO(get_logger(), "%s: %s", s->topic.c_str(), st.message.c_str());
        } else if (!in_grace) {
          RCLCPP_ERROR(get_logger(), "%s: %s (age %s s)", s->topic.c_str(), st.message.c_str(),
            age ? fmt(*age, 2).c_str() : "n/a");
        }
        s->last = h;
      }
      arr.status.push_back(st);
    }

    for (auto & c : tf_checks_) {
      DiagnosticStatus st;
      st.name = "inspectloop/tf/" + c.parent + "->" + c.child;
      st.hardware_id = "inspectloop";
      bool ok = false;
      try {
        const auto tf = tf_buffer_->lookupTransform(c.parent, c.child, tf2::TimePointZero);
        const double stamp = rclcpp::Time(tf.header.stamp).seconds();
        // static transforms carry stamp 0 and never expire
        const double age = stamp == 0.0 ? 0.0 : t - stamp;
        ok = age <= tf_max_age_s_;
        st.message = ok ? "OK" : "STALE";
        st.values = {kv("age_s", fmt(age, 3)),
          kv("xyz", fmt(tf.transform.translation.x) + " " + fmt(tf.transform.translation.y) +
          " " + fmt(tf.transform.translation.z))};
      } catch (const tf2::TransformException & e) {
        st.message = "UNAVAILABLE";
        st.values = {kv("error", e.what())};
      }
      st.level = ok ? DiagnosticStatus::OK :
        (in_grace ? DiagnosticStatus::WARN : DiagnosticStatus::ERROR);
      if (ok != c.last_ok && (ok || !in_grace)) {
        RCLCPP_INFO(get_logger(), "TF %s -> %s: %s", c.parent.c_str(), c.child.c_str(),
          st.message.c_str());
        c.last_ok = ok;
      }
      arr.status.push_back(st);
    }
    pub_->publish(arr);
  }

  std::vector<std::shared_ptr<Stream>> streams_;
  std::vector<TfCheck> tf_checks_;
  double tf_max_age_s_{1.0};
  double startup_grace_s_{5.0};
  double start_time_{0.0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inspectloop_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspectloop_safety::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
