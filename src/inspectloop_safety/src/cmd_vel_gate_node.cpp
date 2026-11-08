// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
//
// cmd_vel_gate: last-line safety filter between Nav2 and the drive.
//   /cmd_vel (Nav2 / teleop) ──► [gate] ──► /cmd_vel_safe (Gazebo DiffDrive)
// Stops the robot when the LiDAR stream fails, the command stream goes stale,
// an e-stop is latched, or an obstacle is inside the stop distance ahead.

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "inspectloop_safety/topic_monitor.hpp"
#include "inspectloop_safety/velocity_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace inspectloop_safety
{

class CmdVelGateNode : public rclcpp::Node
{
public:
  CmdVelGateNode()
  : Node("cmd_vel_gate"),
    scan_monitor_(declare_parameter("scan_timeout_s", 0.5), 0.0)
  {
    limits_.max_linear = declare_parameter("max_linear", 0.45);
    limits_.max_angular = declare_parameter("max_angular", 1.3);
    limits_.cmd_timeout_s = declare_parameter("cmd_timeout_s", 0.5);
    limits_.stop_distance_m = declare_parameter("stop_distance_m", 0.20);
    limits_.slow_distance_m = declare_parameter("slow_distance_m", 0.45);
    corridor_half_width_ = declare_parameter("corridor_half_width_m", 0.27);
    front_offset_ = declare_parameter("lidar_to_front_m", 0.15);
    const double rate = declare_parameter("publish_rate_hz", 20.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).transient_local());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        last_cmd_ = {msg->linear.x, msg->angular.z};
        last_cmd_time_ = now().seconds();
        have_cmd_ = true;
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        scan_monitor_.recordMessage(now().seconds());
        front_clearance_ = frontClearance(msg->ranges, msg->angle_min, msg->angle_increment,
          msg->range_min, msg->range_max, corridor_half_width_, front_offset_);
      });
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_estop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        estop_ = req->data;
        res->success = true;
        res->message = estop_ ? "e-stop latched" : "e-stop released";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      });

    timer_ = rclcpp::create_timer(this, get_clock(),
        std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "cmd_vel_gate ready: v<=%.2f w<=%.2f stop<%.2fm",
      limits_.max_linear, limits_.max_angular, limits_.stop_distance_m);
  }

private:
  void tick()
  {
    const double t = now().seconds();
    GateInputs in;
    in.command = last_cmd_;
    in.command_age_s = have_cmd_ ? t - last_cmd_time_ : 1e9;
    in.scan_healthy = scan_monitor_.evaluate(t) == StreamHealth::kOk;
    in.min_front_range_m = front_clearance_;
    in.estop = estop_;

    const GateOutput out = applyGate(in, limits_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = out.velocity.linear;
    twist.angular.z = out.velocity.angular;
    pub_->publish(twist);

    if (out.state != last_state_) {
      const auto s = toString(out.state);
      const bool is_fault = out.state == GateState::kStoppedScanFault ||
        out.state == GateState::kStoppedEstop || out.state == GateState::kBlockedObstacle;
      if (is_fault) {
        RCLCPP_WARN(get_logger(), "gate -> %s (front clearance %.2f m)", s.c_str(),
          front_clearance_);
      } else {
        RCLCPP_INFO(get_logger(), "gate -> %s", s.c_str());
      }
      std_msgs::msg::String m;
      m.data = s;
      state_pub_->publish(m);
      last_state_ = out.state;
    }
  }

  GateLimits limits_;
  TopicMonitor scan_monitor_;
  double corridor_half_width_{0.27};
  double front_offset_{0.15};
  Velocity last_cmd_;
  double last_cmd_time_{0.0};
  bool have_cmd_{false};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  bool estop_{false};
  GateState last_state_{GateState::kPass};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inspectloop_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspectloop_safety::CmdVelGateNode>());
  rclcpp::shutdown();
  return 0;
}
