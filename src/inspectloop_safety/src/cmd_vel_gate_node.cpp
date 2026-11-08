// Copyright 2026 Tamer1020
// SPDX-License-Identifier: Apache-2.0
//
// cmd_vel_gate: last-line safety filter between Nav2 and the drive (simulation guard, NOT a
// certified safety function).
//   /cmd_vel (Nav2 / teleop) ──► [gate] ──► /cmd_vel_safe (Gazebo DiffDrive)
//
// The robot is only allowed to move while ALL of these hold (fail closed otherwise):
//  * a LaserScan arrived within scan_timeout_s of *steady* (monotonic) time;
//  * that scan is structurally valid and has enough usable beams (assessScan);
//  * its header stamp is plausible w.r.t. the ROS clock (not zero, not in the future, not stale);
//  * no backward ROS-clock jump happened since that scan (simulation reset);
//  * a velocity command arrived within cmd_timeout_s of steady time (deadman);
//  * no e-stop is latched, and the forward corridor is clear.
// The control loop runs on a wall timer so zero velocities keep being published even if the
// simulation clock stops.

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "inspectloop_safety/velocity_gate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace inspectloop_safety
{

using SteadyClock = std::chrono::steady_clock;

class CmdVelGateNode : public rclcpp::Node
{
public:
  CmdVelGateNode()
  : Node("cmd_vel_gate")
  {
    scan_timeout_s_ = declare_parameter("scan_timeout_s", 0.5);
    scan_max_stamp_age_s_ = declare_parameter("scan_max_stamp_age_s", 1.0);
    scan_max_future_s_ = declare_parameter("scan_max_future_s", 0.2);
    min_usable_fraction_ = declare_parameter("scan_min_usable_fraction", 0.5);
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
        last_cmd_steady_ = SteadyClock::now();
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        onScan(*msg);
      });
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_estop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        estop_ = req->data;
        res->success = true;
        res->message = estop_ ? "e-stop latched" : "e-stop released";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "cmd_vel_gate ready: v<=%.2f w<=%.2f stop<%.2fm (fail-closed)",
      limits_.max_linear, limits_.max_angular, limits_.stop_distance_m);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan & msg)
  {
    ScanData d;
    d.ranges = msg.ranges;
    d.angle_min = msg.angle_min;
    d.angle_max = msg.angle_max;
    d.angle_increment = msg.angle_increment;
    d.range_min = msg.range_min;
    d.range_max = msg.range_max;
    const auto a = assessScan(d, corridor_half_width_, front_offset_, min_usable_fraction_);
    const double stamp = rclcpp::Time(msg.header.stamp, RCL_ROS_TIME).seconds();
    const auto sc = checkStamp(stamp, now().seconds(), scan_max_stamp_age_s_, scan_max_future_s_);
    last_scan_steady_ = SteadyClock::now();
    scan_ok_ = a.validity == ScanValidity::kValid && sc == StampCheck::kOk;
    front_clearance_ = a.front_clearance_m;
    scan_reason_ = a.validity != ScanValidity::kValid ? "scan " + toString(a.validity) :
      (sc != StampCheck::kOk ? "scan stamp " + toString(sc) : "");
    last_scan_stamp_ = stamp;
  }

  void tick()
  {
    const auto t_steady = SteadyClock::now();
    const double t_ros = now().seconds();
    // backward ROS-clock jump (simulation reset): distrust everything received before it
    if (last_tick_ros_ && t_ros < *last_tick_ros_ - 1e-3) {
      RCLCPP_WARN(get_logger(), "ROS clock jumped back %.3f -> %.3f s: invalidating scan state",
        *last_tick_ros_, t_ros);
      scan_ok_ = false;
      scan_reason_ = "clock jumped backwards";
      last_scan_steady_.reset();
    }
    last_tick_ros_ = t_ros;

    std::string reason = scan_reason_;
    bool scan_healthy = scan_ok_ && last_scan_steady_.has_value();
    if (!last_scan_steady_) {
      reason = reason.empty() ? "no scan" : reason;
    } else {
      const double scan_age = std::chrono::duration<double>(t_steady - *last_scan_steady_).count();
      if (scan_age > scan_timeout_s_) {
        scan_healthy = false;
        reason = "scan timeout";
      }
    }

    GateInputs in;
    in.command = last_cmd_;
    in.command_age_s = last_cmd_steady_ ?
      std::chrono::duration<double>(t_steady - *last_cmd_steady_).count() :
      std::numeric_limits<double>::infinity();
    in.scan_healthy = scan_healthy;
    in.min_front_range_m = front_clearance_;
    in.estop = estop_;

    const GateOutput out = applyGate(in, limits_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = out.velocity.linear;
    twist.angular.z = out.velocity.angular;
    pub_->publish(twist);

    if (out.state != last_state_) {
      std::string s = toString(out.state);
      if (out.state == GateState::kStoppedScanFault && !reason.empty()) {
        s += " (" + reason + ")";
      }
      const bool is_fault = out.state == GateState::kStoppedScanFault ||
        out.state == GateState::kStoppedEstop || out.state == GateState::kBlockedObstacle;
      if (is_fault) {
        RCLCPP_WARN(get_logger(), "gate -> %s (front clearance %.2f m)", s.c_str(),
          front_clearance_);
      } else {
        RCLCPP_INFO(get_logger(), "gate -> %s", s.c_str());
      }
      std_msgs::msg::String m;
      m.data = toString(out.state);
      state_pub_->publish(m);
      last_state_ = out.state;
    }
  }

  GateLimits limits_;
  double scan_timeout_s_{0.5};
  double scan_max_stamp_age_s_{1.0};
  double scan_max_future_s_{0.2};
  double min_usable_fraction_{0.5};
  double corridor_half_width_{0.27};
  double front_offset_{0.15};
  Velocity last_cmd_;
  std::optional<SteadyClock::time_point> last_cmd_steady_;
  std::optional<SteadyClock::time_point> last_scan_steady_;
  std::optional<double> last_tick_ros_;
  double last_scan_stamp_{0.0};
  bool scan_ok_{false};
  std::string scan_reason_{"no scan"};
  double front_clearance_{std::numeric_limits<double>::quiet_NaN()};
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
