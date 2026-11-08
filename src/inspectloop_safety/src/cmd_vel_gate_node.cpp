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
//  * the sector swept by the commanded motion is OBSERVED (V3, Astra V2 review F3): forward
//    motion needs the forward stopping corridor covered, reverse motion the rear corridor, an
//    in-place turn the whole circle. NaN / out-of-range / unsampled directions are unknown, never
//    free space. An unobserved direction zeroes that motion component (BLOCKED_UNOBSERVED);
//  * its header stamp is plausible w.r.t. the ROS clock (not zero, not in the future, not stale);
//  * no backward ROS-clock jump happened since that scan (simulation reset);
//  * a velocity command arrived within cmd_timeout_s of steady time (deadman);
//  * no e-stop is latched, and the forward corridor is clear.
// The control loop runs on a wall timer so zero velocities keep being published even if the
// simulation clock stops.

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "inspectloop_safety/gate_supervisor.hpp"
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
  : Node("cmd_vel_gate"), sup_(loadParams())
  {
    const double rate = declare_parameter("publish_rate_hz", 20.0);
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_safe", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "~/state", rclcpp::QoS(1).transient_local());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        sup_.onCommand({msg->linear.x, msg->angular.z}, steadyNow());
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        ScanData d;
        d.ranges = msg->ranges;
        d.angle_min = msg->angle_min;
        d.angle_max = msg->angle_max;
        d.angle_increment = msg->angle_increment;
        d.range_min = msg->range_min;
        d.range_max = msg->range_max;
        sup_.onScan(d, rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).seconds(), now().seconds(),
          steadyNow());
      });
    estop_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_estop", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        sup_.setEstop(req->data);
        res->success = true;
        res->message = req->data ? "e-stop latched" : "e-stop released";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      });

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
    const auto & p = sup_.params();
    RCLCPP_INFO(get_logger(), "cmd_vel_gate ready: v<=%.2f w<=%.2f stop<%.2fm; sectors fwd "
      "+-%.1f deg rear +-%.1f deg, coverage>=%.2f gap<=%.1f deg (fail-closed)",
      p.limits.max_linear, p.limits.max_angular, p.limits.stop_distance_m,
      p.coverage.forwardHalfAngle() * 180.0 / M_PI, p.coverage.rearHalfAngle() * 180.0 / M_PI,
      p.coverage.min_sector_coverage, p.coverage.max_sector_gap_rad * 180.0 / M_PI);
  }

private:
  static double steadyNow()
  {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
  }

  SupervisorParams loadParams()
  {
    constexpr double kDeg = M_PI / 180.0;
    SupervisorParams p;
    p.scan_timeout_s = declare_parameter("scan_timeout_s", 0.5);
    p.scan_max_stamp_age_s = declare_parameter("scan_max_stamp_age_s", 1.0);
    p.scan_max_future_s = declare_parameter("scan_max_future_s", 0.2);
    p.limits.max_linear = declare_parameter("max_linear", 0.45);
    p.limits.max_angular = declare_parameter("max_angular", 1.3);
    p.limits.cmd_timeout_s = declare_parameter("cmd_timeout_s", 0.5);
    p.limits.stop_distance_m = declare_parameter("stop_distance_m", 0.20);
    p.limits.slow_distance_m = declare_parameter("slow_distance_m", 0.45);
    auto & c = p.coverage;
    c.min_usable_fraction = declare_parameter("scan_min_usable_fraction", 0.5);
    c.corridor_half_width_m = declare_parameter("corridor_half_width_m", 0.27);
    c.front_offset_m = declare_parameter("lidar_to_front_m", 0.15);
    c.rear_offset_m = declare_parameter("lidar_to_rear_m", 0.35);
    c.sensor_x_m = declare_parameter("lidar_x_m", 0.10);
    c.rotation_radius_m = declare_parameter("rotation_radius_m", 0.35);
    c.forward_half_angle_rad = declare_parameter("forward_sector_half_angle_deg", 0.0) * kDeg;
    c.rear_half_angle_rad = declare_parameter("rear_sector_half_angle_deg", 0.0) * kDeg;
    c.min_sector_coverage = declare_parameter("sector_min_coverage", 0.95);
    c.max_sector_gap_rad = declare_parameter("sector_max_gap_deg", 2.0) * kDeg;
    c.max_angle_increment_rad = declare_parameter("scan_max_angle_increment_deg", 1.0) * kDeg;
    return p;
  }

  void tick()
  {
    const auto r = sup_.tick(now().seconds(), steadyNow());
    if (r.clock_jump) {
      RCLCPP_WARN(get_logger(), "ROS clock jumped backwards: scan state invalidated");
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = r.output.velocity.linear;
    twist.angular.z = r.output.velocity.angular;
    pub_->publish(twist);

    if (r.output.state != last_state_ || r.reason != last_reason_) {
      const bool changed_state = r.output.state != last_state_;
      std::string s = toString(r.output.state);
      if ((r.output.state == GateState::kStoppedScanFault ||
        r.output.state == GateState::kBlockedUnobserved) && !r.reason.empty())
      {
        s += " (" + r.reason + ")";
      }
      const bool is_fault = r.output.state == GateState::kStoppedScanFault ||
        r.output.state == GateState::kStoppedEstop ||
        r.output.state == GateState::kBlockedObstacle ||
        r.output.state == GateState::kBlockedUnobserved;
      const auto & a = sup_.lastAssessment();
      if (is_fault) {
        RCLCPP_WARN(get_logger(), "gate -> %s (front clearance %.2f m, fwd coverage %.2f gap "
          "%.1f deg)", s.c_str(), a.front_clearance_m, a.forward.coverage,
          a.forward.max_gap_rad * 180.0 / M_PI);
      } else if (changed_state) {
        RCLCPP_INFO(get_logger(), "gate -> %s", s.c_str());
      }
      if (changed_state) {
        std_msgs::msg::String m;
        m.data = toString(r.output.state);
        state_pub_->publish(m);
      }
      last_state_ = r.output.state;
      last_reason_ = r.reason;
    }
  }

  GateSupervisor sup_;
  GateState last_state_{GateState::kPass};
  std::string last_reason_;

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
