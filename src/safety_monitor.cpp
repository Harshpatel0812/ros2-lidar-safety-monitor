// Copyright 2026 Harsh Patel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_safety_monitor/safety_logic.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor()
  : Node("safety_monitor"),
    stop_latched_(true),
    received_scan_(false),
    last_minimum_(std::numeric_limits<float>::infinity()),
    commanded_forward_speed_(0.0)
  {
    stop_distance_ =
      declare_parameter<double>("stop_distance", 0.45);

    release_distance_ =
      declare_parameter<double>("release_distance", 0.60);

    const double field_of_view_degrees =
      declare_parameter<double>("field_of_view_degrees", 60.0);

    scan_timeout_ =
      declare_parameter<double>("scan_timeout", 0.50);

    reaction_time_ =
      declare_parameter<double>("reaction_time", 0.25);

    braking_deceleration_ =
      declare_parameter<double>("braking_deceleration", 0.80);

    max_stop_distance_ =
      declare_parameter<double>("max_stop_distance", 1.50);

    constexpr double pi = 3.14159265358979323846;

    half_field_of_view_rad_ =
      field_of_view_degrees * pi / 360.0;

    if (stop_distance_ <= 0.0) {
      throw std::invalid_argument(
        "stop_distance must be greater than zero");
    }

    if (release_distance_ <= stop_distance_) {
      throw std::invalid_argument(
        "release_distance must be greater than stop_distance");
    }

    if (scan_timeout_ <= 0.0) {
      throw std::invalid_argument(
        "scan_timeout must be greater than zero");
    }

    if (reaction_time_ < 0.0) {
      throw std::invalid_argument(
        "reaction_time must not be negative");
    }

    if (braking_deceleration_ <= 0.0) {
      throw std::invalid_argument(
        "braking_deceleration must be greater than zero");
    }

    if (max_stop_distance_ < stop_distance_) {
      throw std::invalid_argument(
        "max_stop_distance must be greater than or equal to "
        "stop_distance");
    }

    auto state_qos =
      rclcpp::QoS(1).reliable().transient_local();

    stop_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
        "/safety/stop",
        state_qos);

    clearance_publisher_ =
      create_publisher<std_msgs::msg::Float32>(
        "/safety/min_clearance",
        10);

    active_stop_distance_publisher_ =
      create_publisher<std_msgs::msg::Float32>(
        "/safety/active_stop_distance",
        state_qos);

    scan_subscription_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(
          &SafetyMonitor::scan_callback,
          this,
          _1));

    velocity_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_raw",
        10,
        std::bind(
          &SafetyMonitor::velocity_callback,
          this,
          _1));

    reset_service_ =
      create_service<std_srvs::srv::Trigger>(
        "/safety/reset",
        std::bind(
          &SafetyMonitor::reset_callback,
          this,
          _1,
          _2));

    watchdog_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(
          &SafetyMonitor::watchdog_callback,
          this));

    publish_stop_state();
    publish_active_stop_distance();

    RCLCPP_INFO(
      get_logger(),
      "Safety monitor ready: base stop=%.2f m, "
      "base release=%.2f m, FOV=%.1f deg, "
      "scan timeout=%.2f s",
      stop_distance_,
      release_distance_,
      field_of_view_degrees,
      scan_timeout_);

    RCLCPP_INFO(
      get_logger(),
      "Dynamic stopping: reaction=%.2f s, "
      "braking deceleration=%.2f m/s^2, "
      "maximum stop distance=%.2f m",
      reaction_time_,
      braking_deceleration_,
      max_stop_distance_);

    RCLCPP_WARN(
      get_logger(),
      "Safety stop is latched until valid lidar data arrives "
      "and the reset service is called.");
  }

private:
  double active_stop_distance() const
  {
    return robot_safety_monitor::calculate_dynamic_stop_distance(
      stop_distance_,
      commanded_forward_speed_,
      reaction_time_,
      braking_deceleration_,
      max_stop_distance_);
  }

  double active_release_distance() const
  {
    const double hysteresis_margin =
      release_distance_ - stop_distance_;

    return active_stop_distance() + hysteresis_margin;
  }

  void publish_active_stop_distance()
  {
    std_msgs::msg::Float32 message;

    message.data =
      static_cast<float>(active_stop_distance());

    active_stop_distance_publisher_->publish(message);
  }

  void velocity_callback(
    const geometry_msgs::msg::Twist::SharedPtr command)
  {
    commanded_forward_speed_ = command->linear.x;

    publish_active_stop_distance();

    if (
      scan_is_fresh() &&
      std::isfinite(last_minimum_) &&
      last_minimum_ <= active_stop_distance())
    {
      latch_stop(
        "Obstacle entered the dynamic stopping zone.");
    }
  }

  void scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    received_scan_ = true;
    last_scan_time_ = now();

    last_minimum_ =
      robot_safety_monitor::minimum_range_in_sector(
        scan->ranges,
        scan->angle_min,
        scan->angle_increment,
        scan->range_min,
        scan->range_max,
        half_field_of_view_rad_);

    std_msgs::msg::Float32 clearance_message;
    clearance_message.data = last_minimum_;
    clearance_publisher_->publish(clearance_message);

    publish_active_stop_distance();

    if (!std::isfinite(last_minimum_)) {
      latch_stop("No valid lidar ranges were found.");
      return;
    }

    if (last_minimum_ <= active_stop_distance()) {
      latch_stop(
        "Obstacle entered the dynamic stopping zone.");
    }
  }

  void watchdog_callback()
  {
    if (!scan_is_fresh()) {
      latch_stop("Lidar data timed out.");
    }
  }

  bool scan_is_fresh()
  {
    if (!received_scan_) {
      return false;
    }

    const double age =
      (now() - last_scan_time_).seconds();

    return age <= scan_timeout_;
  }

  void latch_stop(const std::string & reason)
  {
    if (stop_latched_) {
      return;
    }

    stop_latched_ = true;
    publish_stop_state();

    RCLCPP_ERROR(
      get_logger(),
      "SAFETY STOP: %s",
      reason.c_str());
  }

  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!scan_is_fresh()) {
      response->success = false;
      response->message =
        "Reset rejected: lidar data is missing or stale.";
      return;
    }

    if (!std::isfinite(last_minimum_)) {
      response->success = false;
      response->message =
        "Reset rejected: lidar contains no valid ranges.";
      return;
    }

    if (last_minimum_ < active_release_distance()) {
      response->success = false;
      response->message =
        "Reset rejected: obstacle remains inside "
        "the release distance.";
      return;
    }

    if (!stop_latched_) {
      response->success = true;
      response->message =
        "Safety stop is already clear.";
      return;
    }

    stop_latched_ = false;
    publish_stop_state();

    response->success = true;
    response->message = "Safety stop cleared.";

    RCLCPP_INFO(
      get_logger(),
      "Safety stop reset by service request.");
  }

  void publish_stop_state()
  {
    std_msgs::msg::Bool message;
    message.data = stop_latched_;
    stop_publisher_->publish(message);
  }

  bool stop_latched_;
  bool received_scan_;

  float last_minimum_;

  double commanded_forward_speed_;
  double stop_distance_;
  double release_distance_;
  double half_field_of_view_rad_;
  double scan_timeout_;
  double reaction_time_;
  double braking_deceleration_;
  double max_stop_distance_;

  rclcpp::Time last_scan_time_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    stop_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    clearance_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    active_stop_distance_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    velocity_subscription_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    reset_service_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
