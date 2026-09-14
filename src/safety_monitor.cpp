#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

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
  : Node("safety_monitor"), stop_latched_(false), last_minimum_(INFINITY)
  {
    stop_distance_ = declare_parameter<double>("stop_distance", 0.45);
    release_distance_ = declare_parameter<double>("release_distance", 0.60);
    const double field_of_view_degrees = declare_parameter<double>("field_of_view_degrees", 60.0);
    constexpr double pi = 3.14159265358979323846;
    half_field_of_view_rad_ = field_of_view_degrees * pi / 360.0;

    if (stop_distance_ <= 0.0 || release_distance_ <= stop_distance_) {
      throw std::invalid_argument("release_distance must be greater than stop_distance > 0");
    }

    auto stop_qos = rclcpp::QoS(1).reliable().transient_local();
    stop_publisher_ = create_publisher<std_msgs::msg::Bool>("/safety/stop", stop_qos);
    clearance_publisher_ = create_publisher<std_msgs::msg::Float32>("/safety/min_clearance", 10);

    auto scan_qos = rclcpp::SensorDataQoS();
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos, std::bind(&SafetyMonitor::scan_callback, this, _1));

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/safety/reset", std::bind(&SafetyMonitor::reset_callback, this, _1, _2));

    publish_stop_state();
    RCLCPP_INFO(
      get_logger(), "Ready: stop=%.2f m, release=%.2f m, FOV=%.1f deg",
      stop_distance_, release_distance_, field_of_view_degrees);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    last_minimum_ = robot_safety_monitor::minimum_range_in_sector(
      scan->ranges, scan->angle_min, scan->angle_increment,
      scan->range_min, scan->range_max, half_field_of_view_rad_);

    std_msgs::msg::Float32 clearance_message;
    clearance_message.data = last_minimum_;
    clearance_publisher_->publish(clearance_message);

    if (last_minimum_ <= stop_distance_ && !stop_latched_) {
      stop_latched_ = true;
      RCLCPP_WARN(get_logger(), "SAFETY STOP: obstacle at %.3f m", last_minimum_);
      publish_stop_state();
    }
  }

  void reset_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!stop_latched_) {
      response->success = true;
      response->message = "Safety stop is already clear.";
      return;
    }

    if (last_minimum_ < release_distance_) {
      response->success = false;
      response->message = "Reset rejected: obstacle remains inside release distance.";
      return;
    }

    stop_latched_ = false;
    publish_stop_state();
    response->success = true;
    response->message = "Safety stop cleared.";
    RCLCPP_INFO(get_logger(), "Safety stop reset by service request");
  }

  void publish_stop_state()
  {
    std_msgs::msg::Bool message;
    message.data = stop_latched_;
    stop_publisher_->publish(message);
  }

  bool stop_latched_;
  float last_minimum_;
  double stop_distance_;
  double release_distance_;
  double half_field_of_view_rad_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr clearance_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
