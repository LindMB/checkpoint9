#include "attach_shelf/pre_approach.h"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/qos.hpp"

PreApproach::PreApproach(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::RELIABLE);

  this->laser_scan_sub_ =
      this->rclcpp::create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos, std::bind(&PreApproach::laser_scan_clbk_, this));

  this->cmd_vel_unstamped_pub =
      this->rclcpp::create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 10);

  auto timer_period = std::schrono::milliseconds(100);
  this->cmd_vel_unstamped_pub_timer_ = this->rclcpp::create_wall_timer(
      timer_period,
      std::bind(&PreApproach::cmd_vel_unstamped_pub_timer_clbk_, this));
}

void PreApproach::laser_scan_clbk_(
    const sensor_msgs::msg::LaserScan::SharedPtr &msg) {}

void PreApproach::move_forward_() {}
void PreApproach::detect_obstacle_at_x_meters_(const double meters) {}
void PreApproach::stop_robot() {}
void PreApproach::rotate_of_x_degrees_(const double degrees) {}

void PreApproach::cmd_vel_unstamped_pub_timer_clbk_() {

  /// Move forward

  /// Detect the wall at x meters

  /// Stop

  /// Rotate of x degrees
}
