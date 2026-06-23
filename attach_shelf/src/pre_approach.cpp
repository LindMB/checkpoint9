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

  this->odom_sub_ = this->rclcpp::create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos, std::bind(&PreApproach::odom_callback_, this));

  this->cmd_vel_unstamped_pub_ =
      this->rclcpp::create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 10);

  auto timer_period = std::schrono::milliseconds(100);
  this->cmd_vel_unstamped_pub_timer_ = this->rclcpp::create_wall_timer(
      timer_period,
      std::bind(&PreApproach::cmd_vel_unstamped_pub_timer_clbk_, this));
}

void PreApproach::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {

}

void PreApproach::laser_scan_clbk_(
    const sensor_msgs::msg::LaserScan::SharedPtr &msg) {

  if (!this->obstacle_detected_) {
    if (is_obstacle_detected_at_x_meters_(this->meters_from_wall_, msg)) {
      this->obstacle_detected_ = true;
    } else {
      // Do nothing
    }
  }
}

void PreApproach::move_forward_() {

  auto move_forward = geometry_msgs::msgs::Twist();
  move_forward.linear.x = 0.5;
  move_forward.angular.z = 0.0;

  cmd_vel_unstamped_pub_.publish(move_forward);
}

void PreApproach::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist();

  cmd_vel_unstamped_pub_.publish(stop_msg);
}

bool PreApproach::is_obstacle_detected_at_x_meters_(const double meters) {}

void PreApproach::rotate_of_x_degrees_(const double degrees) {}

void PreApproach::cmd_vel_unstamped_pub_timer_clbk_() {

  /// Move forward
  move_forward_();

  /// Detect the wall at x meters
  if (this->obstacle_detected_) {

    /// Stop
    stop_robot();

    /// Rotate of x degrees
  }
}
