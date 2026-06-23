#pragma once
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"

class PreApproach : public rclcpp::Node() {

public:
  PreApproach(const std::string &node_name);

  void stop_robot();

  ~PreApproach() = default;

private:
  std::string node_name_;

  const double meters_from_wall_;
  const double degrees_to_rotate_;

  double previous_yam_;
  double accumulated_yaw = 0.0;
  double linear_x_vel = 0.5;
  double angular_z_vel = 0.5;

  bool first_odom_ = false;
  bool rotation_completed_ = false;
  bool obstacle_detected_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_unstamped_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_unstamped_pub_timer_;

  void laser_scan_clbk_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmd_vel_unstamped_pub_timer_clbk_();

  void move_forward_();
  void rotate_of_x_degrees_(const double angle_deg);
  bool is_obstacle_detected_at_x_meters_(
      const double meters, const sensor_msgs::msg::LaserScan::SharedPtr msg);
};