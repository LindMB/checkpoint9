#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class PreApproach : public rclcpp::Node {

public:
  PreApproach(const std::string &node_name);

  void stop_robot();

  ~PreApproach() = default;

private:
  std::string node_name_;

  double obstacle_; // Distance from the wall (in m)
  int degrees_;     // Degrees to rotate the robot after stopping

  double previous_yam_;
  double accumulated_yaw_ = 0.0;
  double linear_x_vel = 0.5;
  double angular_z_vel = 0.5;

  bool first_odom_ = false;
  bool obstacle_detected_ = false;
  bool rotation_completed_ = false;
  bool pre_approach_completed_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_unstamped_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_unstamped_pub_timer_;

  void laser_scan_clbk_(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmd_vel_unstamped_pub_timer_clbk_();

  void move_forward_();
  void rotate_of_x_degrees_();
  bool is_obstacle_detected_at_x_meters_(
      const sensor_msgs::msg::LaserScan::SharedPtr msg);
};