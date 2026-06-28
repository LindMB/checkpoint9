#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {

public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

  void stop_robot();
  bool is_pre_approach_completed() const;

  ~PreApproach() = default;

private:
  double obstacle_; // Distance from the wall (in m)
  int degrees_;     // Degrees to rotate the robot after stopping

  double previous_yaw_;
  double accumulated_yaw_ = 0.0;
  double linear_x_vel = 0.2;
  double angular_z_vel = 0.4;

  bool first_odom_ = true;
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

} // namespace my_components