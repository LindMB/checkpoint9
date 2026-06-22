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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_unstamped_pub_timer_;

  void laser_scan_clbk_(const sensor_msgs::msg::LaserScan::SharedPtr &msg);
  void cmd_vel_unstamped_pub_timer_clbk_();

  void move_forward_();
  void detect_obstacle_at_x_meters_(const double meters);
  void rotate_of_x_degrees_(const double degrees);
};