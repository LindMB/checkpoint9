#pragma once
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"

class PreApproach : public rclcpp::Node() {

public:
  PreApproach(const std::string &node_name);

  ~PreApproach() = default;

private:
  std::string node_name_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_pub;
  rclcpp::TimerBase::SharedPtr cmd_vel_unstamped_pub_timer_;

  void laser_scan_clbk(const sensor_msgs::msg::LaserScan::SharedPtr &msg);
  void cmd_vel_unstamped_pub_timer_clbk();
};