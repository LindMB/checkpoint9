#pragma once

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <string>

class ApproachService : public rclcpp::Node {

public:
  using GoToLoading = attach_shelf::srv::GoToLoading;

  ApproachService();
  ~ApproachService() = default;

private:
  double cart_x = 0.0;
  double cart_y = 0.0;
  double robot_to_cart_frame_dist_ = 0.0;

  bool legs_center_computable_ = false;
  bool cart_frame_available_ = false;
  bool start_final_approach_ = false;

  std::string robot_frame_ = "/robot_base_link";
  std::string target_frame_ = "/cart_frame";

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Service<GoToLoading>::SharedPtr approach_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  geometry_msgs::msg::TransformStamped cart_frame_tf_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr process_approach_timer_;

  void
  approach_service_clbk_(const std::shared_ptr<GoToLoading::Request> request,
                         const std::shared_ptr<GoToLoading::Response> response);
  void laser_scan_clbk_(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void publish_cart_frame_timer_clbk_();

  std::vector<std::vector<size_t>>
  identify_shelf_leg_index_groups_(const std::vector<size_t> &indices_vect);

  bool is_legs_center_computable_(
      const std::vector<std::vector<size_t>> &leg_groups);
  void compute_legs_center_(const std::vector<std::vector<size_t>> &leg_groups);

  void calculate_dist_robot_to_cart_frame_();

  void process_approach_timer_clbk_();
};
