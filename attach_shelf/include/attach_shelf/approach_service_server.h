#pragma once

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>

class ApproachService : public rclcpp::Node {

public:
  using GoToLoading = attach_shelf::srv::GoToLoading;

  ApproachService();
  ~ApproachService() = default;

private:
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

  rclcpp::Service<GoToLoading>::SharedPtr approach_service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  void
  approach_service_clbk_(const std::shared_ptr<GoToLoading::Request> request,
                         const std::shared_ptr<GoToLoading::Response> response);
  void laser_scan_clbk_(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::vector<std::vector<size_t>>
  identify_shelf_leg_index_groups_(const std::vector<size_t> &indices_vect);

  bool
  is_legs_center_computable(const std::vector<std::vector<size_t>> &leg_groups);
  void compute_legs_center_(const std::vector<std::vector<size_t>> &leg_groups,
                            double &cart_x, double &cart_y);
};
