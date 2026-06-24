#pragma once

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include <memory>

class ApproachService : public rclcpp::Node {

public:
  using GoToLoading = attach_shelf::srv::GoToLoading;

  ApproachService();
  ~ApproachService() = default;

private:
  rclcpp::Service<GoToLoading>::SharedPtr approach_service_;

  void
  approach_service_clbk_(const std::shared_ptr<GoToLoading::Request> request,
                         const std::shared_ptr<GoToLoading::Response> response);
};
