#include "attach_shelf/approach_service_server.h"

ApproachService::ApproachService() : Node("appraoch_service") {

  std::string service_name = "/approach_shelf";

  this->approach_service_ = this->create_service<GoToLoading>(
      service_name, std::bind(&ApproachService::approach_service_clbk_, this,
                              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "%s Approach Service Server Ready...",
              service_name.c_str());
}

void ApproachService::approach_service_clbk_(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Approach Service Requested...");

  
}