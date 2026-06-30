#include "my_components/attach_client.h"
#include "rclcpp/client.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <chrono>
#include <cmath>
#include <memory>

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client_node", options) {

  this->final_approach_ = true;
  this->request_sent_ = false;

  auto timer_period = std::chrono::seconds(1);
  this->timer_ = this->create_wall_timer(
      timer_period, std::bind(&AttachClient::timer_callback_, this));

  const std::string service_name = "/approach_shelf";
  this->attach_client_ = this->create_client<GoToLoading>(service_name);

  RCLCPP_INFO(this->get_logger(),
              "AttachClient initialised | final_approach=%s",
              this->final_approach_ ? "true" : "false");
}

void AttachClient::timer_callback_() {

  // Check if a request has already been sent
  if (this->request_sent_) {
    return;
  }

  // Check if the service is available
  if (!this->attach_client_->service_is_ready()) {

    // Check if ROS is still working
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  auto request = std::make_shared<GoToLoading::Request>();

  request->attach_to_shelf = this->final_approach_;

  // Send the request asynchronously
  // and when a response is received, handle it with
  // handle_service_response_
  auto result_future = this->attach_client_->async_send_request(
      request, std::bind(&AttachClient::handle_service_response_, this,
                         std::placeholders::_1));

  this->request_sent_ = true;

  RCLCPP_INFO(this->get_logger(), "Request sent to /approach_shelf");
}

void AttachClient::handle_service_response_(
    const rclcpp::Client<GoToLoading>::SharedFuture result_future) {

  auto response = result_future.get();

  if (this->final_approach_) {

    if (response->complete) {

      RCLCPP_INFO(this->get_logger(), "Request response - Shelf legs detected "
                                      "and request successfully processed");

    } else {

      RCLCPP_INFO(this->get_logger(),
                  "Request response - Final approach not initiated");
    }

  } else {

    RCLCPP_INFO(this->get_logger(), "Final approach not initiated");
  }
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)