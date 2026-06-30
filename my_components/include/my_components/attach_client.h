#ifndef MY_COMPONENTS__ATTACH_CLIENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_HPP_

#include "my_components/srv/go_to_loading.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {

public:
  using GoToLoading = my_components::srv::GoToLoading;

  explicit AttachClient(const rclcpp::NodeOptions &options);

  ~AttachClient() = default;

private:
  bool final_approach_;
  bool request_sent_;

  rclcpp::Client<GoToLoading>::SharedPtr attach_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback_();

  void handle_service_response_(
      const rclcpp::Client<GoToLoading>::SharedFuture result_future);
};

} // namespace my_components

#endif