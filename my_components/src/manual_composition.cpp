#include "my_components/attach_server.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

auto attach_server_node = std::shared_ptr<my_components::AttachServer>();

void signal_handler(int /*signum*/) { // Intentionally unused

  // Check if the node has not been already destroyed
  // since there is no point in doing that after the final approach
  if (attach_server_node) {
    attach_server_node->stop_robot();
  }

  rclcpp::shutdown();
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  attach_server_node = std::make_shared<my_components::AttachServer>(options);

  executor.add_node(attach_server_node);

  // Register the signal handler for CTRL+C
  std::signal(SIGINT, signal_handler);

  rclcpp::Rate rate(10); // 10Hz = 1/10 sec = 0,1 sec = 100 ms

  while (rclcpp::ok() && !attach_server_node->is_final_approach_completed()) {
    executor.spin_some();
    rate.sleep();
  }

  RCLCPP_INFO(attach_server_node->get_logger(),
              "Shutting down in 3 seconds...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  executor.remove_node(attach_server_node);
  // Destroy publishers/subscribers/timers BEFORE shutdown
  attach_server_node.reset();

  rclcpp::shutdown();
  return 0;
}
