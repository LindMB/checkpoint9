#include "my_components/attach_server.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"

auto approach_service_node = std::shared_ptr<ApproachService>();

void signal_handler(int /*signum*/) { // Intentionally unused

  // Check if the node has not been already destroyed
  // since there is no point in doing that after the final approach
  if (approach_service_node) {
    approach_service_node->stop_robot();
  }

  rclcpp::shutdown();
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  approach_service_node = std::make_shared<ApproachService>(options);

  executor.add_node(approach_service_node);

  // Register the signal handler for CTRL+C
  std::signal(SIGINT, signal_handler);

  rclcpp::Rate rate(10); // 10Hz = 1/10 sec = 0,1 sec = 100 ms

  while (rclcpp::ok() &&
         !approach_service_node->is_final_approach_completed()) {
    executor.spin_some();
    rate.sleep();
  }

  RCLCPP_INFO(approach_service_node->get_logger(),
              "Shutting down in 3 seconds...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  executor.remove_node(approach_service_node);
  // Destroy publishers/subscribers/timers BEFORE shutdown
  approach_service_node.reset();

  rclcpp::shutdown();
  return 0;
}
