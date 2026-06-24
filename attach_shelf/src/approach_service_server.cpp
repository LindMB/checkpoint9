#include "attach_shelf/approach_service_server.h"
#include <cmath>
#include <vector>

ApproachService::ApproachService() : Node("appraoch_service") {

  std::string service_name = "/approach_shelf";

  this->approach_service_ = this->create_service<GoToLoading>(
      service_name, std::bind(&ApproachService::approach_service_clbk_, this,
                              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "%s Approach Service Server Ready...",
              service_name.c_str());
}

void ApproachService::laser_scan_clbk_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  this->last_scan_ = msg;
}

std::vector<std::vector<size_t>>
ApproachService::identify_shelf_leg_index_groups_(
    const std::vector<size_t> &indices_vect) {

  const size_t same_leg_group_threshold = 5;

  // Vector of group of index indicating a leg detected
  std::vector<std::vector<size_t>> leg_groups;
  std::vector<size_t> current_group;

  current_group.push_back(indices_vect[0]);

  for (size_t i = 1; i < indices_vect.size(); i++) {

    bool is_index_in_same_leg_group =
        ((indices_vect[i] - indices_vect[i - 1]) < same_leg_group_threshold);

    if (is_index_in_same_leg_group) {
      current_group.push_back(indices_vect[i]);

    } else {
      // Add the current group to leg_groups
      leg_groups.push_back(current_group_vect);
      // Empty the current group
      current_group.clear();
      // Add the index in the current group
      current_group.push_back(indices_vect[i]);
    }
  }

  // Add the current group to leg_groups
  // since it did not encounter another distant index to save it
  leg_groups.push_back(current_group_vect);

  return leg_groups;
}

void ApproachService::approach_service_clbk_(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Approach Service Requested...");

  // If the laser scanner is not working
  if (!this->last_scan_) {
    response->complete = false;
    return;
  }

  std::vector<size_t> shelf_leg_detected_indices;

  for (size_t i = 0; this->last_scan_->intensities.size(); i++) {

    bool is_shelf_leg_detected = (this->last_scan_->intensities[i] > 8000);

    // If the ray intensity is different from inf, -inf and NAN and ...
    if (std::isfinite(this->last_scan_->ranges[i]) && is_shelf_leg_detected) {
      shelf_leg_detected_indices.push_back(i);
    }
  }

  std::vector<std::vector<size_t>> leg_groups;
  leg_groups = identify_shelf_leg_index_groups_(shelf_leg_detected_indices);
}