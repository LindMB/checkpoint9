#include "attach_shelf/approach_service_server.h"
#include "geometry_msgs/msg/detail/transform_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <memory>
#include <vector>

ApproachService::ApproachService() : Node("approach_service") {

  std::string service_name = "/approach_shelf";

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laser_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos,
          std::bind(&ApproachService::laser_scan_clbk_, this,
                    std::placeholders::_1));

  this->approach_service_ = this->create_service<GoToLoading>(
      service_name, std::bind(&ApproachService::approach_service_clbk_, this,
                              std::placeholders::_1, std::placeholders::_2));

  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  auto tf_timer_period = std::chrono::milliseconds(100);
  this->tf_timer_ = this->create_wall_timer(
      tf_timer_period,
      std::bind(&ApproachService::publish_cart_frame_timer_clbk_, this));

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->process_approach_timer_ = this->create_wall_timer(
      tf_timer_period,
      std::bind(&ApproachService::process_approach_timer_clbk_, this));

  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&ApproachService::odom_callback_, this, std::placeholders::_1));

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

  if (indices_vect.empty()) {
    return {};
  }

  const size_t same_leg_group_threshold = 5;

  // Vector of group of indices indicating a leg detected
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
      leg_groups.push_back(current_group);
      // Empty the current group
      current_group.clear();
      // Add the index in the current group
      current_group.push_back(indices_vect[i]);
    }
  }

  // Add the current group to leg_groups
  // since it did not encounter another distant index to save it
  leg_groups.push_back(current_group);

  return leg_groups;
}

bool ApproachService::is_legs_center_computable_(
    const std::vector<std::vector<size_t>> &leg_groups) {

  //  If the laser only detects 1 shelf leg or none
  if (leg_groups.size() < 2) {
    return false;
  } else { // If the laser detects the 2 shelf legs (at least)
    return true;
  }
}

void ApproachService::compute_legs_center_(
    const std::vector<std::vector<size_t>> &leg_groups) {

  size_t leg_1_index;
  size_t leg_2_index;

  // Leg 1
  leg_1_index =
      leg_groups[0][leg_groups[0].size() / 2]; // Median index of the group

  double leg_1_angle = this->last_scan_->angle_min +
                       leg_1_index * this->last_scan_->angle_increment;
  double leg_1_ray_length = this->last_scan_->ranges[leg_1_index];

  // Leg 1 - Position (x, y)
  double leg_1_x, leg_1_y;
  leg_1_x = leg_1_ray_length * std::cos(leg_1_angle); // x = r * cos(theta)
  leg_1_y = leg_1_ray_length * std::sin(leg_1_angle); // y = r * sin(theta)

  // Leg 2
  leg_2_index =
      leg_groups[1][leg_groups[1].size() / 2]; // Median index of the group

  double leg_2_angle = this->last_scan_->angle_min +
                       leg_2_index * this->last_scan_->angle_increment;
  double leg_2_ray_length = this->last_scan_->ranges[leg_2_index];

  // Leg 2 - Position (x, y)
  double leg_2_x, leg_2_y;
  leg_2_x = leg_2_ray_length * std::cos(leg_2_angle); // x = r * cos(theta)
  leg_2_y = leg_2_ray_length * std::sin(leg_2_angle); // y = r * sin(theta)

  // Center point between both legs
  this->cart_x = (leg_1_x + leg_2_x) / 2.0;
  this->cart_y = (leg_1_y + leg_2_y) / 2.0;
}

void ApproachService::publish_cart_frame_timer_clbk_() {

  // Publish cart_frame
  // It is the TF between the laser frame and
  // the frame located at the central point between both shelf legs

  if (!this->legs_center_computable_) {
    return;
  }

  // Header
  this->cart_frame_tf_.header.stamp = this->now();
  this->cart_frame_tf_.header.frame_id = this->last_scan_->header.frame_id;

  // Child Frame ID
  this->cart_frame_tf_.child_frame_id = "cart_frame";

  // Transform
  this->cart_frame_tf_.transform.translation.x = cart_x;
  this->cart_frame_tf_.transform.translation.y = cart_y;
  this->cart_frame_tf_.transform.translation.z = 0.0;

  this->cart_frame_tf_.transform.rotation.w = 1.0;

  // Publish the transform
  this->tf_broadcaster_->sendTransform(this->cart_frame_tf_);

  // Indicate that the tf is available
  this->cart_frame_available_ = true;
}

void ApproachService::calculate_errors_robot_to_cart_frame_() {

  // If the transform is not published yet
  // Or if the robot has already reached cart_frame
  if (!this->cart_frame_available_ || this->cart_frame_reached_) {
    return;
  }

  /// Get the most recent transform between
  /// the robot frame (/robot_base_link) and the target frame (/cart_frame)

  geometry_msgs::msg::TransformStamped tf_stamped;

  try {
    // Get the position of Target Frame in the Reference Frame
    tf_stamped = this->tf_buffer_->lookupTransform(
        this->robot_frame_,  // Reference frame
        this->target_frame_, // Target frame
        tf2::TimePointZero   // Most recent transform
    );

  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN(this->get_logger(), "TF from %s to %s unavailable : %s",
                this->robot_frame_.c_str(), this->target_frame_.c_str(),
                e.what());
    return;
  }

  // Calculate the distance error
  double x = tf_stamped.transform.translation.x;
  double y = tf_stamped.transform.translation.y;

  this->error_dist_ = std::sqrt(x * x + y * y);
  this->error_yaw_ = std::atan2(y, x);
}

void ApproachService::move_robot_to_cart_frame_() {

  auto move_msg = geometry_msgs::msg::Twist();

  double error_threshold = 0.03; // 3 cm

  if (this->error_dist_ > error_threshold) {

    // Move forward slowly
    move_msg.linear.x = 0.25;
    // Do not rotate faster than 0.4m/s
    move_msg.angular.z =
        std::clamp(this->kp_yaw_ * this->error_yaw_, -0.4, 0.4);

  } else {
    move_msg.linear.x = 0.0;
    move_msg.angular.z = 0.0;

    this->cart_frame_reached_ = true;
  }

  this->cmd_vel_pub_->publish(move_msg);
}

void ApproachService::odom_callback_(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  // If the robot is not located at cart_frame...
  if (!this->cart_frame_reached_) {
    return;
  }

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double dist = std::sqrt(x * x + y * y);

  // For the first odom msg, just init previous_dist_
  if (this->first_odom_) {
    this->previous_dist_ = dist;
    this->first_odom_ = false;
    return;
  }

  // Calculate the variation distance
  double delta_dist = dist - this->previous_dist_;

  // Calculate the cumulated travelled distance of the robot in straight line
  this->accumulated_dist_ += std::abs(delta_dist);

  // Update previous_dist_
  this->previous_dist_ = dist;
}

void ApproachService::move_forward_() {

  auto move_msg = geometry_msgs::msg::Twist();

  move_msg.linear.x = 0.15;
  move_msg.angular.z = 0.0;

  this->cmd_vel_pub_->publish(move_msg);
}

void ApproachService::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist();

  this->cmd_vel_pub_->publish(stop_msg);
}

void ApproachService::process_approach_timer_clbk_() {

  // If it has NOT been requested to start the final approach...
  if (!this->start_final_approach_) {
    return;
  }

  calculate_errors_robot_to_cart_frame_();

  if (!this->cart_frame_reached_) {
    move_robot_to_cart_frame_();

  } else { // cart_frame has been reached already...

    // If distance under shelf not travelled yet...
    if (!this->dist_under_shelf_travelled_) {

      if (this->accumulated_dist_ < this->dist_to_move_under_shelf_) {
        move_forward_();

      } else {
        stop_robot();
        this->dist_under_shelf_travelled_ = true;

        this->start_final_approach_ = false; // Reset start_final_approach_
        this->accumulated_dist_ = 0.0;       // Reset accumulated_dist_

        // Reset first_odom_ since to force odom calculations when the robot is
        this->first_odom_ = true;
      }
    } else {
      // Do nothing
    }
  }
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

  for (size_t i = 0; i < this->last_scan_->intensities.size(); i++) {

    bool is_shelf_leg_detected = (this->last_scan_->intensities[i] > 8000);

    // If the ray intensity is different from inf, -inf and NAN and ...
    if (std::isfinite(this->last_scan_->ranges[i]) && is_shelf_leg_detected) {
      shelf_leg_detected_indices.push_back(i);
    }
  }

  // Identify detected entities with the laser scanner
  std::vector<std::vector<size_t>> leg_groups;
  leg_groups = identify_shelf_leg_index_groups_(shelf_leg_detected_indices);

  // Compute the central point between both shelf legs (if possible)
  this->legs_center_computable_ = is_legs_center_computable_(leg_groups);
  if (this->legs_center_computable_) {

    compute_legs_center_(leg_groups);

    // If it has been requested to start the final approach
    if (request->attach_to_shelf) {
      this->start_final_approach_ = true;
    } else {
      // Do nothing
    }

    // Shelf legs detected and request successfully processed
    response->complete = true;

  } else {
    response->complete = false;
    return;
  }
}

auto approach_service_node = std::shared_ptr<ApproachService>();

void signal_handler(int /*signum*/) { // Intentionally unused
  approach_service_node->stop_robot();
  rclcpp::shutdown();
}

int main(int argc, char **argc) {

  rclcpp::init(argc, argv);

  approach_service_node = std::make_shared<ApproachService>();

  // Register the signal handler for CTRL+C
  std::signal(SIGINT, signal_handler);

  rclcpp::spin(approach_service_node);
  return 0;
}