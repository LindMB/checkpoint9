#include "my_components/attach_server.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <memory>
#include <vector>

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server_node", options) {

  std::string service_name = "/approach_shelf";

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laser_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos,
          std::bind(&AttachServer::laser_scan_clbk_, this,
                    std::placeholders::_1));

  this->approach_service_ = this->create_service<GoToLoading>(
      service_name, std::bind(&AttachServer::approach_service_clbk_, this,
                              std::placeholders::_1, std::placeholders::_2));

  this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  auto tf_timer_period = std::chrono::milliseconds(100);
  this->tf_timer_ = this->create_wall_timer(
      tf_timer_period,
      std::bind(&AttachServer::publish_cart_frame_timer_clbk_, this));

  this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->process_approach_timer_ = this->create_wall_timer(
      tf_timer_period,
      std::bind(&AttachServer::process_approach_timer_clbk_, this));

  this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diffbot_base_controller/cmd_vel_unstamped", 10);

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&AttachServer::odom_callback_, this, std::placeholders::_1));

  this->elevator_up_pub_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

  RCLCPP_INFO(this->get_logger(), "%s Approach Service Server Ready...",
              service_name.c_str());
}

void AttachServer::laser_scan_clbk_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  this->last_scan_ = msg;
}

std::vector<std::vector<size_t>> AttachServer::identify_shelf_leg_index_groups_(
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

bool AttachServer::is_legs_center_computable_(
    const std::vector<std::vector<size_t>> &leg_groups) {

  // If the laser only detects 1 shelf leg or none
  if (leg_groups.size() < 2) {
    return false;
  } else { // If the laser detects the 2 shelf legs (at least)
    return true;
  }
}

void AttachServer::compute_legs_center_(
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

void AttachServer::prepare_cart_frame_tf_() {

  // Create the central point between both legs in the LASER FRAME
  geometry_msgs::msg::PointStamped cart_point_laser_frame;

  cart_point_laser_frame.header.frame_id = this->last_scan_->header.frame_id;
  cart_point_laser_frame.header.stamp = this->last_scan_->header.stamp;

  cart_point_laser_frame.point.x = this->cart_x;
  cart_point_laser_frame.point.y = this->cart_y;
  cart_point_laser_frame.point.z = 0.0;

  // Create the central point between both legs in the ODOM FRAME
  geometry_msgs::msg::PointStamped cart_point_odom_frame;

  // Convert the central point from the laser frame to the odom frame
  try {
    cart_point_odom_frame = this->tf_buffer_->transform(
        cart_point_laser_frame, "odom",
        tf2::durationFromSec(1.0)); // Wait 1 sec for the TF to be available

  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform cart point to odom frame: %s", ex.what());
    return;
  }

  /// Update the TF of cart_frame into odom

  // Header
  // this->cart_frame_tf_.header.stamp will be filled before each tf publication
  this->cart_frame_tf_.header.frame_id = "odom";

  // Child Frame ID
  this->cart_frame_tf_.child_frame_id = "cart_frame";

  // Transform
  this->cart_frame_tf_.transform.translation.x = cart_point_odom_frame.point.x;
  this->cart_frame_tf_.transform.translation.y = cart_point_odom_frame.point.y;
  this->cart_frame_tf_.transform.translation.z = 0.0;

  this->cart_frame_tf_.transform.rotation.x = 0.0;
  this->cart_frame_tf_.transform.rotation.y = 0.0;
  this->cart_frame_tf_.transform.rotation.z = 0.0;
  this->cart_frame_tf_.transform.rotation.w = 1.0;

  this->cart_frame_tf_ready_ = true;
}

void AttachServer::publish_cart_frame_timer_clbk_() {

  // Publish cart_frame
  // It is the TF between the odom frame and
  // the frame located at the central point between both shelf legs
  if (!this->cart_frame_tf_ready_) {
    return;
  }

  // Update the stamp before publishing
  this->cart_frame_tf_.header.stamp = this->last_scan_->header.stamp;

  // Publish the transform
  this->tf_broadcaster_->sendTransform(this->cart_frame_tf_);

  // Indicate that the tf is available
  this->cart_frame_available_ = true;
}

bool AttachServer::is_error_robot_to_cart_frame_computable_() {

  // If the transform is not published yet
  // Or if the robot has already reached cart_frame
  if (!this->cart_frame_available_ || this->cart_frame_reached_) {
    return false;
  }

  /// Get the most recent transform between
  /// the robot frame (/robot_base_link) and the target frame (/cart_frame)

  geometry_msgs::msg::TransformStamped tf_stamped;

  try {
    // Get the position of Target Frame in the Reference Frame
    tf_stamped = this->tf_buffer_->lookupTransform(
        this->robot_frame_,  // Reference frame (robot_base_link)
        this->target_frame_, // Target frame (cart_frame)
        tf2::TimePointZero   // Most recent transform
    );

  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN(this->get_logger(), "TF from %s to %s unavailable : %s",
                this->robot_frame_.c_str(), this->target_frame_.c_str(),
                e.what());
    return false;
  }

  // Calculate the distance error
  double x = tf_stamped.transform.translation.x;
  double y = tf_stamped.transform.translation.y;

  this->error_dist_ = std::sqrt(x * x + y * y);
  this->error_yaw_ = std::atan2(y, x);

  return true;
}

void AttachServer::move_robot_to_cart_frame_() {

  auto move_msg = geometry_msgs::msg::Twist();

  double error_threshold = 0.04; // 4 cm

  if (this->error_dist_ > error_threshold) {

    // Move forward slowly
    move_msg.linear.x = 0.15;
    // Do not rotate faster than 0.4m/s
    move_msg.angular.z =
        std::clamp(this->kp_yaw_ * this->error_yaw_, -1.0, 1.0);

    RCLCPP_INFO(this->get_logger(), "Error Distance under shelf : %f",
                this->error_dist_);

  } else {
    move_msg.linear.x = 0.0;
    move_msg.angular.z = 0.0;

    this->cart_frame_reached_ = true;
  }

  this->cmd_vel_pub_->publish(move_msg);
}

void AttachServer::odom_callback_(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

  // If the robot is not located at cart_frame...
  if (!this->cart_frame_reached_) {
    return;
  }

  // Robot Current position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // For the first odom msg, just init (x, y) of the previous position
  if (this->first_odom_) {
    this->previous_x_ = x;
    this->previous_y_ = y;
    this->first_odom_ = false;
    return;
  }

  // Difference in x between the current and the previous robot position
  double dx = x - this->previous_x_;
  // Difference in y between the current and the previous robot position
  double dy = y - this->previous_y_;

  // Calculate the cumulated travelled distance of the robot between 2 positions
  this->accumulated_dist_ += std::sqrt(dx * dx + dy * dy);

  // Update (x, y) of the previous position
  this->previous_x_ = x;
  this->previous_y_ = y;
}

void AttachServer::move_forward_() {

  auto move_msg = geometry_msgs::msg::Twist();

  move_msg.linear.x = 0.15;
  move_msg.angular.z = 0.0;

  this->cmd_vel_pub_->publish(move_msg);
}

void AttachServer::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist();

  this->cmd_vel_pub_->publish(stop_msg);
}

void AttachServer::process_approach_timer_clbk_() {

  // If it has NOT been requested to start the final approach...
  if (!this->start_final_approach_) {
    return;
  }

  if (!this->cart_frame_reached_) {

    if (!is_error_robot_to_cart_frame_computable_()) {
      return;
    }

    /*RCLCPP_INFO(this->get_logger(), "this->now() = %.3f",
                this->now().seconds());

    RCLCPP_INFO(this->get_logger(), "scan stamp = %.3f",
                rclcpp::Time(this->last_scan_->header.stamp).seconds());*/

    move_robot_to_cart_frame_();

    // return to guarantee that the robot will move to cart_frame
    // as long as it does not reach cart_frame
    return;
  }

  // Only if cart_frame has been reached...
  // Check the distance under shelf the robot has to travel
  if (!this->dist_under_shelf_travelled_) {

    if (this->accumulated_dist_ < this->dist_to_move_under_shelf_) {
      move_forward_();

      RCLCPP_INFO(this->get_logger(), "accumulated_dist_ : %f",
                  this->accumulated_dist_);

    } else {
      stop_robot();

      auto elevator_up_msg = std_msgs::msg::String();
      this->elevator_up_pub_->publish(elevator_up_msg);

      RCLCPP_INFO(this->get_logger(), "Robot lifted the shelf successfully !");

      this->dist_under_shelf_travelled_ = true;

      RCLCPP_INFO(this->get_logger(),
                  "Final approach completed successfully !");

      this->start_final_approach_ = false; // Reset start_final_approach_
      this->accumulated_dist_ = 0.0;       // Reset accumulated_dist_

      // Reset first_odom_ since to force odom calculations when the robot is
      this->first_odom_ = true;

      RCLCPP_INFO(this->get_logger(), "Shutting down in 3 seconds...");
      rclcpp::sleep_for(std::chrono::seconds(3));

      rclcpp::shutdown();
    }
  }
}

void AttachServer::approach_service_clbk_(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Approach Service Requested...");

  // If the laser scanner is not working
  if (!this->last_scan_) {
    response->complete = false;
    RCLCPP_WARN(this->get_logger(), "Laser scan is not working.");
    return;
  }

  std::vector<size_t> shelf_leg_detected_indices;

  for (size_t i = 0; i < this->last_scan_->intensities.size(); i++) {

    bool is_shelf_leg_detected = (this->last_scan_->intensities[i] > 6000);

    // If the ray intensity is different from inf, -inf and NAN and ...
    if (std::isfinite(this->last_scan_->ranges[i]) && is_shelf_leg_detected) {
      shelf_leg_detected_indices.push_back(i);
    }
  }

  // Identify detected entities with the laser scanner
  std::vector<std::vector<size_t>> leg_groups;
  leg_groups = identify_shelf_leg_index_groups_(shelf_leg_detected_indices);

  RCLCPP_INFO(this->get_logger(), "Leg groups = %zu", leg_groups.size());

  // Compute the central point between both shelf legs (if possible)
  this->legs_center_computable_ = is_legs_center_computable_(leg_groups);
  if (this->legs_center_computable_) {

    compute_legs_center_(leg_groups);

    prepare_cart_frame_tf_();

    // cart_frame_tf_ publication starts here...

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
    RCLCPP_WARN(this->get_logger(), "The central point is not computable.");
    return;
  }
}

bool AttachServer::is_final_approach_completed() {
  return this->dist_under_shelf_travelled_;
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)