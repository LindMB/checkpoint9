#include "attach_shelf/pre_approach.h"
#include "rclcpp/create_publisher.hpp"
#include "rclcpp/create_subscription.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

PreApproach::PreApproach(const std::string &node_name)
    : Node(node_name), node_name_(node_name) {

  // Declare launch parameters (typed, with defaults)
  this->declare_parameter<double>("obstacle", 0.2);
  this->declare_parameter<int>("degrees", 90);
  this->declare_parameter<bool>("final_approach", true);

  // Read launch parameters once at startup
  this->obstacle_ = this->get_parameter("obstacle").as_double();
  this->degrees_ = this->get_parameter("degrees").as_int();
  this->final_approach_ = this->get_parameter("final_approach").as_bool();

  auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

  this->laser_scan_sub_ =
      this->create_subscription<sensor_msgs::msg::LaserScan>(
          "/scan", qos,
          std::bind(&PreApproach::laser_scan_clbk_, this,
                    std::placeholders::_1));

  this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&PreApproach::odom_callback_, this, std::placeholders::_1));

  this->cmd_vel_unstamped_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
          "/diffbot_base_controller/cmd_vel_unstamped", 10);

  auto timer_period = std::chrono::milliseconds(100);
  this->cmd_vel_unstamped_pub_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&PreApproach::cmd_vel_unstamped_pub_timer_clbk_, this));

  RCLCPP_INFO(this->get_logger(),
              "PreApproach initialised | obstacle=%.2f degrees=%d", obstacle_,
              degrees_);
}

void PreApproach::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {

  // Retrieve robot orientation from msg
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  // Convert quaternion to angle (roll, pitch, yaw)
  // with yaw -> orientation around the z-axis
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // For the first odom msg, just init previous_yam_
  if (first_odom_) {
    this->previous_yaw_ = yaw;
    this->first_odom_ = false;
    return;
  }

  // Calculate the variation angle around the z-axis
  double delta_yaw = yaw - this->previous_yaw_;

  // Normalize the delta_yaw to the range [-pi, pi]
  if (delta_yaw > M_PI) {
    delta_yaw -= 2.0 * M_PI;
  } else if (delta_yaw < -M_PI) {
    delta_yaw += 2.0 * M_PI;
  }

  // If the robot is currently rotating of x degrees after reaching the wall
  if (this->obstacle_detected_ && !this->rotation_completed_) {

    this->accumulated_yaw_ += std::abs(delta_yaw);

    RCLCPP_INFO(this->get_logger(), "accumulated_yaw_ : %.2f",
                this->accumulated_yaw_);
  }

  // Update previous_yaw for the next calculation
  this->previous_yaw_ = yaw;
}

void PreApproach::laser_scan_clbk_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // If no obstacle detected yet
  if (!this->obstacle_detected_) {
    if (is_obstacle_detected_at_x_meters_(msg)) {
      this->obstacle_detected_ = true;
    } else {
      // Do nothing
    }
  } else {
    // Do nothing
  }
}

void PreApproach::move_forward_() {

  auto move_forward = geometry_msgs::msg::Twist();
  move_forward.linear.x = this->linear_x_vel;
  move_forward.angular.z = 0.0;

  cmd_vel_unstamped_pub_->publish(move_forward);
}

void PreApproach::stop_robot() {

  auto stop_msg = geometry_msgs::msg::Twist();

  cmd_vel_unstamped_pub_->publish(stop_msg);
}

bool PreApproach::is_obstacle_detected_at_x_meters_(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  double angle_rad;

  for (size_t i = 0; i < msg->ranges.size(); i++) {

    angle_rad = msg->angle_min + (i * msg->angle_increment);

    // Front section -> Between -30deg and 30deg
    bool is_angle_in_front_section =
        (angle_rad >= -(M_PI / 6) && angle_rad <= M_PI / 6);

    // If the ray distance is different from inf, -inf and NAN
    // and is in the front section
    if (std::isfinite(msg->ranges[i]) && is_angle_in_front_section) {

      // If obstacle detected
      if (msg->ranges[i] <= this->obstacle_) {
        return true;
      }
    }
  }

  return false;
}

void PreApproach::rotate_of_x_degrees_() {

  // Convert angle in degrees into radians
  double angle_rad = (M_PI * this->degrees_) / 180;

  // Init rotation message to publish
  auto rotate_msg = geometry_msgs::msg::Twist();
  rotate_msg.linear.x = 0.0;

  // Define the rotation direction based on the sign of angle_rad
  if (angle_rad < 0) {
    rotate_msg.angular.z = -this->angular_z_vel;
  } else {
    rotate_msg.angular.z = this->angular_z_vel;
  }

  // If the robot finished rotating of x degrees
  if (this->accumulated_yaw_ >= std::abs(angle_rad)) {

    // Stop the robot
    stop_robot();

    this->rotation_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "Rotation completed !");

    this->pre_approach_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "Pre-Approach completed !");

    // Prepare robot for next rotation
    this->accumulated_yaw_ = 0.0;

  } else { // Continue to rotate
    cmd_vel_unstamped_pub_->publish(rotate_msg);
  }
}

void PreApproach::cmd_vel_unstamped_pub_timer_clbk_() {

  if (!this->pre_approach_completed_) {

    /// If the wall has been detected wall at x meters
    if (this->obstacle_detected_) {

      /// Stop
      stop_robot();

      /// Rotate of x degrees
      rotate_of_x_degrees_();

    } else {
      /// Move forward
      move_forward_();
    }

  } else {
    // Do nothing
  }
}

bool PreApproach::is_pre_approach_completed() const {
  return this->pre_approach_completed_;
}

std::shared_ptr<PreApproach> attach_shelf_node;

void signal_handler(int /*signum*/) { // intentionally unused

  attach_shelf_node->stop_robot();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  attach_shelf_node = std::make_shared<PreApproach>("attach_shelf_node");

  // Register the signal handler for CTRL+C
  std::signal(SIGINT, signal_handler);

  rclcpp::Rate rate(10); // 10Hz = 1/10 sec = 0,1 sec = 100 ms

  while (rclcpp::ok() && !attach_shelf_node->is_pre_approach_completed()) {
    rclcpp::spin_some(attach_shelf_node);
    rate.sleep();
  }

  RCLCPP_INFO(attach_shelf_node->get_logger(), "Shutting down in 3 seconds...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Destroy publishers/subscribers/timers BEFORE shutdown
  attach_shelf_node.reset();

  rclcpp::shutdown();
  return 0;
}
