#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/common/robot_pose_transform.hpp"
#include "fuel_exploration/controller/exploration_controller_config.hpp"
#include "fuel_exploration/controller/exploration_controller_state.hpp"
#include "fuel_exploration/controller/path_tracking.hpp"

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace {

using fuel_exploration::quaternion_to_yaw;
using fuel_exploration::RobotPose;
using fuel_exploration::transform_robot_pose;
using fuel_exploration::controller::compute_path_tracking_command;
using fuel_exploration::controller::declare_exploration_controller_config;
using fuel_exploration::controller::ExplorationControllerConfig;
using fuel_exploration::controller::ExplorationControllerState;

} // namespace

class ExplorationController : public rclcpp::Node {
public:
  ExplorationController() : Node("exploration_controller") {
    config_ = declare_exploration_controller_config(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 20,
        std::bind(&ExplorationController::odom_callback, this,
                  std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        config_.path_topic, 10,
        std::bind(&ExplorationController::path_callback, this,
                  std::placeholders::_1));
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(config_.cmd_vel_topic, 10);

    const auto control_period =
        std::chrono::duration<double>(1.0 / config_.control_rate_hz);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
        std::bind(&ExplorationController::control_loop, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    state_.robot_pose = RobotPose{msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  quaternion_to_yaw(msg->pose.pose.orientation),
                                  msg->header.frame_id};
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    state_.path = *msg;
    if (state_.path.poses.empty()) {
      state_.active_target_index.reset();
      publish_stop();
      return;
    }

    state_.active_target_index = 0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Received planned path with %zu poses",
                         state_.path.poses.size());
  }

  void publish_stop() {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
  }

  std::optional<RobotPose> robot_pose_in_frame(
      const std::string &target_frame) const {
    return transform_robot_pose(state_.robot_pose, target_frame, *tf_buffer_,
                                config_.transform_timeout_s, this->get_logger(),
                                state_.warned_transform_failure);
  }

  void control_loop() {
    if (!state_.robot_pose.has_value() || state_.path.poses.empty()) {
      publish_stop();
      return;
    }

    const std::string target_frame =
        state_.path.header.frame_id.empty() ? state_.robot_pose->frame_id
                                            : state_.path.header.frame_id;
    const auto control_pose = robot_pose_in_frame(target_frame);
    if (!control_pose.has_value()) {
      if (config_.path_frame_must_match) {
        publish_stop();
        return;
      }
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Unable to transform robot pose into path frame '%s'; stopping until TF is available.",
          target_frame.c_str());
      publish_stop();
      return;
    }

    const auto tracking_result =
        compute_path_tracking_command(state_.path, *control_pose, config_.tracking);
    if (tracking_result.goal_reached) {
      publish_stop();
      state_.active_target_index.reset();
      return;
    }

    state_.active_target_index = tracking_result.active_target_index;
    cmd_vel_pub_->publish(tracking_result.cmd_vel);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ExplorationControllerConfig config_;
  ExplorationControllerState state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplorationController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
