#include "fuel_exploration/common/robot_pose_transform.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fuel_exploration {

std::optional<RobotPose> transform_robot_pose(
    const std::optional<RobotPose> &robot_pose, const std::string &target_frame,
    tf2_ros::Buffer &tf_buffer, const double timeout_s,
    const rclcpp::Logger &logger, bool &warned_transform_failure) {
  if (!robot_pose.has_value()) {
    return std::nullopt;
  }

  if (robot_pose->frame_id == target_frame) {
    return robot_pose;
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header.frame_id = robot_pose->frame_id;
  source_pose.header.stamp = builtin_interfaces::msg::Time{};
  source_pose.pose.position.x = robot_pose->x;
  source_pose.pose.position.y = robot_pose->y;
  source_pose.pose.position.z = 0.0;
  source_pose.pose.orientation = yaw_to_quaternion(robot_pose->yaw);

  try {
    const auto transformed_pose = tf_buffer.transform(
        source_pose, target_frame, tf2::durationFromSec(timeout_s));
    warned_transform_failure = false;
    return RobotPose{
        transformed_pose.pose.position.x, transformed_pose.pose.position.y,
        quaternion_to_yaw(transformed_pose.pose.orientation), target_frame};
  } catch (const tf2::TransformException &exception) {
    if (!warned_transform_failure) {
      RCLCPP_WARN(
          logger, "Failed to transform robot pose from '%s' to '%s': %s",
          robot_pose->frame_id.c_str(), target_frame.c_str(), exception.what());
      warned_transform_failure = true;
    }
    return std::nullopt;
  }
}

} // namespace fuel_exploration
