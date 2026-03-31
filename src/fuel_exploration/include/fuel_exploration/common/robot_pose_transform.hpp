#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"

#include <optional>
#include <string>

#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>

namespace fuel_exploration {

std::optional<RobotPose> transform_robot_pose(
    const std::optional<RobotPose> &robot_pose,
    const std::string &target_frame,
    tf2_ros::Buffer &tf_buffer,
    double timeout_s,
    const rclcpp::Logger &logger,
    bool &warned_transform_failure);

}  // namespace fuel_exploration
