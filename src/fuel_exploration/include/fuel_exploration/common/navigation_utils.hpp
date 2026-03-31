#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

#include <string>

namespace fuel_exploration {

struct RobotPose {
  double x;
  double y;
  double yaw;
  std::string frame_id;
};

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &quaternion);

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);

double normalize_angle(double angle);

double distance_xy(double x0, double y0, double x1, double y1);

double clamp_abs(double value, double limit);

}  // namespace fuel_exploration
