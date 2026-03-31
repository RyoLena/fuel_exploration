#include "fuel_exploration/common/navigation_utils.hpp"

#include <algorithm>
#include <cmath>

namespace fuel_exploration {

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &quaternion) {
  const double siny_cosp =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw / 2.0);
  quaternion.w = std::cos(yaw / 2.0);
  return quaternion;
}

double normalize_angle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;

  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double distance_xy(const double x0, const double y0, const double x1,
                   const double y1) {
  return std::hypot(x1 - x0, y1 - y0);
}

double clamp_abs(const double value, const double limit) {
  return std::clamp(value, -limit, limit);
}

}  // namespace fuel_exploration
