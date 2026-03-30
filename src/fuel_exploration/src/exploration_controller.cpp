#include <algorithm>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

namespace {

struct RobotPose {
  double x;
  double y;
  double yaw;
  std::string frame_id;
};

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
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
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

} // namespace

class ExplorationController : public rclcpp::Node {
public:
  ExplorationController() : Node("exploration_controller") {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    path_topic_ =
        this->declare_parameter<std::string>("path_topic", "/next_exploration_path");
    cmd_vel_topic_ =
        this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    control_rate_hz_ = this->declare_parameter<double>("control_rate_hz", 10.0);
    lookahead_distance_ =
        this->declare_parameter<double>("lookahead_distance", 0.35);
    waypoint_tolerance_ =
        this->declare_parameter<double>("waypoint_tolerance", 0.15);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.20);
    slow_down_distance_ =
        this->declare_parameter<double>("slow_down_distance", 0.60);
    max_linear_speed_ =
        this->declare_parameter<double>("max_linear_speed", 0.22);
    max_angular_speed_ =
        this->declare_parameter<double>("max_angular_speed", 1.20);
    linear_kp_ = this->declare_parameter<double>("linear_kp", 0.80);
    angular_kp_ = this->declare_parameter<double>("angular_kp", 1.80);
    rotate_in_place_threshold_ =
        this->declare_parameter<double>("rotate_in_place_threshold", 0.55);
    path_frame_must_match_ =
        this->declare_parameter<bool>("path_frame_must_match", false);
    transform_timeout_s_ =
        this->declare_parameter<double>("transform_timeout_s", 0.1);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 20,
        std::bind(&ExplorationController::odom_callback, this,
                  std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10,
        std::bind(&ExplorationController::path_callback, this,
                  std::placeholders::_1));
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
        std::bind(&ExplorationController::control_loop, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = RobotPose{msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            quaternion_to_yaw(msg->pose.pose.orientation),
                            msg->header.frame_id};
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
    if (path_.poses.empty()) {
      active_target_index_.reset();
      publish_stop();
      return;
    }

    active_target_index_ = 0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Received planned path with %zu poses",
                         path_.poses.size());
  }

  void publish_stop() {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
  }

  std::optional<RobotPose> robot_pose_in_frame(
      const std::string &target_frame) const {
    if (!robot_pose_.has_value()) {
      return std::nullopt;
    }

    if (robot_pose_->frame_id == target_frame) {
      return robot_pose_;
    }

    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header.frame_id = robot_pose_->frame_id;
    source_pose.header.stamp = builtin_interfaces::msg::Time{};
    source_pose.pose.position.x = robot_pose_->x;
    source_pose.pose.position.y = robot_pose_->y;
    source_pose.pose.position.z = 0.0;
    source_pose.pose.orientation = yaw_to_quaternion(robot_pose_->yaw);

    try {
      const auto transformed_pose = tf_buffer_->transform(
          source_pose, target_frame, tf2::durationFromSec(transform_timeout_s_));
      warned_transform_failure_ = false;
      return RobotPose{transformed_pose.pose.position.x,
                       transformed_pose.pose.position.y,
                       quaternion_to_yaw(transformed_pose.pose.orientation),
                       target_frame};
    } catch (const tf2::TransformException &exception) {
      if (!warned_transform_failure_) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to transform robot pose from '%s' to '%s': %s",
                    robot_pose_->frame_id.c_str(), target_frame.c_str(),
                    exception.what());
        warned_transform_failure_ = true;
      }
      return std::nullopt;
    }
  }

  std::size_t find_closest_pose_index() const {
    if (!robot_pose_.has_value() || path_.poses.empty()) {
      return 0;
    }

    std::size_t closest_index = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t idx = 0; idx < path_.poses.size(); ++idx) {
      const auto &pose = path_.poses[idx].pose.position;
      const double distance =
          distance_xy(robot_pose_->x, robot_pose_->y, pose.x, pose.y);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = idx;
      }
    }
    return closest_index;
  }

  std::size_t choose_lookahead_index(const std::size_t closest_index) const {
    if (path_.poses.empty()) {
      return 0;
    }

    double accumulated_distance = 0.0;
    std::size_t target_index = closest_index;
    while (target_index + 1 < path_.poses.size() &&
           accumulated_distance < lookahead_distance_) {
      const auto &current = path_.poses[target_index].pose.position;
      const auto &next = path_.poses[target_index + 1].pose.position;
      accumulated_distance += distance_xy(current.x, current.y, next.x, next.y);
      ++target_index;
    }
    return target_index;
  }

  void control_loop() {
    if (!robot_pose_.has_value() || path_.poses.empty()) {
      publish_stop();
      return;
    }

    const std::string target_frame =
        path_.header.frame_id.empty() ? robot_pose_->frame_id : path_.header.frame_id;
    const auto control_pose = robot_pose_in_frame(target_frame);
    if (!control_pose.has_value()) {
      if (path_frame_must_match_) {
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

    const RobotPose &robot_pose = *control_pose;
    const std::size_t closest_index = [this, &robot_pose]() {
      std::size_t closest_index = 0;
      double closest_distance = std::numeric_limits<double>::infinity();
      for (std::size_t idx = 0; idx < path_.poses.size(); ++idx) {
        const auto &pose = path_.poses[idx].pose.position;
        const double distance =
            distance_xy(robot_pose.x, robot_pose.y, pose.x, pose.y);
        if (distance < closest_distance) {
          closest_distance = distance;
          closest_index = idx;
        }
      }
      return closest_index;
    }();
    const auto &goal_pose = path_.poses.back().pose.position;
    const double goal_distance =
        distance_xy(robot_pose.x, robot_pose.y, goal_pose.x, goal_pose.y);
    if (goal_distance < goal_tolerance_) {
      publish_stop();
      active_target_index_.reset();
      return;
    }

    std::size_t target_index = choose_lookahead_index(closest_index);
    while (target_index + 1 < path_.poses.size()) {
      const auto &target_pose = path_.poses[target_index].pose.position;
      const double target_distance =
          distance_xy(robot_pose.x, robot_pose.y, target_pose.x, target_pose.y);
      if (target_distance > waypoint_tolerance_) {
        break;
      }
      ++target_index;
    }
    active_target_index_ = target_index;

    const auto &target_pose = path_.poses[target_index].pose.position;
    const double target_heading =
        std::atan2(target_pose.y - robot_pose.y, target_pose.x - robot_pose.x);
    const double heading_error =
        normalize_angle(target_heading - robot_pose.yaw);

    double linear_speed =
        std::min(max_linear_speed_, linear_kp_ * goal_distance);
    if (goal_distance < slow_down_distance_) {
      linear_speed *= goal_distance / std::max(0.05, slow_down_distance_);
    }
    linear_speed *= std::max(0.0, std::cos(heading_error));
    if (std::abs(heading_error) > rotate_in_place_threshold_) {
      linear_speed = 0.0;
    }

    const double angular_speed =
        clamp_abs(angular_kp_ * heading_error, max_angular_speed_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = std::max(0.0, linear_speed);
    cmd_vel.angular.z = angular_speed;
    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_topic_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  double control_rate_hz_{10.0};
  double lookahead_distance_{0.35};
  double waypoint_tolerance_{0.15};
  double goal_tolerance_{0.20};
  double slow_down_distance_{0.60};
  double max_linear_speed_{0.22};
  double max_angular_speed_{1.20};
  double linear_kp_{0.80};
  double angular_kp_{1.80};
  double rotate_in_place_threshold_{0.55};
  bool path_frame_must_match_{false};
  double transform_timeout_s_{0.1};

  std::optional<RobotPose> robot_pose_;
  nav_msgs::msg::Path path_;
  std::optional<std::size_t> active_target_index_;
  mutable bool warned_transform_failure_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplorationController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
