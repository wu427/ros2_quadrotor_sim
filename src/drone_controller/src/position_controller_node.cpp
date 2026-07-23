#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller_node")
  {
    mass_ = declare_parameter<double>("mass", 1.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    arm_length_ = declare_parameter<double>("arm_length", 0.22);
    kf_ = declare_parameter<double>("thrust_coefficient", 8.54858e-6);
    km_ = declare_parameter<double>("moment_coefficient", 1.37e-7);
    max_rpm_ = declare_parameter<double>("max_rpm", 10000.0);

    kp_ << declare_parameter<double>("position_kp_x", 1.8),
      declare_parameter<double>("position_kp_y", 1.8),
      declare_parameter<double>("position_kp_z", 3.0);
    kd_ << declare_parameter<double>("position_kd_x", 2.2),
      declare_parameter<double>("position_kd_y", 2.2),
      declare_parameter<double>("position_kd_z", 2.5);
    kr_ << declare_parameter<double>("attitude_kp_roll", 0.08),
      declare_parameter<double>("attitude_kp_pitch", 0.08),
      declare_parameter<double>("attitude_kp_yaw", 0.05);
    kw_ << declare_parameter<double>("angular_rate_kd_roll", 0.018),
      declare_parameter<double>("angular_rate_kd_pitch", 0.018),
      declare_parameter<double>("angular_rate_kd_yaw", 0.012);
    inertia_ << declare_parameter<double>("inertia_xx", 0.005),
      declare_parameter<double>("inertia_yy", 0.005),
      declare_parameter<double>("inertia_zz", 0.009);

    max_horizontal_acceleration_ =
      declare_parameter<double>("max_horizontal_acceleration", 3.0);
    max_vertical_acceleration_ =
      declare_parameter<double>("max_vertical_acceleration", 3.0);
    max_tilt_rad_ =
      declare_parameter<double>("max_tilt_angle_deg", 25.0) * kPi / 180.0;
    goal_distance_limit_ =
      declare_parameter<double>("goal_distance_limit", 10.0);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      std::bind(&PositionControllerNode::goalCallback, this,
      std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      std::bind(&PositionControllerNode::odomCallback, this,
      std::placeholders::_1));
    motor_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/drone/motor_rpm_cmd", 10);
    desired_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone/desired_pose", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&PositionControllerNode::update, this));

    RCLCPP_INFO(get_logger(),
      "3D position + geometric attitude controller started");
  }

private:
  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Eigen::Vector3d requested(
      msg->pose.position.x,
      msg->pose.position.y,
      std::max(0.0, msg->pose.position.z));

    if (odom_received_) {
      Eigen::Vector3d displacement = requested - position_;
      if (displacement.norm() > goal_distance_limit_) {
        requested = position_ +
          displacement.normalized() * goal_distance_limit_;
        RCLCPP_WARN(get_logger(), "Goal distance limited to %.1f m",
          goal_distance_limit_);
      }
    }

    target_position_ = requested;

    Eigen::Quaterniond goal_q(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);

    if (goal_q.norm() > 1e-6) {
      goal_q.normalize();
      target_yaw_ = std::atan2(
        2.0 * (goal_q.w() * goal_q.z() +
        goal_q.x() * goal_q.y()),
        1.0 - 2.0 * (goal_q.y() * goal_q.y() +
        goal_q.z() * goal_q.z()));
    } else {
      target_yaw_ = 0.0;
    }

    goal_received_ = true;
    RCLCPP_INFO(get_logger(),
      "Goal [%.2f, %.2f, %.2f], yaw %.1f deg",
      target_position_.x(), target_position_.y(),
      target_position_.z(), target_yaw_ * 180.0 / kPi);
  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_ << msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z;
    velocity_ << msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z;
    body_rate_ << msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z;

    orientation_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    if (orientation_.norm() < 1e-6) {
      orientation_ = Eigen::Quaterniond::Identity();
    } else {
      orientation_.normalize();
    }
    odom_received_ = true;
  }

  void update()
  {
    if (!goal_received_ || !odom_received_) {
      publishRpm({0.0, 0.0, 0.0, 0.0});
      return;
    }

    Eigen::Vector3d position_error = target_position_ - position_;
    Eigen::Vector3d desired_acceleration =
      kp_.cwiseProduct(position_error) -
      kd_.cwiseProduct(velocity_);
    limitAcceleration(desired_acceleration);

    Eigen::Vector3d desired_force =
      mass_ * (desired_acceleration +
      Eigen::Vector3d(0.0, 0.0, gravity_));
    limitTilt(desired_force);

    Eigen::Matrix3d desired_rotation =
      desiredRotation(desired_force, target_yaw_);
    Eigen::Matrix3d current_rotation =
      orientation_.toRotationMatrix();

    Eigen::Matrix3d error_matrix =
      0.5 * (desired_rotation.transpose() * current_rotation -
      current_rotation.transpose() * desired_rotation);
    Eigen::Vector3d attitude_error(
      error_matrix(2, 1), error_matrix(0, 2),
      error_matrix(1, 0));

    Eigen::Vector3d angular_momentum =
      inertia_.cwiseProduct(body_rate_);
    Eigen::Vector3d torque =
      -kr_.cwiseProduct(attitude_error) -
      kw_.cwiseProduct(body_rate_) +
      body_rate_.cross(angular_momentum);

    double thrust = desired_force.dot(current_rotation.col(2));
    thrust = std::clamp(thrust, 0.0, 4.0 * mass_ * gravity_);

    auto rpm = mix(thrust, torque);
    publishRpm(rpm);
    publishDesiredPose(desired_rotation);

    if (++log_counter_ % 100 == 0) {
      RCLCPP_INFO(get_logger(),
        "goal=[%.2f %.2f %.2f] pos=[%.2f %.2f %.2f] "
        "error=%.3f rpm=[%.0f %.0f %.0f %.0f]",
        target_position_.x(), target_position_.y(),
        target_position_.z(), position_.x(), position_.y(),
        position_.z(), position_error.norm(),
        rpm[0], rpm[1], rpm[2], rpm[3]);
    }
  }

  void limitAcceleration(Eigen::Vector3d & a) const
  {
    Eigen::Vector2d horizontal(a.x(), a.y());
    if (horizontal.norm() > max_horizontal_acceleration_) {
      horizontal *= max_horizontal_acceleration_ / horizontal.norm();
      a.x() = horizontal.x();
      a.y() = horizontal.y();
    }
    a.z() = std::clamp(a.z(),
      -max_vertical_acceleration_, max_vertical_acceleration_);
  }

  void limitTilt(Eigen::Vector3d & force) const
  {
    force.z() = std::max(force.z(), 1e-3);
    Eigen::Vector2d horizontal(force.x(), force.y());
    const double limit = force.z() * std::tan(max_tilt_rad_);
    if (horizontal.norm() > limit) {
      horizontal *= limit / horizontal.norm();
      force.x() = horizontal.x();
      force.y() = horizontal.y();
    }
  }

  Eigen::Matrix3d desiredRotation(
    const Eigen::Vector3d & force, double yaw) const
  {
    Eigen::Vector3d b3 = force.normalized();
    Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d b2 = b3.cross(heading);
    if (b2.norm() < 1e-6) {
      heading << -std::sin(yaw), std::cos(yaw), 0.0;
      b2 = b3.cross(heading);
    }
    b2.normalize();
    Eigen::Vector3d b1 = b2.cross(b3).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = b1;
    rotation.col(1) = b2;
    rotation.col(2) = b3;
    return rotation;
  }

  std::array<double, 4> mix(
    double thrust, const Eigen::Vector3d & torque) const
  {
    const double arm = arm_length_ / std::sqrt(2.0);
    const double s = thrust / kf_;
    const double r = torque.x() / (arm * kf_);
    const double p = torque.y() / (arm * kf_);
    const double y = torque.z() / km_;

    std::array<double, 4> omega_squared{
      0.25 * (s + r - p - y),
      0.25 * (s - r - p + y),
      0.25 * (s - r + p - y),
      0.25 * (s + r + p + y)};

    const double max_omega = max_rpm_ * 2.0 * kPi / 60.0;
    const double max_omega_squared = max_omega * max_omega;
    std::array<double, 4> rpm{};

    for (std::size_t i = 0; i < 4; ++i) {
      omega_squared[i] =
        std::clamp(omega_squared[i], 0.0, max_omega_squared);
      rpm[i] = std::sqrt(omega_squared[i]) * 60.0 /
        (2.0 * kPi);
    }
    return rpm;
  }

  void publishRpm(const std::array<double, 4> & rpm)
  {
    std_msgs::msg::Float32MultiArray message;
    message.data.resize(4);
    for (std::size_t i = 0; i < 4; ++i) {
      message.data[i] = static_cast<float>(rpm[i]);
    }
    motor_pub_->publish(message);
  }

  void publishDesiredPose(const Eigen::Matrix3d & rotation)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = now();
    message.header.frame_id = "map";
    message.pose.position.x = target_position_.x();
    message.pose.position.y = target_position_.y();
    message.pose.position.z = target_position_.z();

    Eigen::Quaterniond q(rotation);
    message.pose.orientation.w = q.w();
    message.pose.orientation.x = q.x();
    message.pose.orientation.y = q.y();
    message.pose.orientation.z = q.z();
    desired_pose_pub_->publish(message);
  }

  static constexpr double kPi = 3.14159265358979323846;

  double mass_{1.0}, gravity_{9.81}, arm_length_{0.22};
  double kf_{8.54858e-6}, km_{1.37e-7}, max_rpm_{10000.0};
  double max_horizontal_acceleration_{3.0};
  double max_vertical_acceleration_{3.0};
  double max_tilt_rad_{25.0 * kPi / 180.0};
  double goal_distance_limit_{10.0};

  Eigen::Vector3d kp_{1.8, 1.8, 3.0};
  Eigen::Vector3d kd_{2.2, 2.2, 2.5};
  Eigen::Vector3d kr_{0.08, 0.08, 0.05};
  Eigen::Vector3d kw_{0.018, 0.018, 0.012};
  Eigen::Vector3d inertia_{0.005, 0.005, 0.009};

  Eigen::Vector3d target_position_{0.0, 0.0, 0.0};
  double target_yaw_{0.0};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_rate_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};

  bool goal_received_{false};
  bool odom_received_{false};
  std::size_t log_counter_{0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
