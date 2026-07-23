#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 200.0);
    mass_ = declare_parameter<double>("mass", 1.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);
    arm_length_ = declare_parameter<double>("arm_length", 0.22);
    thrust_coefficient_ =
      declare_parameter<double>("thrust_coefficient", 8.54858e-6);
    moment_coefficient_ =
      declare_parameter<double>("moment_coefficient", 1.37e-7);
    motor_time_constant_ =
      declare_parameter<double>("motor_time_constant", 0.05);
    max_rpm_ = declare_parameter<double>("max_rpm", 10000.0);

    inertia_.x() = declare_parameter<double>("inertia_xx", 0.005);
    inertia_.y() = declare_parameter<double>("inertia_yy", 0.005);
    inertia_.z() = declare_parameter<double>("inertia_zz", 0.009);

    linear_drag_.x() = declare_parameter<double>("linear_drag_x", 0.10);
    linear_drag_.y() = declare_parameter<double>("linear_drag_y", 0.10);
    linear_drag_.z() = declare_parameter<double>("linear_drag_z", 0.15);

    angular_drag_.x() = declare_parameter<double>("angular_drag_x", 0.002);
    angular_drag_.y() = declare_parameter<double>("angular_drag_y", 0.002);
    angular_drag_.z() = declare_parameter<double>("angular_drag_z", 0.003);

    if (update_rate_hz_ <= 0.0 || mass_ <= 0.0 ||
        thrust_coefficient_ <= 0.0 || motor_time_constant_ <= 0.0 ||
        inertia_.minCoeff() <= 0.0)
    {
      throw std::runtime_error("Invalid non-positive dynamics parameter");
    }

    dt_ = 1.0 / update_rate_hz_;

    motor_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/drone/motor_rpm_cmd", 10,
      std::bind(&QuadrotorDynamicsNode::motorCallback, this,
        std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/drone/imu", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
    actual_rpm_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>("/drone/motor_rpm", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_.header.frame_id = "map";

    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_),
      std::bind(&QuadrotorDynamicsNode::update, this));

    RCLCPP_INFO(get_logger(), "6DoF dynamics started at %.1f Hz", update_rate_hz_);
    RCLCPP_INFO(get_logger(),
      "Motor order: M1 front-left, M2 front-right, "
      "M3 rear-right, M4 rear-left");
  }

private:
  void motorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(get_logger(), "Expected 4 RPM values, received %zu",
        msg->data.size());
      return;
    }

    for (std::size_t i = 0; i < 4; ++i) {
      const double value = std::isfinite(msg->data[i]) ?
        static_cast<double>(msg->data[i]) : 0.0;
      commanded_rpm_[i] = std::clamp(value, 0.0, max_rpm_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "RPM cmd: [%.1f, %.1f, %.1f, %.1f]",
      commanded_rpm_[0], commanded_rpm_[1],
      commanded_rpm_[2], commanded_rpm_[3]);
  }

  void update()
  {
    updateMotors();
    integrateDynamics();
    ++step_count_;

    const std::size_t state_divider = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::round(update_rate_hz_ / 50.0)));
    const std::size_t path_divider = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::round(update_rate_hz_ / 10.0)));
    const auto stamp = now();

    if (step_count_ % state_divider == 0) {
      publishState(stamp);
    }
    if (step_count_ % path_divider == 0) {
      publishPath(stamp);
    }
  }

  void updateMotors()
  {
    const double alpha = std::clamp(dt_ / motor_time_constant_, 0.0, 1.0);
    for (std::size_t i = 0; i < 4; ++i) {
      actual_rpm_[i] += alpha * (commanded_rpm_[i] - actual_rpm_[i]);
      actual_rpm_[i] = std::clamp(actual_rpm_[i], 0.0, max_rpm_);
    }
  }

  void integrateDynamics()
  {
    constexpr double pi = 3.14159265358979323846;
    std::array<double, 4> omega_squared{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> rotor_thrust{0.0, 0.0, 0.0, 0.0};

    for (std::size_t i = 0; i < 4; ++i) {
      const double rotor_omega = actual_rpm_[i] * 2.0 * pi / 60.0;
      omega_squared[i] = rotor_omega * rotor_omega;
      rotor_thrust[i] = thrust_coefficient_ * omega_squared[i];
    }

    const double total_thrust = rotor_thrust[0] + rotor_thrust[1] +
      rotor_thrust[2] + rotor_thrust[3];
    const double arm = arm_length_ / std::sqrt(2.0);

    body_torque_.x() = arm * (
      rotor_thrust[0] - rotor_thrust[1] -
      rotor_thrust[2] + rotor_thrust[3]);
    body_torque_.y() = arm * (
      -rotor_thrust[0] - rotor_thrust[1] +
      rotor_thrust[2] + rotor_thrust[3]);
    body_torque_.z() = moment_coefficient_ * (
      -omega_squared[0] + omega_squared[1] -
      omega_squared[2] + omega_squared[3]);

    const Eigen::Matrix3d rotation_body_to_world =
      orientation_.toRotationMatrix();
    const Eigen::Vector3d thrust_world = rotation_body_to_world *
      Eigen::Vector3d(0.0, 0.0, total_thrust);
    const Eigen::Vector3d gravity_force(0.0, 0.0, -mass_ * gravity_);
    const Eigen::Vector3d drag_force = -linear_drag_.cwiseProduct(velocity_);
    acceleration_ = (thrust_world + gravity_force + drag_force) / mass_;

    const Eigen::Vector3d angular_momentum =
      inertia_.cwiseProduct(body_angular_velocity_);
    const Eigen::Vector3d gyroscopic_term =
      body_angular_velocity_.cross(angular_momentum);
    const Eigen::Vector3d angular_drag_torque =
      angular_drag_.cwiseProduct(body_angular_velocity_);
    body_angular_acceleration_ =
      (body_torque_ - gyroscopic_term - angular_drag_torque)
      .cwiseQuotient(inertia_);

    velocity_ += acceleration_ * dt_;
    position_ += velocity_ * dt_;
    body_angular_velocity_ += body_angular_acceleration_ * dt_;

    const Eigen::Quaterniond omega_quaternion(
      0.0,
      body_angular_velocity_.x(),
      body_angular_velocity_.y(),
      body_angular_velocity_.z());
    Eigen::Quaterniond q_dot = orientation_ * omega_quaternion;
    q_dot.coeffs() *= 0.5;
    orientation_.coeffs() += q_dot.coeffs() * dt_;
    orientation_.normalize();

    applyGroundConstraint();
    applyNumericalSafety();
  }

  void applyGroundConstraint()
  {
    if (position_.z() < 0.0) {
      position_.z() = 0.0;
      if (velocity_.z() < 0.0) {
        velocity_.z() = 0.0;
      }
      if (acceleration_.z() < 0.0) {
        acceleration_.z() = 0.0;
      }
    }

    if (position_.z() <= 0.0 && velocity_.z() <= 0.0 &&
        acceleration_.z() <= 0.0)
    {
      position_.z() = 0.0;
      velocity_.z() = 0.0;
      acceleration_.z() = 0.0;
    }
  }

  void applyNumericalSafety()
  {
    const bool finite = position_.allFinite() && velocity_.allFinite() &&
      acceleration_.allFinite() && body_angular_velocity_.allFinite() &&
      body_angular_acceleration_.allFinite() &&
      orientation_.coeffs().allFinite();

    if (!finite) {
      RCLCPP_ERROR(get_logger(),
        "Non-finite state detected. Resetting simulator state.");
      position_.setZero();
      velocity_.setZero();
      acceleration_.setZero();
      body_angular_velocity_.setZero();
      body_angular_acceleration_.setZero();
      body_torque_.setZero();
      orientation_ = Eigen::Quaterniond::Identity();
      actual_rpm_.fill(0.0);
      commanded_rpm_.fill(0.0);
    }
  }

  void publishState(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = position_.x();
    odom.pose.pose.position.y = position_.y();
    odom.pose.pose.position.z = position_.z();
    odom.pose.pose.orientation.w = orientation_.w();
    odom.pose.pose.orientation.x = orientation_.x();
    odom.pose.pose.orientation.y = orientation_.y();
    odom.pose.pose.orientation.z = orientation_.z();
    odom.twist.twist.linear.x = velocity_.x();
    odom.twist.twist.linear.y = velocity_.y();
    odom.twist.twist.linear.z = velocity_.z();
    odom.twist.twist.angular.x = body_angular_velocity_.x();
    odom.twist.twist.angular.y = body_angular_velocity_.y();
    odom.twist.twist.angular.z = body_angular_velocity_.z();
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.orientation.w = orientation_.w();
    imu.orientation.x = orientation_.x();
    imu.orientation.y = orientation_.y();
    imu.orientation.z = orientation_.z();
    imu.angular_velocity.x = body_angular_velocity_.x();
    imu.angular_velocity.y = body_angular_velocity_.y();
    imu.angular_velocity.z = body_angular_velocity_.z();

    const Eigen::Vector3d gravity_world(0.0, 0.0, -gravity_);
    const Eigen::Vector3d specific_force_body = orientation_.inverse() *
      (acceleration_ - gravity_world);
    imu.linear_acceleration.x = specific_force_body.x();
    imu.linear_acceleration.y = specific_force_body.y();
    imu.linear_acceleration.z = specific_force_body.z();
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = position_.x();
    transform.transform.translation.y = position_.y();
    transform.transform.translation.z = position_.z();
    transform.transform.rotation.w = orientation_.w();
    transform.transform.rotation.x = orientation_.x();
    transform.transform.rotation.y = orientation_.y();
    transform.transform.rotation.z = orientation_.z();
    tf_broadcaster_->sendTransform(transform);

    std_msgs::msg::Float32MultiArray rpm_message;
    rpm_message.data.resize(4);
    for (std::size_t i = 0; i < 4; ++i) {
      rpm_message.data[i] = static_cast<float>(actual_rpm_[i]);
    }
    actual_rpm_pub_->publish(rpm_message);
  }

  void publishPath(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = position_.x();
    pose.pose.position.y = position_.y();
    pose.pose.position.z = position_.z();
    pose.pose.orientation.w = orientation_.w();
    pose.pose.orientation.x = orientation_.x();
    pose.pose.orientation.y = orientation_.y();
    pose.pose.orientation.z = orientation_.z();

    path_.header.stamp = stamp;
    path_.poses.push_back(pose);
    if (path_.poses.size() > 5000) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);
  }

  double update_rate_hz_{200.0};
  double dt_{0.005};
  double mass_{1.0};
  double gravity_{9.81};
  double arm_length_{0.22};
  double thrust_coefficient_{8.54858e-6};
  double moment_coefficient_{1.37e-7};
  double motor_time_constant_{0.05};
  double max_rpm_{10000.0};

  Eigen::Vector3d inertia_{0.005, 0.005, 0.009};
  Eigen::Vector3d linear_drag_{0.10, 0.10, 0.15};
  Eigen::Vector3d angular_drag_{0.002, 0.002, 0.003};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d body_angular_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_angular_acceleration_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_torque_{Eigen::Vector3d::Zero()};

  std::array<double, 4> commanded_rpm_{0.0, 0.0, 0.0, 0.0};
  std::array<double, 4> actual_rpm_{0.0, 0.0, 0.0, 0.0};
  std::size_t step_count_{0};

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actual_rpm_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QuadrotorDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
