#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class QuadrotorDynamicsNode : public rclcpp::Node
{
public:
  QuadrotorDynamicsNode()
  : Node("quadrotor_dynamics_node")
  {
    mass_ = declare_parameter<double>(
      "mass", 1.0);

    gravity_ = declare_parameter<double>(
      "gravity", 9.81);

    thrust_coefficient_ = declare_parameter<double>(
      "thrust_coefficient", 8.54858e-6);

    motor_time_constant_ = declare_parameter<double>(
      "motor_time_constant", 0.05);

    vertical_drag_ = declare_parameter<double>(
      "vertical_drag", 0.15);

    max_rpm_ = declare_parameter<double>(
      "max_rpm", 10000.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/drone/odom", 10);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/drone/imu", 10);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/drone/path", 10);

    motor_sub_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      "/drone/motor_rpm_cmd", 10,
      std::bind(
        &QuadrotorDynamicsNode::motorCallback,
        this,
        std::placeholders::_1));

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = "map";

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&QuadrotorDynamicsNode::update, this));

    RCLCPP_INFO(get_logger(), "Dynamics node started at 50 Hz");
  }

private:
  void motorCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN(
        get_logger(),
        "Expected 4 RPM values, received %zu",
        msg->data.size());
      return;
    }

    for (std::size_t i = 0; i < 4; ++i) {
      rpm_[i] = std::clamp(
        static_cast<double>(msg->data[i]),
        0.0,
        max_rpm_);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "RPM cmd: [%.1f, %.1f, %.1f, %.1f]",
      rpm_[0], rpm_[1], rpm_[2], rpm_[3]);
  }

  void update()
  {
    const auto stamp = now();

    constexpr double dt = 0.02;
    constexpr double pi = 3.14159265358979323846;

    const double motor_alpha = std::clamp(
      dt / motor_time_constant_, 0.0, 1.0);

    double total_thrust = 0.0;

    for (std::size_t i = 0; i < 4; ++i) {
      actual_rpm_[i] += motor_alpha *
        (rpm_[i] - actual_rpm_[i]);

      actual_rpm_[i] = std::clamp(
        actual_rpm_[i], 0.0, max_rpm_);

      const double omega =
        actual_rpm_[i] * 2.0 * pi / 60.0;

      total_thrust +=
        thrust_coefficient_ * omega * omega;
    }

    acceleration_z_ =
      total_thrust / mass_
      - gravity_
      - vertical_drag_ * velocity_z_ / mass_;

    if (
      position_z_ <= 0.0 &&
      velocity_z_ <= 0.0 &&
      acceleration_z_ <= 0.0)
    {
      position_z_ = 0.0;
      velocity_z_ = 0.0;
      acceleration_z_ = 0.0;
    } else {
      velocity_z_ += acceleration_z_ * dt;
      position_z_ += velocity_z_ * dt;

      if (position_z_ < 0.0) {
        position_z_ = 0.0;
        velocity_z_ = 0.0;
        acceleration_z_ = 0.0;
      }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = position_z_;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.z = velocity_z_;
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.orientation.w = 1.0;
    imu.linear_acceleration.z = acceleration_z_;
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.z = position_z_;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);

    ++counter_;

    if (counter_ % 5 == 0) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "map";
      pose.pose.position.z = position_z_;
      pose.pose.orientation.w = 1.0;

      path_.header.stamp = stamp;
      path_.poses.push_back(pose);

      if (path_.poses.size() > 2000) {
        path_.poses.erase(path_.poses.begin());
      }

      path_pub_->publish(path_);
    }
  }

  double mass_{1.0};
  double gravity_{9.81};
  double thrust_coefficient_{8.54858e-6};
  double motor_time_constant_{0.05};
  double vertical_drag_{0.15};
  double max_rpm_{10000.0};

  double position_z_{0.0};
  double velocity_z_{0.0};
  double acceleration_z_{0.0};

  std::array<double, 4> rpm_{
    0.0, 0.0, 0.0, 0.0};

  std::array<double, 4> actual_rpm_{
    0.0, 0.0, 0.0, 0.0};

  std::size_t counter_{0};

  rclcpp::Subscription<
    std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;

  rclcpp::Publisher<
    nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Publisher<
    sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  rclcpp::Publisher<
    nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::unique_ptr<
    tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
