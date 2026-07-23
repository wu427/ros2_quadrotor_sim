#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class PositionControllerNode : public rclcpp::Node
{
public:
  PositionControllerNode()
  : Node("position_controller_node")
  {
    mass_ = declare_parameter<double>("mass", 1.0);
    gravity_ = declare_parameter<double>("gravity", 9.81);

    thrust_coefficient_ = declare_parameter<double>(
      "thrust_coefficient", 8.54858e-6);

    kp_z_ = declare_parameter<double>("kp_z", 3.0);
    kd_z_ = declare_parameter<double>("kd_z", 2.5);

    max_acceleration_z_ = declare_parameter<double>(
      "max_acceleration_z", 3.0);

    max_rpm_ = declare_parameter<double>(
      "max_rpm", 10000.0);

    motor_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
      "/drone/motor_rpm_cmd", 10);

    goal_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone/goal", 10,
      std::bind(
        &PositionControllerNode::goalCallback,
        this,
        std::placeholders::_1));

    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      "/drone/odom", 10,
      std::bind(
        &PositionControllerNode::odomCallback,
        this,
        std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PositionControllerNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Height controller started: Kp=%.2f, Kd=%.2f",
      kp_z_,
      kd_z_);

    RCLCPP_INFO(
      get_logger(),
      "Waiting for target on /drone/goal");
  }

private:
  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    target_z_ = std::max(
      0.0,
      msg->pose.position.z);

    goal_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "New target height: %.3f m",
      target_z_);
  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_z_ = msg->pose.pose.position.z;
    current_velocity_z_ = msg->twist.twist.linear.z;
    odom_received_ = true;
  }

  void update()
  {
    std_msgs::msg::Float32MultiArray command;
    command.data.resize(4, 0.0F);

    if (!goal_received_ || !odom_received_) {
      motor_pub_->publish(command);
      return;
    }

    const double position_error =
      target_z_ - current_z_;

    double desired_acceleration =
      kp_z_ * position_error
      - kd_z_ * current_velocity_z_;

    desired_acceleration = std::clamp(
      desired_acceleration,
      -max_acceleration_z_,
      max_acceleration_z_);

    double total_thrust =
      mass_ * (gravity_ + desired_acceleration);

    total_thrust = std::max(
      0.0,
      total_thrust);

    constexpr double pi =
      3.14159265358979323846;

    const double omega_squared =
      total_thrust /
      (4.0 * thrust_coefficient_);

    double motor_rpm = 0.0;

    if (omega_squared > 0.0) {
      const double omega =
        std::sqrt(omega_squared);

      motor_rpm =
        omega * 60.0 / (2.0 * pi);
    }

    motor_rpm = std::clamp(
      motor_rpm,
      0.0,
      max_rpm_);

    for (std::size_t i = 0; i < 4; ++i) {
      command.data[i] =
        static_cast<float>(motor_rpm);
    }

    motor_pub_->publish(command);

    ++log_counter_;

    if (log_counter_ % 50 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "target=%.2f z=%.2f vz=%.2f error=%.2f rpm=%.1f",
        target_z_,
        current_z_,
        current_velocity_z_,
        position_error,
        motor_rpm);
    }
  }

  double mass_{1.0};
  double gravity_{9.81};
  double thrust_coefficient_{8.54858e-6};

  double kp_z_{3.0};
  double kd_z_{2.5};
  double max_acceleration_z_{3.0};
  double max_rpm_{10000.0};

  double target_z_{0.0};
  double current_z_{0.0};
  double current_velocity_z_{0.0};

  bool goal_received_{false};
  bool odom_received_{false};

  std::size_t log_counter_{0};

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<PositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}
