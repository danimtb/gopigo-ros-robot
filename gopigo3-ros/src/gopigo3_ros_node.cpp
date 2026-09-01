#include "gopigo3_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

GoPiGo3RosNode::GoPiGo3RosNode()
: rclcpp::Node("gopigo3_ros"),
  last_cmd_time_(0, 0, RCL_ROS_TIME)
{
  const auto topic = declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.3);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
  max_motor_dps_ = declare_parameter<double>("max_motor_dps", 500.0);
  cmd_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_timeout", 0.5));

  driver_.connect();
  last_cmd_time_ = now();

  RCLCPP_INFO(
    get_logger(), "GoPiGo3 connected: wheel radius %.4f m, wheel separation %.4f m",
    driver_.wheel_radius(), driver_.wheel_separation());

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist & msg) { on_cmd_vel(msg); });

  watchdog_ = create_wall_timer(50ms, [this]() { on_timer(); });

  RCLCPP_INFO(
    get_logger(), "Listening on '%s', stopping the motors after %.2f s without commands",
    topic.c_str(), cmd_timeout_.seconds());
}

GoPiGo3RosNode::~GoPiGo3RosNode()
{
  stop();
}

void GoPiGo3RosNode::on_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  last_cmd_time_ = now();
  drive(msg);
}

void GoPiGo3RosNode::on_timer()
{
  // turtle_teleop_key publishes a single Twist per keystroke, so the robot would keep
  // rolling forever after the last key press without this watchdog.
  if (!stopped_ && (now() - last_cmd_time_) > cmd_timeout_) {
    stop();
  }
}

void GoPiGo3RosNode::drive(const geometry_msgs::msg::Twist & msg)
{
  const double linear = std::clamp(msg.linear.x, -max_linear_speed_, max_linear_speed_);
  const double angular = std::clamp(msg.angular.z, -max_angular_speed_, max_angular_speed_);

  const double half_base = driver_.wheel_separation() / 2.0;
  const double radius = driver_.wheel_radius();

  // Differential drive: wheel linear speed (m/s) -> wheel shaft speed (deg/s).
  const double to_dps = 180.0 / (M_PI * radius);
  double left_dps = (linear - angular * half_base) * to_dps;
  double right_dps = (linear + angular * half_base) * to_dps;

  // Scale both wheels together when saturating, so the turn ratio is preserved.
  const double peak = std::max(std::abs(left_dps), std::abs(right_dps));
  if (peak > max_motor_dps_) {
    const double factor = max_motor_dps_ / peak;
    left_dps *= factor;
    right_dps *= factor;
  }

  RCLCPP_DEBUG(
    get_logger(), "cmd_vel v=%.3f m/s w=%.3f rad/s -> left %.0f dps, right %.0f dps",
    linear, angular, left_dps, right_dps);

  driver_.set_wheel_speeds(left_dps, right_dps);
  stopped_ = (left_dps == 0.0 && right_dps == 0.0);
}

void GoPiGo3RosNode::stop()
{
  driver_.stop();
  stopped_ = true;
}
