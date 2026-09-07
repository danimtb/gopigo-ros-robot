#include "gopigo3_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

GoPiGo3RosNode::GoPiGo3RosNode()
: rclcpp::Node("gopigo3_ros"),
  last_cmd_time_(0, 0, RCL_ROS_TIME),
  last_teleop_time_(0, 0, RCL_ROS_TIME)
{
  const auto topic = declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.3);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
  max_motor_dps_ = declare_parameter<double>("max_motor_dps", 500.0);
  cmd_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_timeout", 0.5));

  const auto lf_port_name = declare_parameter<std::string>("line_follower_port", "I2C");
  const auto lf_topic = declare_parameter<std::string>("line_follower_topic", "/line_follower");
  line_follow_ = declare_parameter<bool>("line_follow", false);
  line_follow_speed_ = declare_parameter<double>("line_follow_speed", 0.12);
  line_follow_kp_ = declare_parameter<double>("line_follow_kp", 1.8);

  driver_.connect();
  last_cmd_time_ = now();

  RCLCPP_INFO(
    get_logger(), "GoPiGo3 connected: wheel radius %.4f m, wheel separation %.4f m",
    driver_.wheel_radius(), driver_.wheel_separation());

  const auto lf_port = parse_line_follower_port(lf_port_name);
  if (lf_port != LineFollowerPort::Off) {
    line_follower_ready_ = driver_.init_line_follower(lf_port);
    if (line_follower_ready_) {
      const char * kind =
        driver_.line_follower_kind() == LineFollowerKind::Black ? "black (6 sensors)"
                                                                : "red (5 sensors)";
      RCLCPP_INFO(
        get_logger(), "Line follower on '%s' (%s). Publishing '%s'", lf_port_name.c_str(), kind,
        lf_topic.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No line follower on '%s'. Plug it into the I2C Grove or AD1/AD2 and check I2C "
        "(`raspi-config`).",
        lf_port_name.c_str());
    }
  }

  line_raw_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(lf_topic, rclcpp::QoS(10));
  line_position_pub_ =
    create_publisher<std_msgs::msg::Float32>(lf_topic + "/position", rclcpp::QoS(10));
  line_state_pub_ = create_publisher<std_msgs::msg::String>(lf_topic + "/state", rclcpp::QoS(10));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist & msg) { on_cmd_vel(msg); });

  watchdog_ = create_wall_timer(50ms, [this]() { on_timer(); });

  RCLCPP_INFO(
    get_logger(), "Listening on '%s', stopping the motors after %.2f s without commands",
    topic.c_str(), cmd_timeout_.seconds());
  if (line_follow_ && line_follower_ready_) {
    RCLCPP_INFO(
      get_logger(), "Line follow enabled at %.2f m/s (kp=%.2f)", line_follow_speed_,
      line_follow_kp_);
  }
}

GoPiGo3RosNode::~GoPiGo3RosNode()
{
  stop();
}

void GoPiGo3RosNode::on_cmd_vel(const geometry_msgs::msg::Twist & msg)
{
  last_cmd_time_ = now();
  last_teleop_time_ = last_cmd_time_;
  drive(msg);
}

void GoPiGo3RosNode::on_timer()
{
  publish_line_follower();

  if (!stopped_ && (now() - last_cmd_time_) > cmd_timeout_) {
    stop();
  }
}

void GoPiGo3RosNode::publish_line_follower()
{
  if (!line_follower_ready_) {
    return;
  }

  LineFollowerReading reading;
  if (!driver_.read_line_follower(reading)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Line follower read failed");
    return;
  }

  std_msgs::msg::Float32MultiArray raw;
  raw.data.resize(reading.count);
  for (std::size_t i = 0; i < reading.count; ++i) {
    raw.data[i] = static_cast<float>(reading.sensors[i]);
  }
  line_raw_pub_->publish(raw);

  std_msgs::msg::Float32 position;
  position.data = static_cast<float>(reading.position);
  line_position_pub_->publish(position);

  std_msgs::msg::String state;
  state.data = reading.state;
  line_state_pub_->publish(state);

  if (line_follow_ && (now() - last_teleop_time_) > cmd_timeout_) {
    follow_line(reading);
  }
}

void GoPiGo3RosNode::follow_line(const LineFollowerReading & reading)
{
  geometry_msgs::msg::Twist cmd;
  if (reading.lost == 2) {
    stop();
    return;
  }

  const double error = reading.position - 0.5;
  cmd.linear.x = line_follow_speed_ * (1.0 - std::min(std::abs(error) * 1.6, 0.7));
  cmd.angular.z = -line_follow_kp_ * error;
  last_cmd_time_ = now();
  drive(cmd);
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

LineFollowerPort GoPiGo3RosNode::parse_line_follower_port(const std::string & name) const
{
  if (name == "I2C" || name == "i2c") {
    return LineFollowerPort::I2c;
  }
  if (name == "AD1" || name == "ad1") {
    return LineFollowerPort::Ad1;
  }
  if (name == "AD2" || name == "ad2") {
    return LineFollowerPort::Ad2;
  }
  if (name == "off" || name == "none") {
    return LineFollowerPort::Off;
  }
  RCLCPP_WARN(
    get_logger(), "Unknown line_follower_port '%s' (use I2C, AD1, AD2, or off)", name.c_str());
  return LineFollowerPort::Off;
}
