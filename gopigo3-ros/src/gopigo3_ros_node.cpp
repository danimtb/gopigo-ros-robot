#include "gopigo3_ros_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

GoPiGo3RosNode::GoPiGo3RosNode()
: rclcpp::Node("gopigo3_ros"),
  last_cmd_time_(0, 0, RCL_ROS_TIME),
  last_teleop_time_(0, 0, RCL_ROS_TIME),
  last_line_time_(0, 0, RCL_ROS_TIME),
  line_lost_since_(0, 0, RCL_ROS_TIME)
{
  const auto topic = relative_topic(
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel"), "cmd_vel_topic");
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.3);
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 2.0);
  max_motor_dps_ = declare_parameter<double>("max_motor_dps", 500.0);
  cmd_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_timeout", 0.5));

  const auto lf_port_name = declare_parameter<std::string>("line_follower_port", "I2C");
  const auto lf_topic = relative_topic(
    declare_parameter<std::string>("line_follower_topic", "line_follower"), "line_follower_topic");
  line_follow_ = declare_parameter<bool>("line_follow", false);
  line_follow_speed_ = declare_parameter<double>("line_follow_speed", 0.12);
  line_follow_kp_ = declare_parameter<double>("line_follow_kp", 2.6);
  line_follow_kd_ = declare_parameter<double>("line_follow_kd", 0.08);
  line_threshold_ = declare_parameter<double>("line_threshold", 0.6);
  line_search_timeout_ =
    rclcpp::Duration::from_seconds(declare_parameter<double>("line_search_timeout", 1.5));

  const auto color_port_name = declare_parameter<std::string>("color_sensor_port", "I2C");
  const auto color_topic = relative_topic(
    declare_parameter<std::string>("color_sensor_topic", "color"), "color_sensor_topic");
  const bool color_led = declare_parameter<bool>("color_led", true);

  driver_.connect();
  last_cmd_time_ = now();

  RCLCPP_INFO(
    get_logger(), "GoPiGo3 connected: wheel radius %.4f m, wheel separation %.4f m",
    driver_.wheel_radius(), driver_.wheel_separation());

  const auto lf_port = parse_grove_i2c_port(lf_port_name, "line_follower_port");
  if (lf_port != GroveI2cPort::Off) {
    line_follower_ready_ = driver_.init_line_follower(lf_port);
    if (line_follower_ready_) {
      const char * kind =
        driver_.line_follower_kind() == LineFollowerKind::Black ? "black (6 sensors)"
                                                                : "red (5 sensors)";
      RCLCPP_INFO(
        get_logger(), "Line follower on '%s' (%s)", lf_port_name.c_str(), kind);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No line follower on '%s'. Plug it into the I2C Grove or AD1/AD2, enable I2C "
        "(`raspi-config`), and check that `i2cdetect -y 1` lists 0x06.",
        lf_port_name.c_str());
    }
  }

  const auto color_port = parse_grove_i2c_port(color_port_name, "color_sensor_port");
  if (color_port != GroveI2cPort::Off) {
    color_ready_ = driver_.init_color_sensor(color_port, color_led);
    if (color_ready_) {
      RCLCPP_INFO(
        get_logger(), "Color sensor (TCS34725) on '%s', LED %s", color_port_name.c_str(),
        color_led ? "on" : "off");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "No color sensor on '%s'. Plug the Dexter Light & Color Sensor into I2C (or AD1/AD2) "
        "and enable I2C (`raspi-config`).",
        color_port_name.c_str());
    }
  }

  line_raw_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(lf_topic, rclcpp::QoS(10));
  line_position_pub_ =
    create_publisher<std_msgs::msg::Float32>(lf_topic + "/position", rclcpp::QoS(10));
  line_state_pub_ = create_publisher<std_msgs::msg::String>(lf_topic + "/state", rclcpp::QoS(10));
  color_pub_ = create_publisher<std_msgs::msg::ColorRGBA>(color_topic, rclcpp::QoS(10));
  color_name_pub_ = create_publisher<std_msgs::msg::String>(color_topic + "/name", rclcpp::QoS(10));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    topic, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::Twist & msg) { on_cmd_vel(msg); });

  const auto led_ns = relative_topic(
    declare_parameter<std::string>("led_topic_prefix", "led"), "led_topic_prefix");
  eye_left_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
    led_ns + "/eye/left", rclcpp::QoS(10),
    [this](const std_msgs::msg::ColorRGBA & msg) { on_eye_left(msg); });
  eye_right_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
    led_ns + "/eye/right", rclcpp::QoS(10),
    [this](const std_msgs::msg::ColorRGBA & msg) { on_eye_right(msg); });
  eyes_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
    led_ns + "/eyes", rclcpp::QoS(10),
    [this](const std_msgs::msg::ColorRGBA & msg) { on_eyes(msg); });
  blinker_left_sub_ = create_subscription<std_msgs::msg::Float32>(
    led_ns + "/blinker/left", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32 & msg) { on_blinker_left(msg); });
  blinker_right_sub_ = create_subscription<std_msgs::msg::Float32>(
    led_ns + "/blinker/right", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32 & msg) { on_blinker_right(msg); });
  blinkers_sub_ = create_subscription<std_msgs::msg::Float32>(
    led_ns + "/blinkers", rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32 & msg) { on_blinkers(msg); });

  // The line follower drives the steering, so it gets its own fast timer. Each read blocks
  // ~10 ms on I2C, so colour (which nothing steers by) polls far more slowly.
  line_timer_ = create_wall_timer(20ms, [this]() { publish_line_follower(); });
  color_timer_ = create_wall_timer(100ms, [this]() { publish_color(); });
  watchdog_ = create_wall_timer(50ms, [this]() { on_timer(); });

  if (std::string(get_namespace()) == "/") {
    RCLCPP_WARN(
      get_logger(),
      "Node namespace is '/'. With two robots, start each with a unique "
      "`--ros-args -r __ns:=/gopigo_a` (and `/gopigo_b`) so topics do not collide.");
  } else {
    RCLCPP_INFO(get_logger(), "Robot namespace '%s'", get_namespace());
  }

  RCLCPP_INFO(
    get_logger(), "Listening on '%s', stopping the motors after %.2f s without commands",
    cmd_vel_sub_->get_topic_name(), cmd_timeout_.seconds());
  if (line_follower_ready_) {
    RCLCPP_INFO(get_logger(), "Line follower topic '%s'", line_raw_pub_->get_topic_name());
  }
  if (color_ready_) {
    RCLCPP_INFO(get_logger(), "Color topic '%s'", color_pub_->get_topic_name());
  }
  RCLCPP_INFO(
    get_logger(), "Board LEDs: '%s', '%s'", eyes_sub_->get_topic_name(),
    blinkers_sub_->get_topic_name());
  if (line_follow_ && line_follower_ready_) {
    RCLCPP_INFO(
      get_logger(), "Line follow enabled at %.2f m/s (kp=%.2f, kd=%.2f, threshold=%.2f)",
      line_follow_speed_, line_follow_kp_, line_follow_kd_, line_threshold_);
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
  if (!driver_.read_line_follower(reading, line_threshold_)) {
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

void GoPiGo3RosNode::publish_color()
{
  if (!color_ready_) {
    return;
  }

  ColorReading reading;
  if (!driver_.read_color(reading)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Color sensor read failed");
    return;
  }

  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(reading.red);
  color.g = static_cast<float>(reading.green);
  color.b = static_cast<float>(reading.blue);
  color.a = static_cast<float>(reading.clear);
  color_pub_->publish(color);

  std_msgs::msg::String name;
  name.data = reading.name;
  color_name_pub_->publish(name);
}

void GoPiGo3RosNode::follow_line(const LineFollowerReading & reading)
{
  const auto stamp = now();
  geometry_msgs::msg::Twist cmd;

  if (reading.lost == 2) {
    // Nothing under the board. Keep the turn that was being applied when the line went out
    // of view, crawling, so it comes back in: braking here just leaves the robot stranded.
    if (line_lost_since_.nanoseconds() == 0) {
      line_lost_since_ = stamp;
    }
    if (last_line_error_ == 0.0 || (stamp - line_lost_since_) > line_search_timeout_) {
      stop();
      return;
    }
    cmd.linear.x = line_follow_speed_ * 0.3;
    cmd.angular.z = std::copysign(line_follow_kp_ * 0.5, -last_line_error_);
    last_cmd_time_ = stamp;
    drive(cmd);
    return;
  }

  line_lost_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  const double error = reading.position - 0.5;
  const double dt =
    (last_line_time_.nanoseconds() == 0) ? 0.0 : (stamp - last_line_time_).seconds();
  const double derivative = (dt > 0.0) ? (error - last_line_error_) / dt : 0.0;
  last_line_time_ = stamp;
  last_line_error_ = error;

  cmd.linear.x = line_follow_speed_ * (1.0 - std::min(std::abs(error) * 1.6, 0.7));
  cmd.angular.z = -line_follow_kp_ * error - line_follow_kd_ * derivative;
  last_cmd_time_ = stamp;
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

void GoPiGo3RosNode::on_eye_left(const std_msgs::msg::ColorRGBA & msg)
{
  driver_.set_eye_left(msg.r, msg.g, msg.b);
}

void GoPiGo3RosNode::on_eye_right(const std_msgs::msg::ColorRGBA & msg)
{
  driver_.set_eye_right(msg.r, msg.g, msg.b);
}

void GoPiGo3RosNode::on_eyes(const std_msgs::msg::ColorRGBA & msg)
{
  driver_.set_eyes(msg.r, msg.g, msg.b);
}

void GoPiGo3RosNode::on_blinker_left(const std_msgs::msg::Float32 & msg)
{
  driver_.set_blinker_left(msg.data);
}

void GoPiGo3RosNode::on_blinker_right(const std_msgs::msg::Float32 & msg)
{
  driver_.set_blinker_right(msg.data);
}

void GoPiGo3RosNode::on_blinkers(const std_msgs::msg::Float32 & msg)
{
  driver_.set_blinkers(msg.data);
}

GroveI2cPort GoPiGo3RosNode::parse_grove_i2c_port(const std::string & name, const char * param) const
{
  if (name == "I2C" || name == "i2c") {
    return GroveI2cPort::I2c;
  }
  if (name == "AD1" || name == "ad1") {
    return GroveI2cPort::Ad1;
  }
  if (name == "AD2" || name == "ad2") {
    return GroveI2cPort::Ad2;
  }
  if (name == "off" || name == "none") {
    return GroveI2cPort::Off;
  }
  RCLCPP_WARN(
    get_logger(), "Unknown %s '%s' (use I2C, AD1, AD2, or off)", param, name.c_str());
  return GroveI2cPort::Off;
}

std::string GoPiGo3RosNode::relative_topic(std::string name, const char * param) const
{
  if (!name.empty() && name.front() == '/') {
    RCLCPP_WARN(
      get_logger(),
      "%s '%s' starts with '/'; dropping it so `--ros-args -r __ns:=/gopigo_a` applies", param,
      name.c_str());
    while (!name.empty() && name.front() == '/') {
      name.erase(0, 1);
    }
  }
  return name;
}
