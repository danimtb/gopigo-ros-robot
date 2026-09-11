#include "gopigo3_ros_node.hpp"

#include "convoy_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace
{
rclcpp::PublisherOptions convoy_pub_options()
{
  rclcpp::PublisherOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return options;
}

rclcpp::SubscriptionOptions convoy_sub_options()
{
  rclcpp::SubscriptionOptions options;
  options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  return options;
}

bool is_convoy_param(const std::string & name)
{
  return name == "robot_id" || name == "convoy_enable" || name == "convoy_role" ||
         name == "cruise_speed" || name == "turbo_speed" || name == "sync_pause" ||
         name == "leader_timeout" || name == "color_min_saturation" ||
         name == "color_min_clear" || name == "color_debounce" || name == "color_cooldown";
}
}  // namespace

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
  max_angular_speed_ = declare_parameter<double>("max_angular_speed", 4.0);
  max_motor_dps_ = declare_parameter<double>("max_motor_dps", 700.0);
  cmd_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_timeout", 0.5));

  const auto lf_port_name = declare_parameter<std::string>("line_follower_port", "I2C");
  const auto lf_topic = relative_topic(
    declare_parameter<std::string>("line_follower_topic", "line_follower"), "line_follower_topic");
  line_follow_ = declare_parameter<bool>("line_follow", false);
  line_follow_speed_ = declare_parameter<double>("line_follow_speed", 0.10);
  line_follow_kp_ = declare_parameter<double>("line_follow_kp", 8.0);
  line_follow_kd_ = declare_parameter<double>("line_follow_kd", 0.08);
  line_follow_slowdown_ = declare_parameter<double>("line_follow_slowdown", 0.35);
  line_threshold_ = declare_parameter<double>("line_threshold", 0.10);
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

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const auto peer_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  // Intra-process + topic statistics instantiate TypedIntraProcessBuffer / MetricsMessage
  // templates that aarch64 gcc 13 rejects. The convoy bus is inter-robot anyway.
  convoy_command_pub_ = create_publisher<std_msgs::msg::String>(
    "/convoy/command", command_qos, convoy_pub_options());
  convoy_peer_pub_ =
    create_publisher<std_msgs::msg::String>("/convoy/peer", peer_qos, convoy_pub_options());
  convoy_ = std::make_unique<ConvoyController>(*this, convoy_command_pub_, convoy_peer_pub_);
  convoy_command_sub_ = create_subscription<std_msgs::msg::String>(
    "/convoy/command", command_qos,
    [this](const std_msgs::msg::String & msg) { convoy_->on_command(msg); },
    convoy_sub_options());
  convoy_peer_sub_ = create_subscription<std_msgs::msg::String>(
    "/convoy/peer", peer_qos,
    [this](const std_msgs::msg::String & msg) { convoy_->on_peer(msg); },
    convoy_sub_options());
  const auto run_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  convoy_run_sub_ = create_subscription<std_msgs::msg::String>(
    "/convoy/run", run_qos,
    [this](const std_msgs::msg::String & msg) { convoy_->on_run(msg); },
    convoy_sub_options());

  // After declare_parameter in ConvoyController: that call would otherwise hit this
  // callback and reject robot_id as a "runtime" change.
  param_callback_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return on_set_parameters(params);
    });

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
  if (convoy_) {
    convoy_->tick();
  }
  if (!stopped_ && !convoy_hold_ && (now() - last_cmd_time_) > cmd_timeout_) {
    stop();
  }
}

void GoPiGo3RosNode::publish_line_follower()
{
  if (!line_follower_ready_) {
    return;
  }

  // Each read blocks ~10 ms on I2C. Skip while teleop is live so cmd_vel is not
  // queued behind that settle delay on the single-threaded executor.
  if (teleop_recent()) {
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

  if (line_follow_ && !convoy_hold_ && (now() - last_teleop_time_) > cmd_timeout_) {
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

  if (convoy_) {
    convoy_->on_color(reading);
  }
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
    cmd.angular.z =
      std::copysign(line_follow_kp_ * 0.5, -last_line_error_) + steer_bias_;
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

  // Only a mild slowdown in the curve: cutting the forward speed hard makes the outer wheel
  // no faster than when driving straight, so the robot ends up pivoting on a braked inner
  // wheel instead of driving around the curve.
  const double turn = std::min(std::abs(error) / 0.5, 1.0);
  cmd.linear.x = line_follow_speed_ * (1.0 - line_follow_slowdown_ * turn);
  cmd.angular.z = -line_follow_kp_ * error - line_follow_kd_ * derivative + steer_bias_;

  if (std::abs(cmd.angular.z) > max_angular_speed_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Line follow asks for %.2f rad/s, capped by max_angular_speed %.2f", cmd.angular.z,
      max_angular_speed_);
  }

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

bool GoPiGo3RosNode::teleop_recent() const
{
  if (last_teleop_time_.nanoseconds() == 0) {
    return false;
  }
  return (now() - last_teleop_time_) <= cmd_timeout_;
}

void GoPiGo3RosNode::set_follow_speed(double speed)
{
  line_follow_speed_ = std::max(0.0, speed);
}

void GoPiGo3RosNode::set_steer_bias(double bias_rad)
{
  steer_bias_ = bias_rad;
}

void GoPiGo3RosNode::set_convoy_hold(bool hold)
{
  convoy_hold_ = hold;
}

void GoPiGo3RosNode::set_eyes(double red, double green, double blue)
{
  driver_.set_eyes(red, green, blue);
}

void GoPiGo3RosNode::set_eye_left(double red, double green, double blue)
{
  driver_.set_eye_left(red, green, blue);
}

void GoPiGo3RosNode::set_eye_right(double red, double green, double blue)
{
  driver_.set_eye_right(red, green, blue);
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

rcl_interfaces::msg::SetParametersResult GoPiGo3RosNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Reject the whole set before touching anything, so a bad value never lands half applied.
  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (name == "line_follow") {
      if (param.as_bool() && !line_follower_ready_) {
        result.successful = false;
        result.reason = "no line follower answered at start-up";
      }
    } else if (name == "line_follow_slowdown") {
      if (param.as_double() < 0.0 || param.as_double() > 1.0) {
        result.successful = false;
        result.reason = "line_follow_slowdown is a fraction of the speed, keep it within 0 and 1";
      }
    } else if (name == "line_threshold") {
      if (param.as_double() <= 0.0 || param.as_double() > 1.0) {
        result.successful = false;
        result.reason = "line_threshold is a sensor reading, keep it within 0 and 1";
      }
    } else if (
      name == "line_follow_speed" || name == "line_follow_kp" || name == "line_follow_kd" ||
      name == "max_linear_speed" || name == "max_angular_speed" || name == "max_motor_dps" ||
      name == "cmd_timeout" || name == "line_search_timeout") {
      if (param.as_double() < 0.0) {
        result.successful = false;
        result.reason = name + " cannot be negative";
      }
    } else if (name == "robot_id") {
      result.successful = false;
      result.reason = "robot_id only takes effect when the node starts";
    } else if (name == "convoy_role") {
      const auto & role = param.as_string();
      if (role != "leader" && role != "follower") {
        result.successful = false;
        result.reason = "convoy_role must be leader or follower";
      }
    } else if (name == "color_debounce") {
      if (param.as_int() < 1) {
        result.successful = false;
        result.reason = "color_debounce must be >= 1";
      }
    } else if (
      name == "cruise_speed" || name == "turbo_speed" || name == "sync_pause" ||
      name == "leader_timeout" || name == "color_min_saturation" || name == "color_min_clear" ||
      name == "color_cooldown") {
      if (param.as_double() < 0.0) {
        result.successful = false;
        result.reason = name + " cannot be negative";
      }
    } else if (
      name == "cmd_vel_topic" || name == "line_follower_topic" || name == "color_sensor_topic" ||
      name == "led_topic_prefix" || name == "line_follower_port" ||
      name == "color_sensor_port" || name == "color_led") {
      // Topics, ports and the colour LED are wired up once, so accepting a new value here
      // would leave the parameter and the running node disagreeing.
      result.successful = false;
      result.reason = name + " only takes effect when the node starts";
    }

    if (!result.successful) {
      return result;
    }
  }

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (convoy_ && is_convoy_param(name)) {
      const auto convoy_result = convoy_->apply_parameter(param);
      if (!convoy_result.successful) {
        return convoy_result;
      }
      continue;
    }
    if (name == "line_follow") {
      line_follow_ = param.as_bool();
      last_line_error_ = 0.0;
      last_line_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      line_lost_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      if (!line_follow_) {
        stop();  // hand the wheels back to teleop standing still, not on the last command
      }
      RCLCPP_INFO(
        get_logger(), "Line follow %s", line_follow_ ? "on" : "off, waiting for teleop");
    } else if (name == "line_follow_speed") {
      line_follow_speed_ = param.as_double();
    } else if (name == "line_follow_kp") {
      line_follow_kp_ = param.as_double();
    } else if (name == "line_follow_kd") {
      line_follow_kd_ = param.as_double();
    } else if (name == "line_follow_slowdown") {
      line_follow_slowdown_ = param.as_double();
    } else if (name == "line_threshold") {
      line_threshold_ = param.as_double();
    } else if (name == "max_linear_speed") {
      max_linear_speed_ = param.as_double();
    } else if (name == "max_angular_speed") {
      max_angular_speed_ = param.as_double();
    } else if (name == "max_motor_dps") {
      max_motor_dps_ = param.as_double();
    } else if (name == "cmd_timeout") {
      cmd_timeout_ = rclcpp::Duration::from_seconds(param.as_double());
    } else if (name == "line_search_timeout") {
      line_search_timeout_ = rclcpp::Duration::from_seconds(param.as_double());
    }
  }

  return result;
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
