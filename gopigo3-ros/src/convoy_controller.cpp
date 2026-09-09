#include "convoy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <vector>

#include "gopigo3_ros_node.hpp"

namespace
{
std::vector<std::string> split_csv(const std::string & text)
{
  std::vector<std::string> parts;
  std::string part;
  std::istringstream in(text);
  while (std::getline(in, part, ',')) {
    parts.push_back(part);
  }
  return parts;
}
}  // namespace

ConvoyController::ConvoyController(GoPiGo3RosNode & robot)
: robot_(robot)
{
  declare_parameters();
  load_parameters();

  auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto peer_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  command_pub_ = robot_.create_publisher<std_msgs::msg::String>("/convoy/command", command_qos);
  peer_pub_ = robot_.create_publisher<std_msgs::msg::String>("/convoy/peer", peer_qos);
  command_sub_ = robot_.create_subscription<std_msgs::msg::String>(
    "/convoy/command", command_qos,
    [this](const std_msgs::msg::String & msg) { on_command(msg); });
  peer_sub_ = robot_.create_subscription<std_msgs::msg::String>(
    "/convoy/peer", peer_qos, [this](const std_msgs::msg::String & msg) { on_peer(msg); });

  reset_path_offset();
  last_leader_seen_ = robot_.now();
  if (enabled_ && role_ == Role::Leader) {
    apply_command(Command::Cruise, false);
    publish_command();
  }

  RCLCPP_INFO(
    robot_.get_logger(), "Convoy %s: id='%s' role=%s", enabled_ ? "enabled" : "disabled",
    robot_id_.c_str(), role_name(role_));
}

void ConvoyController::declare_parameters()
{
  robot_.declare_parameter<std::string>("robot_id", robot_id_);
  robot_.declare_parameter<bool>("convoy_enable", enabled_);
  robot_.declare_parameter<std::string>("convoy_role", "leader");
  robot_.declare_parameter<double>("cruise_speed", cruise_speed_);
  robot_.declare_parameter<double>("turbo_speed", turbo_speed_);
  robot_.declare_parameter<double>("gap_target_m", gap_target_m_);
  robot_.declare_parameter<double>("gap_kp", gap_kp_);
  robot_.declare_parameter<double>("leader_timeout", leader_timeout_s_);
  robot_.declare_parameter<double>("color_min_saturation", color_min_saturation_);
  robot_.declare_parameter<double>("color_min_clear", color_min_clear_);
  robot_.declare_parameter<int>("color_debounce", color_debounce_);
  robot_.declare_parameter<double>("color_cooldown", color_cooldown_s_);
  robot_.declare_parameter<double>("handover_bias_rad", handover_bias_rad_);
  robot_.declare_parameter<double>("handover_fork_s", handover_fork_s_);
  robot_.declare_parameter<double>("handover_siding_s", handover_siding_s_);
  robot_.declare_parameter<double>("handover_pass_m", handover_pass_m_);
  robot_.declare_parameter<double>("handover_pass_timeout", handover_pass_timeout_s_);
  robot_.declare_parameter<double>("handover_rejoin_s", handover_rejoin_s_);
}

void ConvoyController::load_parameters()
{
  robot_id_ = robot_.get_parameter("robot_id").as_string();
  enabled_ = robot_.get_parameter("convoy_enable").as_bool();
  const auto role = robot_.get_parameter("convoy_role").as_string();
  role_ = (role == "follower") ? Role::Follower : Role::Leader;
  cruise_speed_ = robot_.get_parameter("cruise_speed").as_double();
  turbo_speed_ = robot_.get_parameter("turbo_speed").as_double();
  gap_target_m_ = robot_.get_parameter("gap_target_m").as_double();
  gap_kp_ = robot_.get_parameter("gap_kp").as_double();
  leader_timeout_s_ = robot_.get_parameter("leader_timeout").as_double();
  color_min_saturation_ = robot_.get_parameter("color_min_saturation").as_double();
  color_min_clear_ = robot_.get_parameter("color_min_clear").as_double();
  color_debounce_ = robot_.get_parameter("color_debounce").as_int();
  color_cooldown_s_ = robot_.get_parameter("color_cooldown").as_double();
  handover_bias_rad_ = robot_.get_parameter("handover_bias_rad").as_double();
  handover_fork_s_ = robot_.get_parameter("handover_fork_s").as_double();
  handover_siding_s_ = robot_.get_parameter("handover_siding_s").as_double();
  handover_pass_m_ = robot_.get_parameter("handover_pass_m").as_double();
  handover_pass_timeout_s_ = robot_.get_parameter("handover_pass_timeout").as_double();
  handover_rejoin_s_ = robot_.get_parameter("handover_rejoin_s").as_double();
}

rcl_interfaces::msg::SetParametersResult ConvoyController::apply_parameter(
  const rclcpp::Parameter & param)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const auto & name = param.get_name();

  if (name == "robot_id") {
    result.successful = false;
    result.reason = "robot_id only takes effect when the node starts";
    return result;
  }
  if (name == "convoy_role") {
    const auto & role = param.as_string();
    if (role != "leader" && role != "follower") {
      result.successful = false;
      result.reason = "convoy_role must be leader or follower";
      return result;
    }
    if (role == "leader") {
      become_leader(cmd_ == Command::Handover ? Command::Cruise : cmd_);
    } else {
      become_follower();
    }
    return result;
  }
  if (name == "convoy_enable") {
    enabled_ = param.as_bool();
    if (enabled_ && role_ == Role::Leader) {
      publish_command();
    }
    return result;
  }
  if (name == "color_debounce") {
    if (param.as_int() < 1) {
      result.successful = false;
      result.reason = "color_debounce must be >= 1";
      return result;
    }
    color_debounce_ = static_cast<int>(param.as_int());
    return result;
  }

  const double value = param.as_double();
  if (value < 0.0 && name != "handover_bias_rad" && name != "gap_target_m") {
    result.successful = false;
    result.reason = name + " cannot be negative";
    return result;
  }
  if (name == "cruise_speed") {
    cruise_speed_ = value;
  } else if (name == "turbo_speed") {
    turbo_speed_ = value;
  } else if (name == "gap_target_m") {
    gap_target_m_ = value;
  } else if (name == "gap_kp") {
    gap_kp_ = value;
  } else if (name == "leader_timeout") {
    leader_timeout_s_ = value;
  } else if (name == "color_min_saturation") {
    color_min_saturation_ = value;
  } else if (name == "color_min_clear") {
    color_min_clear_ = value;
  } else if (name == "color_cooldown") {
    color_cooldown_s_ = value;
  } else if (name == "handover_bias_rad") {
    handover_bias_rad_ = value;
  } else if (name == "handover_fork_s") {
    handover_fork_s_ = value;
  } else if (name == "handover_siding_s") {
    handover_siding_s_ = value;
  } else if (name == "handover_pass_m") {
    handover_pass_m_ = value;
  } else if (name == "handover_pass_timeout") {
    handover_pass_timeout_s_ = value;
  } else if (name == "handover_rejoin_s") {
    handover_rejoin_s_ = value;
  }
  return result;
}

void ConvoyController::on_color(const ColorReading & reading)
{
  if (!enabled_ || role_ != Role::Leader) {
    return;
  }

  std::string card;
  if (!is_card(reading, card)) {
    debounce_name_.clear();
    debounce_count_ = 0;
    return;
  }

  if (card == debounce_name_) {
    ++debounce_count_;
  } else {
    debounce_name_ = card;
    debounce_count_ = 1;
  }
  if (debounce_count_ < color_debounce_) {
    return;
  }

  const auto stamp = robot_.now();
  if (card == last_card_ && last_card_time_.nanoseconds() != 0 &&
      (stamp - last_card_time_).seconds() < color_cooldown_s_) {
    return;
  }
  last_card_ = card;
  last_card_time_ = stamp;

  if (card == "red") {
    apply_command(Command::Stop, false);
    publish_command();
  } else if (card == "yellow") {
    apply_command(Command::Cruise, false);
    publish_command();
  } else if (card == "green") {
    apply_command(Command::Turbo, false);
    publish_command();
  } else if (card == "blue") {
    if (!peer_.valid) {
      RCLCPP_INFO(robot_.get_logger(), "Blue card ignored: no follower on the network");
      return;
    }
    start_handover();
  }
}

void ConvoyController::tick()
{
  if (!enabled_) {
    return;
  }

  const auto stamp = robot_.now();
  if (peer_.valid && (stamp - peer_.stamp).seconds() > leader_timeout_s_ * 2.0) {
    peer_.valid = false;
  }
  if (peer_.valid && peer_is_leader()) {
    last_leader_seen_ = stamp;
  }

  if (role_ == Role::Follower && (stamp - last_leader_seen_).seconds() > leader_timeout_s_) {
    RCLCPP_WARN(robot_.get_logger(), "No leader heartbeat; this robot is taking the lead");
    become_leader(cmd_ == Command::Handover || cmd_ == Command::Stop ? Command::Cruise : cmd_);
    publish_command();
  }

  resolve_roles();

  if (role_ == Role::LeaderSiding) {
    const double elapsed = (stamp - phase_since_).seconds();
    if (elapsed < handover_fork_s_) {
      robot_.set_steer_bias(handover_bias_rad_);
    } else {
      robot_.set_steer_bias(0.0);
    }
    if (elapsed >= handover_fork_s_ + handover_siding_s_) {
      role_ = Role::Waiting;
      robot_.set_steer_bias(0.0);
      robot_.set_convoy_hold(true);
      robot_.stop();
      rejoin_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_INFO(robot_.get_logger(), "Handover: waiting on the siding");
    }
  } else if (role_ == Role::Passing) {
    const double gone = reported_path() - pass_s0_;
    const double elapsed = (stamp - phase_since_).seconds();
    if (gone >= handover_pass_m_ || elapsed >= handover_pass_timeout_s_) {
      become_leader(Command::Cruise);
      publish_command();
      RCLCPP_INFO(robot_.get_logger(), "Handover: this robot is the new leader");
    }
  } else if (role_ == Role::Waiting) {
    if (peer_.valid && peer_.role == Role::Leader) {
      if (rejoin_since_.nanoseconds() == 0) {
        rejoin_since_ = stamp;
      }
      if ((stamp - rejoin_since_).seconds() >= handover_rejoin_s_) {
        become_follower();
        last_leader_seen_ = stamp;
        RCLCPP_INFO(robot_.get_logger(), "Handover: rejoining as follower");
      }
    } else if ((stamp - last_leader_seen_).seconds() > leader_timeout_s_ + handover_pass_timeout_s_) {
      become_leader(Command::Cruise);
      publish_command();
      RCLCPP_WARN(robot_.get_logger(), "Handover timed out; this robot is taking the lead");
    }
  }

  apply_motion();
  apply_leds();
  publish_peer();
}

void ConvoyController::on_command(const std_msgs::msg::String & msg)
{
  Command cmd = Command::Cruise;
  if (!parse_command(msg.data, cmd)) {
    return;
  }
  if (role_ != Role::Follower && role_ != Role::Passing) {
    return;
  }
  apply_command(cmd, true);
}

void ConvoyController::on_peer(const std_msgs::msg::String & msg)
{
  const auto parts = split_csv(msg.data);
  if (parts.size() < 6) {
    return;
  }
  if (parts[0] == robot_id_) {
    return;
  }
  Role role = Role::Follower;
  Command cmd = Command::Cruise;
  if (!parse_role(parts[1], role) || !parse_command(parts[2], cmd)) {
    return;
  }
  try {
    peer_.id = parts[0];
    peer_.role = role;
    peer_.cmd = cmd;
    peer_.s_m = std::stod(parts[3]);
    peer_.v_mps = std::stod(parts[4]);
    peer_.seq = static_cast<std::uint32_t>(std::stoul(parts[5]));
    peer_.stamp = robot_.now();
    peer_.valid = true;
  } catch (const std::exception &) {
    return;
  }
}

void ConvoyController::apply_command(Command cmd, bool from_network)
{
  cmd_ = cmd;
  if (from_network && cmd == Command::Handover && role_ == Role::Follower) {
    role_ = Role::Passing;
    phase_since_ = robot_.now();
    pass_s0_ = reported_path();
    robot_.set_steer_bias(0.0);
    robot_.set_convoy_hold(false);
    RCLCPP_INFO(robot_.get_logger(), "Handover: passing on the main line");
  }
}

void ConvoyController::apply_motion()
{
  if (role_ == Role::Waiting || cmd_ == Command::Stop) {
    robot_.set_convoy_hold(true);
    robot_.stop();
    last_v_ = 0.0;
    return;
  }

  robot_.set_convoy_hold(false);
  double v = command_speed(cmd_ == Command::Handover ? Command::Cruise : cmd_);
  if (role_ == Role::Follower && peer_.valid && peer_is_leader() && cmd_ != Command::Handover) {
    const double error = (reported_path() - peer_.s_m) - gap_target_m_;
    v -= gap_kp_ * error;
  }
  v = std::max(0.0, v);
  last_v_ = v;
  robot_.set_follow_speed(v);
}

void ConvoyController::apply_leds()
{
  const bool leader_side =
    role_ == Role::Leader || role_ == Role::LeaderSiding || role_ == Role::Waiting;
  robot_.set_blinker_left(leader_side ? 1.0 : 0.0);
  robot_.set_blinker_right(leader_side ? 0.0 : 1.0);

  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  switch (cmd_) {
    case Command::Stop:
      r = 1.0;
      break;
    case Command::Cruise:
      r = 1.0;
      g = 0.8;
      break;
    case Command::Turbo:
      g = 1.0;
      break;
    case Command::Handover:
      b = 1.0;
      break;
  }
  robot_.set_eyes(r, g, b);
}

void ConvoyController::become_leader(Command cmd)
{
  role_ = Role::Leader;
  cmd_ = (cmd == Command::Handover) ? Command::Cruise : cmd;
  robot_.set_steer_bias(0.0);
  robot_.set_convoy_hold(cmd_ == Command::Stop);
  reset_path_offset();
}

void ConvoyController::become_follower()
{
  role_ = Role::Follower;
  robot_.set_steer_bias(0.0);
  reset_path_offset();
  rejoin_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void ConvoyController::start_handover()
{
  cmd_ = Command::Handover;
  role_ = Role::LeaderSiding;
  phase_since_ = robot_.now();
  robot_.set_steer_bias(handover_bias_rad_);
  robot_.set_convoy_hold(false);
  publish_command();
  RCLCPP_INFO(robot_.get_logger(), "Handover: taking the siding");
}

void ConvoyController::reset_path_offset()
{
  s_offset_ = robot_.path_length_m();
}

void ConvoyController::resolve_roles()
{
  if (!peer_.valid || !peer_is_leader()) {
    return;
  }
  const bool self_lead =
    role_ == Role::Leader || role_ == Role::LeaderSiding;
  if (self_lead && robot_id_ > peer_.id) {
    RCLCPP_WARN(
      robot_.get_logger(), "Two leaders on the network; '%s' yields to '%s'", robot_id_.c_str(),
      peer_.id.c_str());
    become_follower();
    last_leader_seen_ = robot_.now();
  }
}

bool ConvoyController::peer_is_leader() const
{
  return peer_.role == Role::Leader || peer_.role == Role::LeaderSiding ||
         peer_.role == Role::Passing;
}

bool ConvoyController::is_card(const ColorReading & reading, std::string & card) const
{
  if (reading.clear < color_min_clear_ || reading.saturation < color_min_saturation_) {
    return false;
  }
  if (reading.name != "red" && reading.name != "yellow" && reading.name != "green" &&
      reading.name != "blue") {
    return false;
  }
  card = reading.name;
  return true;
}

double ConvoyController::command_speed(Command cmd) const
{
  if (cmd == Command::Turbo) {
    return turbo_speed_;
  }
  if (cmd == Command::Stop) {
    return 0.0;
  }
  return cruise_speed_;
}

double ConvoyController::reported_path() const
{
  return robot_.path_length_m() - s_offset_;
}

void ConvoyController::publish_command()
{
  std_msgs::msg::String msg;
  msg.data = command_name(cmd_);
  command_pub_->publish(msg);
}

void ConvoyController::publish_peer()
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(3);
  out << robot_id_ << ',' << role_name(role_) << ',' << command_name(cmd_) << ','
      << reported_path() << ',' << last_v_ << ',' << seq_;
  ++seq_;
  std_msgs::msg::String msg;
  msg.data = out.str();
  peer_pub_->publish(msg);
}

const char * ConvoyController::role_name(Role role)
{
  switch (role) {
    case Role::Leader:
      return "leader";
    case Role::Follower:
      return "follower";
    case Role::LeaderSiding:
      return "leader_siding";
    case Role::Waiting:
      return "waiting";
    case Role::Passing:
      return "passing";
  }
  return "follower";
}

const char * ConvoyController::command_name(Command cmd)
{
  switch (cmd) {
    case Command::Stop:
      return "STOP";
    case Command::Cruise:
      return "CRUISE";
    case Command::Turbo:
      return "TURBO";
    case Command::Handover:
      return "HANDOVER";
  }
  return "CRUISE";
}

bool ConvoyController::parse_role(const std::string & text, Role & role)
{
  if (text == "leader") {
    role = Role::Leader;
  } else if (text == "follower") {
    role = Role::Follower;
  } else if (text == "leader_siding") {
    role = Role::LeaderSiding;
  } else if (text == "waiting") {
    role = Role::Waiting;
  } else if (text == "passing") {
    role = Role::Passing;
  } else {
    return false;
  }
  return true;
}

bool ConvoyController::parse_command(const std::string & text, Command & cmd)
{
  if (text == "STOP") {
    cmd = Command::Stop;
  } else if (text == "CRUISE") {
    cmd = Command::Cruise;
  } else if (text == "TURBO") {
    cmd = Command::Turbo;
  } else if (text == "HANDOVER") {
    cmd = Command::Handover;
  } else {
    return false;
  }
  return true;
}
