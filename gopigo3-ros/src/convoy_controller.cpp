#include "convoy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <utility>
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

constexpr double kEyeBrightness = 0.25;
constexpr double kFlashSeconds = 3.0;
// Half period of the blink that counts the sync pause down, and the solid "go" that
// closes it.
constexpr double kSyncBlinkSeconds = 0.15;
constexpr double kSyncGoSeconds = 0.3;

struct Rgb
{
  double r;
  double g;
  double b;
};

Rgb card_rgb(const std::string & card)
{
  if (card == "red") {
    return {1.0, 0.0, 0.0};
  }
  if (card == "yellow") {
    return {1.0, 0.75, 0.0};
  }
  if (card == "green") {
    return {0.0, 1.0, 0.0};
  }
  if (card == "blue") {
    return {0.0, 0.0, 1.0};
  }
  return {1.0, 1.0, 1.0};
}
}  // namespace

ConvoyController::ConvoyController(
  GoPiGo3RosNode & robot,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr peer_pub)
: robot_(robot),
  command_pub_(std::move(command_pub)),
  peer_pub_(std::move(peer_pub))
{
  declare_parameters();
  load_parameters();

  last_leader_seen_ = robot_.now();
  if (enabled_) {
    robot_.set_convoy_hold(true);
    robot_.stop();
  }

  RCLCPP_INFO(
    robot_.get_logger(),
    "Convoy %s: id='%s' starting as follower; elects leader if none appears. "
    "Waiting for /convoy/run START",
    enabled_ ? "enabled" : "disabled", robot_id_.c_str());
}

void ConvoyController::declare_parameters()
{
  robot_.declare_parameter<std::string>("robot_id", robot_id_);
  robot_.declare_parameter<bool>("convoy_enable", enabled_);
  robot_.declare_parameter<std::string>("convoy_role", "follower");
  robot_.declare_parameter<double>("cruise_speed", cruise_speed_);
  robot_.declare_parameter<double>("turbo_speed", turbo_speed_);
  robot_.declare_parameter<double>("sync_pause", sync_pause_s_);
  robot_.declare_parameter<double>("leader_timeout", leader_timeout_s_);
  robot_.declare_parameter<double>("color_min_saturation", color_min_saturation_);
  robot_.declare_parameter<double>("color_min_clear", color_min_clear_);
  robot_.declare_parameter<int>("color_debounce", color_debounce_);
  robot_.declare_parameter<double>("color_cooldown", color_cooldown_s_);
}

void ConvoyController::load_parameters()
{
  robot_id_ = robot_.get_parameter("robot_id").as_string();
  enabled_ = robot_.get_parameter("convoy_enable").as_bool();
  const auto role = robot_.get_parameter("convoy_role").as_string();
  role_ = (role == "leader") ? Role::Leader : Role::Follower;
  if (role_ == Role::Leader) {
    term_ = 1;
  }
  cruise_speed_ = robot_.get_parameter("cruise_speed").as_double();
  turbo_speed_ = robot_.get_parameter("turbo_speed").as_double();
  sync_pause_s_ = robot_.get_parameter("sync_pause").as_double();
  leader_timeout_s_ = robot_.get_parameter("leader_timeout").as_double();
  color_min_saturation_ = robot_.get_parameter("color_min_saturation").as_double();
  color_min_clear_ = robot_.get_parameter("color_min_clear").as_double();
  color_debounce_ = robot_.get_parameter("color_debounce").as_int();
  color_cooldown_s_ = robot_.get_parameter("color_cooldown").as_double();
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
      become_leader(drive_command());
    } else {
      become_follower();
    }
    return result;
  }
  if (name == "convoy_enable") {
    enabled_ = param.as_bool();
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
  if (value < 0.0) {
    result.successful = false;
    result.reason = name + " cannot be negative";
    return result;
  }
  if (name == "cruise_speed") {
    cruise_speed_ = value;
  } else if (name == "turbo_speed") {
    turbo_speed_ = value;
  } else if (name == "sync_pause") {
    sync_pause_s_ = value;
  } else if (name == "leader_timeout") {
    leader_timeout_s_ = value;
  } else if (name == "color_min_saturation") {
    color_min_saturation_ = value;
  } else if (name == "color_min_clear") {
    color_min_clear_ = value;
  } else if (name == "color_cooldown") {
    color_cooldown_s_ = value;
  }
  return result;
}

void ConvoyController::on_color(const ColorReading & reading)
{
  if (!enabled_ || !running_) {
    return;
  }
  // The eyes are already counting a pause down, and the wheels are parked: taking a
  // second card here would restart the sequence the follower is waiting on.
  if (sync_kind_ != SyncKind::None) {
    return;
  }

  std::string card;
  if (!accept_card(reading, card)) {
    return;
  }

  if (role_ != Role::Leader) {
    flash_reject();
    return;
  }

  if (card == "blue") {
    if (!peer_.valid) {
      flash_reject();
      RCLCPP_INFO(robot_.get_logger(), "Blue card ignored: no peer on the network");
      return;
    }
    start_handover();
    return;
  }
  if (card == "red") {
    // A stop is not worth a countdown, and both robots brake on their own copy.
    flash_card(card);
    apply_drive(Command::Stop);
    publish_command(Command::Stop);
    return;
  }

  const Command next = (card == "yellow") ? Command::Turbo : Command::Cruise;
  // Hand the order over before applying it: this robot used to speed up here and pull
  // away while the follower was still waiting for /convoy/command.
  publish_command(next);
  if (next == drive_command()) {
    flash_card(card);
    return;
  }
  begin_sync(SyncKind::Speed, next);
}

void ConvoyController::tick()
{
  if (!enabled_) {
    return;
  }

  const auto stamp = robot_.now();
  if (peer_.valid && (stamp - peer_.stamp).seconds() > leader_timeout_s_ * 2.0) {
    peer_.valid = false;
    if (had_peer_) {
      peer_lost_ = true;
    }
  }
  if (peer_.valid && peer_.role == Role::Leader) {
    last_leader_seen_ = stamp;
  }

  if (sync_kind_ == SyncKind::None) {
    // Mid-handover both robots claim the lead for a tick, which resolve_roles would
    // read as a clash and undo the swap.
    maybe_elect();
    resolve_roles();
  }

  if (!running_) {
    robot_.set_convoy_hold(true);
    // Do not fight teleop: previously this stop() ran every 50 ms and made the
    // robot stutter. Leave the wheels alone while a recent cmd_vel is in play.
    if (!robot_.teleop_recent()) {
      robot_.stop();
    }
    last_v_ = 0.0;
    apply_leds();
    publish_peer();
    return;
  }

  if (sync_kind_ != SyncKind::None) {
    if (stamp < sync_until_) {
      apply_sync_leds();
      publish_peer();
      return;
    }
    finish_sync();
  }

  apply_motion();
  apply_leds();
  publish_peer();
}

void ConvoyController::maybe_elect()
{
  if (role_ != Role::Follower) {
    return;
  }
  if (peer_.valid && peer_.role == Role::Leader) {
    return;
  }
  if ((robot_.now() - last_leader_seen_).seconds() <= leader_timeout_s_) {
    return;
  }
  RCLCPP_WARN(robot_.get_logger(), "No leader on the network; this robot is taking the lead");
  become_leader(drive_command());
  if (running_) {
    publish_command(drive_command());
  }
}

void ConvoyController::on_command(const std_msgs::msg::String & msg)
{
  if (!running_ || role_ != Role::Follower) {
    return;
  }
  Command cmd = Command::Cruise;
  if (!parse_command(msg.data, cmd)) {
    return;
  }
  if (cmd == Command::Stop) {
    // Brake now and drop any pause in progress, rather than finishing the countdown.
    sync_kind_ = SyncKind::None;
    flash_card(card_name(cmd));
    apply_drive(cmd);
    return;
  }
  if (sync_kind_ != SyncKind::None) {
    return;
  }
  if (cmd == Command::Handover) {
    begin_sync(SyncKind::TakeLead, drive_command());
    return;
  }
  if (cmd == drive_command()) {
    // A new leader re-announces the speed already in effect, which needs no pause.
    return;
  }
  begin_sync(SyncKind::Speed, cmd);
}

void ConvoyController::on_peer(const std_msgs::msg::String & msg)
{
  const auto parts = split_csv(msg.data);
  if (parts.size() < 5) {
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
    const bool was_valid = peer_.valid;
    peer_.id = parts[0];
    peer_.role = role;
    peer_.cmd = cmd;
    // parts[3] is the peer's speed, published for `ros2 topic echo` only: nothing here
    // steers by it now that both robots run the speed that came over /convoy/command.
    peer_.seq = static_cast<std::uint32_t>(std::stoul(parts[4]));
    peer_.term = (parts.size() >= 6) ? static_cast<std::uint32_t>(std::stoul(parts[5])) : 0;
    peer_.stamp = robot_.now();
    peer_.valid = true;
    had_peer_ = true;
    if (!was_valid) {
      if (peer_lost_) {
        flash_rgb(1.0, 1.0, 1.0);
      }
      peer_lost_ = false;
    }
  } catch (const std::exception &) {
    return;
  }
}

void ConvoyController::on_run(const std_msgs::msg::String & msg)
{
  std::string order = msg.data;
  while (!order.empty() && (order.back() == '\n' || order.back() == '\r' || order.back() == ' ')) {
    order.pop_back();
  }
  if (order == "START" || order == "start" || order == "GO" || order == "go") {
    set_running(true);
  } else if (order == "STOP" || order == "stop") {
    set_running(false);
  }
}

void ConvoyController::set_running(bool running)
{
  if (!enabled_ || running_ == running) {
    return;
  }
  running_ = running;
  if (!running_) {
    robot_.set_convoy_hold(true);
    robot_.stop();
    last_v_ = 0.0;
    sync_kind_ = SyncKind::None;
    RCLCPP_INFO(robot_.get_logger(), "Convoy paused; publish START on /convoy/run to resume");
    return;
  }
  flash_rgb(1.0, 1.0, 1.0);
  if (role_ == Role::Leader) {
    apply_drive(Command::Cruise);
    publish_command(Command::Cruise);
  }
  RCLCPP_INFO(robot_.get_logger(), "Convoy START");
}

void ConvoyController::apply_drive(Command cmd)
{
  if (cmd == Command::Handover) {
    return;
  }
  last_drive_cmd_ = cmd;
}

void ConvoyController::apply_motion()
{
  const Command drive = drive_command();
  if (drive == Command::Stop) {
    robot_.set_convoy_hold(true);
    robot_.stop();
    last_v_ = 0.0;
    return;
  }

  // Leader and follower run the same table of speeds off the same order, so there is
  // nothing to correct: the pause in begin_sync() is what keeps them together. An
  // encoder gap controller used to live here and would slam one robot to the speed
  // cap while the other sat at zero, because the two odometers had no shared origin.
  robot_.set_convoy_hold(false);
  last_v_ = command_speed(drive);
  robot_.set_follow_speed(last_v_);
}

void ConvoyController::begin_sync(SyncKind kind, Command cmd)
{
  sync_kind_ = kind;
  sync_cmd_ = cmd;
  sync_until_ = robot_.now() + rclcpp::Duration::from_seconds(sync_pause_s_);

  const Command colour_of = (kind == SyncKind::Speed) ? cmd : Command::Handover;
  const auto rgb = card_rgb(card_name(colour_of));
  sync_r_ = rgb.r * kEyeBrightness;
  sync_g_ = rgb.g * kEyeBrightness;
  sync_b_ = rgb.b * kEyeBrightness;
  // The countdown owns the eyes while it runs, so drop any card flash still pending.
  flash_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Park the wheels once: convoy_hold_ keeps the line follower and the watchdog off
  // them until finish_sync(), so there is no need to re-stop on every tick.
  robot_.set_convoy_hold(true);
  robot_.stop();
  last_v_ = 0.0;
}

void ConvoyController::finish_sync()
{
  const SyncKind kind = sync_kind_;
  sync_kind_ = SyncKind::None;

  switch (kind) {
    case SyncKind::TakeLead:
      become_leader(sync_cmd_);
      publish_command(drive_command());
      RCLCPP_INFO(robot_.get_logger(), "Handover done: this robot leads (term %u)", term_);
      break;
    case SyncKind::GiveLead:
      become_follower();
      RCLCPP_INFO(robot_.get_logger(), "Handover done: this robot follows");
      break;
    case SyncKind::Speed:
    case SyncKind::None:
      apply_drive(sync_cmd_);
      break;
  }
}

void ConvoyController::apply_sync_leds()
{
  const double left = (sync_until_ - robot_.now()).seconds();
  // Blink the card colour down to the last stretch, then hold it solid: the stand can
  // see both robots waiting on each other and then leaving together.
  const bool on = (left <= kSyncGoSeconds) ||
    (static_cast<int>(left / kSyncBlinkSeconds) % 2 == 0);
  robot_.set_eyes(on ? sync_r_ : 0.0, on ? sync_g_ : 0.0, on ? sync_b_ : 0.0);
}

void ConvoyController::apply_leds()
{
  const auto stamp = robot_.now();
  if (flash_until_.nanoseconds() != 0 && stamp < flash_until_) {
    robot_.set_eyes(flash_r_, flash_g_, flash_b_);
    return;
  }

  if (!running_) {
    const double v =
      (static_cast<int>(stamp.nanoseconds() / 400000000) % 2 == 0) ? kEyeBrightness : 0.0;
    robot_.set_eyes(v, v, v);
    return;
  }
  if (role_ == Role::Follower) {
    robot_.set_eyes(0.0, 0.0, 0.0);
    return;
  }
  if (!peer_.valid) {
    const double v = peer_lost_ && (static_cast<int>(stamp.nanoseconds() / 250000000) % 2 == 0)
                       ? kEyeBrightness
                       : (peer_lost_ ? 0.0 : kEyeBrightness);
    robot_.set_eye_left(v, v, v);
    robot_.set_eye_right(0.0, 0.0, 0.0);
    return;
  }
  robot_.set_eyes(kEyeBrightness, kEyeBrightness, kEyeBrightness);
}

void ConvoyController::become_leader(Command cmd)
{
  term_ = std::max(term_, peer_.term) + 1;
  role_ = Role::Leader;
  apply_drive(cmd == Command::Handover ? Command::Cruise : cmd);
}

void ConvoyController::become_follower()
{
  role_ = Role::Follower;
  last_leader_seen_ = robot_.now();
}

void ConvoyController::start_handover()
{
  publish_command(Command::Handover);
  begin_sync(SyncKind::GiveLead, drive_command());
  RCLCPP_INFO(
    robot_.get_logger(), "Handover to '%s': both robots pause, then swap roles",
    peer_.id.c_str());
}

void ConvoyController::resolve_roles()
{
  if (!peer_.valid || peer_.role != Role::Leader || role_ != Role::Leader) {
    return;
  }
  const bool lose = (term_ < peer_.term) || (term_ == peer_.term && robot_id_ > peer_.id);
  if (!lose) {
    return;
  }
  RCLCPP_WARN(
    robot_.get_logger(),
    "Yielding lead to '%s' (term %u vs %u)", peer_.id.c_str(), peer_.term, term_);
  become_follower();
  flash_rgb(1.0, 1.0, 1.0);
}

void ConvoyController::flash_rgb(double red, double green, double blue)
{
  flash_r_ = red * kEyeBrightness;
  flash_g_ = green * kEyeBrightness;
  flash_b_ = blue * kEyeBrightness;
  flash_until_ = robot_.now() + rclcpp::Duration::from_seconds(kFlashSeconds);
}

void ConvoyController::flash_card(const std::string & card)
{
  const auto rgb = card_rgb(card);
  flash_rgb(rgb.r, rgb.g, rgb.b);
}

void ConvoyController::flash_reject()
{
  flash_rgb(1.0, 0.0, 0.0);
}

bool ConvoyController::accept_card(const ColorReading & reading, std::string & card)
{
  if (!is_card(reading, card)) {
    debounce_name_.clear();
    debounce_count_ = 0;
    return false;
  }
  if (card == debounce_name_) {
    ++debounce_count_;
  } else {
    debounce_name_ = card;
    debounce_count_ = 1;
  }
  if (debounce_count_ < color_debounce_) {
    return false;
  }
  const auto stamp = robot_.now();
  if (card == last_card_ && last_card_time_.nanoseconds() != 0 &&
      (stamp - last_card_time_).seconds() < color_cooldown_s_) {
    return false;
  }
  last_card_ = card;
  last_card_time_ = stamp;
  return true;
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

ConvoyController::Command ConvoyController::drive_command() const
{
  return last_drive_cmd_;
}

void ConvoyController::publish_command(Command cmd)
{
  std_msgs::msg::String msg;
  msg.data = command_name(cmd);
  command_pub_->publish(msg);
}

void ConvoyController::publish_peer()
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(3);
  out << robot_id_ << ',' << role_name(role_) << ',' << command_name(last_drive_cmd_) << ','
      << last_v_ << ',' << seq_ << ',' << term_;
  ++seq_;
  std_msgs::msg::String msg;
  msg.data = out.str();
  peer_pub_->publish(msg);
}

const char * ConvoyController::role_name(Role role)
{
  return (role == Role::Leader) ? "leader" : "follower";
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

const char * ConvoyController::card_name(Command cmd)
{
  switch (cmd) {
    case Command::Stop:
      return "red";
    case Command::Cruise:
      return "green";
    case Command::Turbo:
      return "yellow";
    case Command::Handover:
      return "blue";
  }
  return "green";
}

bool ConvoyController::parse_role(const std::string & text, Role & role)
{
  if (text == "leader" || text == "leader_siding" || text == "passing") {
    role = Role::Leader;
    return true;
  }
  if (text == "follower" || text == "waiting") {
    role = Role::Follower;
    return true;
  }
  return false;
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
