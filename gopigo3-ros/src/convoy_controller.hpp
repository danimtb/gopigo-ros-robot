#pragma once

#include <cstdint>
#include <string>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "gopigo3_driver.hpp"

class GoPiGo3RosNode;

// In-process leader/follower state machine. Publishes global /convoy topics and only
// changes line-follow speed, hold, and board LEDs on the robot node.
class ConvoyController
{
public:
  ConvoyController(
    GoPiGo3RosNode & robot,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr peer_pub);

  void on_color(const ColorReading & reading);
  void tick();
  void on_command(const std_msgs::msg::String & msg);
  void on_peer(const std_msgs::msg::String & msg);
  void on_run(const std_msgs::msg::String & msg);

  rcl_interfaces::msg::SetParametersResult apply_parameter(const rclcpp::Parameter & param);

private:
  enum class Role
  {
    Leader,
    Follower
  };

  enum class Command
  {
    Stop,
    Cruise,
    Turbo,
    Handover
  };

  // Both robots sit out the same pause before a speed or role change, so neither one
  // pulls away while the order is still in flight.
  enum class SyncKind
  {
    None,
    Speed,
    TakeLead,
    GiveLead
  };

  struct Peer
  {
    std::string id;
    Role role{Role::Follower};
    Command cmd{Command::Cruise};
    std::uint32_t seq{0};
    std::uint32_t term{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void declare_parameters();
  void load_parameters();
  void publish_command(Command cmd);
  void publish_peer();
  void apply_drive(Command cmd);
  void apply_motion();
  void apply_leds();
  void begin_sync(SyncKind kind, Command cmd);
  void finish_sync();
  void apply_sync_leds();
  void become_leader(Command cmd);
  void become_follower();
  void start_handover();
  void maybe_elect();
  void resolve_roles();
  void set_running(bool running);
  void flash_rgb(double red, double green, double blue);
  void flash_card(const std::string & card);
  void flash_reject();
  bool accept_card(const ColorReading & reading, std::string & card);
  bool is_card(const ColorReading & reading, std::string & card) const;
  double command_speed(Command cmd) const;
  Command drive_command() const;
  static const char * role_name(Role role);
  static const char * command_name(Command cmd);
  static const char * card_name(Command cmd);
  static bool parse_role(const std::string & text, Role & role);
  static bool parse_command(const std::string & text, Command & cmd);

  GoPiGo3RosNode & robot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr peer_pub_;

  std::string robot_id_{"a"};
  bool enabled_{false};
  bool running_{false};
  Role role_{Role::Follower};
  Command last_drive_cmd_{Command::Cruise};
  Peer peer_;
  std::uint32_t term_{0};

  double cruise_speed_{0.06};
  double turbo_speed_{0.09};
  double sync_pause_s_{1.2};
  double leader_timeout_s_{3.0};
  double color_min_saturation_{0.25};
  double color_min_clear_{0.05};
  int color_debounce_{2};
  double color_cooldown_s_{3.0};

  std::string debounce_name_;
  int debounce_count_{0};
  std::string last_card_;
  rclcpp::Time last_card_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leader_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time flash_until_{0, 0, RCL_ROS_TIME};
  double flash_r_{0.0};
  double flash_g_{0.0};
  double flash_b_{0.0};
  SyncKind sync_kind_{SyncKind::None};
  Command sync_cmd_{Command::Cruise};
  rclcpp::Time sync_until_{0, 0, RCL_ROS_TIME};
  double sync_r_{0.0};
  double sync_g_{0.0};
  double sync_b_{0.0};
  std::uint32_t seq_{0};
  double last_v_{0.0};
  bool had_peer_{false};
  bool peer_lost_{false};
};
