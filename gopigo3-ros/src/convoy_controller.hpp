º #pragma once

#include <cstdint>
#include <string>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "gopigo3_driver.hpp"

class GoPiGo3RosNode;

// In-process leader/follower state machine. Publishes global /convoy topics and only
// changes line-follow speed, steer bias, hold, and board LEDs on the robot node.
class ConvoyController
{
public:
  explicit ConvoyController(GoPiGo3RosNode & robot);

  void on_color(const ColorReading & reading);
  void tick();

  rcl_interfaces::msg::SetParametersResult apply_parameter(const rclcpp::Parameter & param);

private:
  enum class Role
  {
    Leader,
    Follower,
    LeaderSiding,
    Waiting,
    Passing
  };

  enum class Command
  {
    Stop,
    Cruise,
    Turbo,
    Handover
  };

  struct Peer
  {
    std::string id;
    Role role{Role::Follower};
    Command cmd{Command::Cruise};
    double s_m{0.0};
    double v_mps{0.0};
    std::uint32_t seq{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void declare_parameters();
  void load_parameters();
  void publish_command();
  void publish_peer();
  void on_command(const std_msgs::msg::String & msg);
  void on_peer(const std_msgs::msg::String & msg);
  void apply_command(Command cmd, bool from_network);
  void apply_motion();
  void apply_leds();
  void become_leader(Command cmd);
  void become_follower();
  void start_handover();
  void reset_path_offset();
  void resolve_roles();
  bool peer_is_leader() const;
  bool is_card(const ColorReading & reading, std::string & card) const;
  double command_speed(Command cmd) const;
  double reported_path() const;
  static const char * role_name(Role role);
  static const char * command_name(Command cmd);
  static bool parse_role(const std::string & text, Role & role);
  static bool parse_command(const std::string & text, Command & cmd);

  GoPiGo3RosNode & robot_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr peer_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr peer_sub_;

  std::string robot_id_{"a"};
  bool enabled_{false};
  Role role_{Role::Leader};
  Command cmd_{Command::Cruise};
  Peer peer_;

  double cruise_speed_{0.10};
  double turbo_speed_{0.18};
  double gap_target_m_{0.0};
  double gap_kp_{0.8};
  double leader_timeout_s_{0.8};
  double color_min_saturation_{0.25};
  double color_min_clear_{0.05};
  int color_debounce_{2};
  double color_cooldown_s_{1.2};
  double handover_bias_rad_{1.2};
  double handover_fork_s_{0.6};
  double handover_siding_s_{1.2};
  double handover_pass_m_{0.6};
  double handover_pass_timeout_s_{8.0};
  double handover_rejoin_s_{1.0};

  std::string debounce_name_;
  int debounce_count_{0};
  std::string last_card_;
  rclcpp::Time last_card_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_leader_seen_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rejoin_since_{0, 0, RCL_ROS_TIME};
  double s_offset_{0.0};
  double pass_s0_{0.0};
  std::uint32_t seq_{0};
  double last_v_{0.0};
};
