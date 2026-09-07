#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "gopigo3_driver.hpp"

// Translates the ROS 2 interface (a geometry_msgs/msg/Twist stream) into GoPiGo3 wheel
// commands, and optionally publishes / follows the Dexter line follower.
class GoPiGo3RosNode : public rclcpp::Node
{
public:
  GoPiGo3RosNode();
  ~GoPiGo3RosNode() override;

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);
  void on_timer();
  void drive(const geometry_msgs::msg::Twist & msg);
  void stop();
  void publish_line_follower();
  void follow_line(const LineFollowerReading & reading);
  LineFollowerPort parse_line_follower_port(const std::string & name) const;

  GoPiGo3Driver driver_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr line_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr line_state_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_teleop_time_;
  bool stopped_{true};
  bool line_follower_ready_{false};
  bool line_follow_{false};

  double max_linear_speed_;
  double max_angular_speed_;
  double max_motor_dps_;
  double line_follow_speed_;
  double line_follow_kp_;
  rclcpp::Duration cmd_timeout_{0, 0};
};
