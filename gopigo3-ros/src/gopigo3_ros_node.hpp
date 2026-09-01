#pragma once

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gopigo3_driver.hpp"

// Translates the ROS 2 interface (a geometry_msgs/msg/Twist stream) into GoPiGo3 wheel
// commands.
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

  GoPiGo3Driver driver_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  rclcpp::Time last_cmd_time_;
  bool stopped_{true};

  double max_linear_speed_;
  double max_angular_speed_;
  double max_motor_dps_;
  rclcpp::Duration cmd_timeout_{0, 0};
};
