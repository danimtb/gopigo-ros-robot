#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "gopigo3_driver.hpp"

class ConvoyController;

// Translates the ROS 2 interface (a geometry_msgs/msg/Twist stream) into GoPiGo3 wheel
// commands, and optionally publishes / follows the Dexter line follower and color sensor.
class GoPiGo3RosNode : public rclcpp::Node
{
public:
  GoPiGo3RosNode();
  ~GoPiGo3RosNode() override;

  void set_follow_speed(double speed);
  double follow_speed() const { return line_follow_speed_; }
  void set_steer_bias(double bias_rad);
  void set_convoy_hold(bool hold);
  void stop();
  double path_length_m();
  void set_eyes(double red, double green, double blue);
  void set_blinker_left(double brightness);
  void set_blinker_right(double brightness);

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg);
  void on_timer();
  void drive(const geometry_msgs::msg::Twist & msg);
  void publish_line_follower();
  void publish_color();
  void follow_line(const LineFollowerReading & reading);
  GroveI2cPort parse_grove_i2c_port(const std::string & name, const char * param) const;
  std::string relative_topic(std::string name, const char * param) const;
  void on_eye_left(const std_msgs::msg::ColorRGBA & msg);
  void on_eye_right(const std_msgs::msg::ColorRGBA & msg);
  void on_eyes(const std_msgs::msg::ColorRGBA & msg);
  void on_blinker_left(const std_msgs::msg::Float32 & msg);
  void on_blinker_right(const std_msgs::msg::Float32 & msg);
  void on_blinkers(const std_msgs::msg::Float32 & msg);
  // Lets `ros2 param set` switch line follow on and off, and retune it, while it drives.
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  GoPiGo3Driver driver_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr line_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_position_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr line_state_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr color_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_name_pub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr eye_left_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr eye_right_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr eyes_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr blinker_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr blinker_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr blinkers_sub_;
  rclcpp::TimerBase::SharedPtr line_timer_;
  rclcpp::TimerBase::SharedPtr color_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  std::unique_ptr<ConvoyController> convoy_;

  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_teleop_time_;
  rclcpp::Time last_line_time_;
  rclcpp::Time line_lost_since_;
  double last_line_error_{0.0};
  bool stopped_{true};
  bool line_follower_ready_{false};
  bool line_follow_{false};
  bool color_ready_{false};
  bool convoy_hold_{false};
  double steer_bias_{0.0};

  double max_linear_speed_;
  double max_angular_speed_;
  double max_motor_dps_;
  double line_follow_speed_;
  double line_follow_kp_;
  double line_follow_kd_;
  double line_follow_slowdown_;
  double line_threshold_;
  rclcpp::Duration cmd_timeout_{0, 0};
  rclcpp::Duration line_search_timeout_{0, 0};
};
