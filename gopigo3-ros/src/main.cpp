#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "gopigo3_ros_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GoPiGo3RosNode> node;
  try {
    node = std::make_shared<GoPiGo3RosNode>();
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("gopigo3_ros"),
      "Cannot talk to the GoPiGo3 board (%s). Check that SPI is enabled "
      "(raspi-config), that the robot is powered on, and that you can read "
      "/dev/spidev0.1.", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);

  // Release the node before shutdown so its destructor stops the motors while the
  // context is still valid.
  node.reset();
  rclcpp::shutdown();
  return 0;
}
