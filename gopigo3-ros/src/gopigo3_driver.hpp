#pragma once

#include <memory>

class GoPiGo3;

// GoPiGo3.h defines non-inline globals (spi_file_handle, spi_setup(), ...) at namespace
// scope, so it may only be included from a single translation unit. gopigo3_driver.cpp is
// that unit; the rest of the project talks to the robot through this wrapper.
class GoPiGo3Driver
{
public:
  GoPiGo3Driver();
  ~GoPiGo3Driver();

  // Handshake with the GoPiGo3 board and reset both encoders.
  // Throws std::runtime_error when SPI is unavailable or the firmware does not match.
  void connect();

  // Wheel speeds in degrees per second of wheel-shaft rotation.
  void set_wheel_speeds(double left_dps, double right_dps);

  void stop();

  double wheel_radius() const;      // metres
  double wheel_separation() const;  // metres

private:
  std::unique_ptr<GoPiGo3> gpg_;
};
