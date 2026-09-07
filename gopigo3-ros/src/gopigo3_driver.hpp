#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class GoPiGo3;

enum class LineFollowerPort
{
  Off,
  I2c,  // Grove I2C on the GoPiGo3 (Raspberry Pi /dev/i2c-1)
  Ad1,  // Grove AD1, black board only in Dexter's docs
  Ad2
};

enum class LineFollowerKind
{
  None,
  Black,  // V2, six IR sensors
  Red     // five IR sensors
};

// 0 = black, 1 = white. Sensors are ordered left → right with the board arrow forward.
struct LineFollowerReading
{
  std::array<double, 6> sensors{};
  std::size_t count{0};
  double position{0.5};  // 0 = line on the left, 0.5 = centre, 1 = right
  int lost{0};           // 0 = line seen, 1 = all black, 2 = all white
  std::string state{"unknown"};
};

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

  // Probe the Dexter line follower. Returns false when the port is Off or no sensor answers.
  bool init_line_follower(LineFollowerPort port);

  bool read_line_follower(LineFollowerReading & reading);

  LineFollowerKind line_follower_kind() const { return lf_kind_; }

  // Wheel speeds in degrees per second of wheel-shaft rotation.
  void set_wheel_speeds(double left_dps, double right_dps);

  void stop();

  double wheel_radius() const;      // metres
  double wheel_separation() const;  // metres

private:
  bool grove_i2c(
    uint8_t grove_port, uint8_t address, const uint8_t * write_bytes, uint8_t write_len,
    uint8_t * read_bytes, uint8_t read_len);
  bool rpi_i2c(
    uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
    uint8_t read_len);
  bool sensor_i2c(
    uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
    uint8_t read_len);

  bool detect_line_follower();
  bool read_black(LineFollowerReading & reading);
  bool read_red(LineFollowerReading & reading);
  void estimate_line(LineFollowerReading & reading) const;

  std::unique_ptr<GoPiGo3> gpg_;
  LineFollowerPort lf_port_{LineFollowerPort::Off};
  LineFollowerKind lf_kind_{LineFollowerKind::None};
  uint8_t grove_port_{0};
  int i2c_fd_{-1};
};
