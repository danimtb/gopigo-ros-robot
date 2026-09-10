#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class GoPiGo3;

enum class GroveI2cPort
{
  Off,
  I2c,  // Grove I2C on the GoPiGo3 (Raspberry Pi /dev/i2c-1)
  Ad1,  // Grove AD1, I2C over the GoPiGo3 firmware
  Ad2
};

using LineFollowerPort = GroveI2cPort;

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

// Dexter Light & Color Sensor (TCS34725). Channels are 0…1.
struct ColorReading
{
  double red{0.0};
  double green{0.0};
  double blue{0.0};
  double clear{0.0};
  double saturation{0.0};
  double value{0.0};
  std::string name{"unknown"};
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
  bool init_line_follower(GroveI2cPort port);

  // Sensors at or above white_threshold count as bare surface and carry no weight in the
  // position estimate: counting them drags every reading towards the centre.
  bool read_line_follower(LineFollowerReading & reading, double white_threshold);

  LineFollowerKind line_follower_kind() const { return lf_kind_; }

  // Probe the Dexter TCS34725. led_on lights the board LED for reflected colour.
  bool init_color_sensor(GroveI2cPort port, bool led_on);

  bool read_color(ColorReading & reading);

  // Board LEDs on the red GoPiGo3 (eyes RGB, blinkers red). Colour and brightness are 0…1.
  void set_eye_left(double red, double green, double blue);
  void set_eye_right(double red, double green, double blue);
  void set_eyes(double red, double green, double blue);
  void set_blinker_left(double brightness);
  void set_blinker_right(double brightness);
  void set_blinkers(double brightness);

  // Wheel speeds in degrees per second of wheel-shaft rotation.
  void set_wheel_speeds(double left_dps, double right_dps);

  void stop();

  // Encoder ticks are degrees of wheel-shaft rotation since connect() (or the last offset).
  bool read_encoders(int32_t & left, int32_t & right);

  // Mean arc length of both wheels, metres. Returns 0 when the board is not connected.
  double path_length_m();

  double wheel_radius() const;      // metres
  double wheel_separation() const;  // metres

private:
  bool ensure_port(GroveI2cPort port);
  uint8_t grove_id(GroveI2cPort port) const;
  bool grove_i2c(
    uint8_t grove_port, uint8_t address, const uint8_t * write_bytes, uint8_t write_len,
    uint8_t * read_bytes, uint8_t read_len);
  bool rpi_one_shot_i2c(uint8_t address, uint8_t * bytes, uint8_t len, bool read);
  bool rpi_i2c(
    uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
    uint8_t read_len, unsigned settle_us);
  // settle_us waits between the register write and the read, needed by the slower sensors.
  bool sensor_i2c(
    GroveI2cPort port, uint8_t address, const uint8_t * write_bytes, uint8_t write_len,
    uint8_t * read_bytes, uint8_t read_len, unsigned settle_us = 1000);

  bool detect_line_follower();
  bool read_black(LineFollowerReading & reading, double white_threshold);
  bool read_red(LineFollowerReading & reading, double white_threshold);
  void estimate_line(LineFollowerReading & reading, double white_threshold) const;

  bool tcs_write8(uint8_t reg, uint8_t value);
  bool tcs_read(uint8_t reg, uint8_t * data, uint8_t len);
  bool tcs_set_led(bool on);
  void guess_color(ColorReading & reading) const;

  std::unique_ptr<GoPiGo3> gpg_;
  GroveI2cPort lf_port_{GroveI2cPort::Off};
  GroveI2cPort color_port_{GroveI2cPort::Off};
  LineFollowerKind lf_kind_{LineFollowerKind::None};
  uint8_t tcs_atime_{254};  // 2 × 2.4 ms, Dexter default
  int i2c_fd_{-1};

  // SPI LED writes are relatively slow; skip them when the PWM has not changed.
  bool eye_left_valid_{false};
  bool eye_right_valid_{false};
  uint8_t eye_left_r_{0};
  uint8_t eye_left_g_{0};
  uint8_t eye_left_b_{0};
  uint8_t eye_right_r_{0};
  uint8_t eye_right_g_{0};
  uint8_t eye_right_b_{0};
};
