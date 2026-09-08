#include "gopigo3_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

#include <GoPiGo3.h>

namespace
{
constexpr uint8_t kLineFollowerAddress = 0x06;
// The line follower needs the register write to settle before it answers: 10 ms is what
// danimtb/robotapp's line_sensor.cpp waits on this same board.
constexpr unsigned kLineFollowerSettleUs = 10000;
// Only used while probing, where the position estimate is thrown away.
constexpr double kDefaultWhiteThreshold = 0.6;
constexpr uint8_t kTcsAddress = 0x29;
constexpr uint8_t kTcsCommand = 0x80;
constexpr uint8_t kTcsEnable = 0x00;
constexpr uint8_t kTcsEnableAien = 0x10;
constexpr uint8_t kTcsEnableAen = 0x02;
constexpr uint8_t kTcsEnablePon = 0x01;
constexpr uint8_t kTcsAtime = 0x01;
constexpr uint8_t kTcsPers = 0x0C;
constexpr uint8_t kTcsControl = 0x0F;
constexpr uint8_t kTcsId = 0x12;
constexpr uint8_t kTcsCdata = 0x14;
constexpr uint8_t kTcsGain16x = 0x02;

int16_t to_dps(double value)
{
  constexpr double kMax = std::numeric_limits<int16_t>::max();
  return static_cast<int16_t>(std::lround(std::clamp(value, -kMax, kMax)));
}

uint8_t to_pwm(double value)
{
  return static_cast<uint8_t>(std::lround(std::clamp(value, 0.0, 1.0) * 255.0));
}

std::string bytes_to_string(const uint8_t * data, std::size_t n)
{
  std::string out;
  for (std::size_t i = 0; i < n && data[i] != 0; ++i) {
    out.push_back(static_cast<char>(data[i]));
  }
  return out;
}
}  // namespace

GoPiGo3Driver::GoPiGo3Driver() = default;

GoPiGo3Driver::~GoPiGo3Driver()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  if (gpg_) {
    gpg_->reset_all();
  }
}

void GoPiGo3Driver::connect()
{
  // Both the constructor (SPI setup) and detect() throw std::runtime_error on failure.
  gpg_ = std::make_unique<GoPiGo3>();
  gpg_->detect();
  gpg_->offset_motor_encoder(MOTOR_LEFT, gpg_->get_motor_encoder(MOTOR_LEFT));
  gpg_->offset_motor_encoder(MOTOR_RIGHT, gpg_->get_motor_encoder(MOTOR_RIGHT));
}

bool GoPiGo3Driver::ensure_port(GroveI2cPort port)
{
  if (port == GroveI2cPort::Off) {
    return false;
  }
  if (port == GroveI2cPort::I2c) {
    if (i2c_fd_ < 0) {
      i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    }
    return i2c_fd_ >= 0;
  }

  gpg_->set_grove_type(grove_id(port), GROVE_TYPE_I2C);
  usleep(10000);
  return true;
}

uint8_t GoPiGo3Driver::grove_id(GroveI2cPort port) const
{
  return (port == GroveI2cPort::Ad1) ? GROVE_1 : GROVE_2;
}

bool GoPiGo3Driver::init_line_follower(GroveI2cPort port)
{
  lf_port_ = port;
  lf_kind_ = LineFollowerKind::None;
  if (!ensure_port(port)) {
    return false;
  }
  return detect_line_follower();
}

bool GoPiGo3Driver::init_color_sensor(GroveI2cPort port, bool led_on)
{
  color_port_ = GroveI2cPort::Off;
  if (!ensure_port(port)) {
    return false;
  }
  color_port_ = port;

  uint8_t chip_id = 0;
  if (!tcs_read(kTcsId, &chip_id, 1) || (chip_id != 0x44 && chip_id != 0x4D)) {
    color_port_ = GroveI2cPort::Off;
    return false;
  }

  // Dexter default: 4.8 ms integration, 16× gain.
  tcs_atime_ = 254;
  if (!tcs_write8(kTcsAtime, tcs_atime_) || !tcs_write8(kTcsControl, kTcsGain16x)) {
    color_port_ = GroveI2cPort::Off;
    return false;
  }

  if (!tcs_write8(kTcsEnable, kTcsEnablePon)) {
    color_port_ = GroveI2cPort::Off;
    return false;
  }
  usleep(10000);
  if (!tcs_write8(kTcsEnable, static_cast<uint8_t>(kTcsEnablePon | kTcsEnableAen))) {
    color_port_ = GroveI2cPort::Off;
    return false;
  }

  if (!tcs_set_led(led_on)) {
    color_port_ = GroveI2cPort::Off;
    return false;
  }
  usleep(static_cast<useconds_t>((256 - tcs_atime_) * 2400 * 2));
  return true;
}

bool GoPiGo3Driver::read_line_follower(LineFollowerReading & reading, double white_threshold)
{
  reading = {};
  if (lf_kind_ == LineFollowerKind::Black) {
    return read_black(reading, white_threshold);
  }
  if (lf_kind_ == LineFollowerKind::Red) {
    return read_red(reading, white_threshold);
  }
  return false;
}

bool GoPiGo3Driver::grove_i2c(
  uint8_t grove_port, uint8_t address, const uint8_t * write_bytes, uint8_t write_len,
  uint8_t * read_bytes, uint8_t read_len)
{
  i2c_struct_t xfer{};
  xfer.address = address;
  xfer.length_write = write_len;
  xfer.length_read = read_len;
  if (write_len > 0 && write_bytes != nullptr) {
    std::memcpy(xfer.buffer_write, write_bytes, write_len);
  }

  const int error = gpg_->grove_i2c_transfer(grove_port, &xfer);
  if (error != ERROR_NONE) {
    return false;
  }
  if (read_len > 0 && read_bytes != nullptr) {
    std::memcpy(read_bytes, xfer.buffer_read, read_len);
  }
  return true;
}

bool GoPiGo3Driver::rpi_one_shot_i2c(uint8_t address, uint8_t * bytes, uint8_t len, bool read)
{
  i2c_msg msg{};
  msg.addr = address;
  msg.flags = read ? I2C_M_RD : 0;
  msg.len = len;
  msg.buf = bytes;

  i2c_rdwr_ioctl_data payload{};
  payload.msgs = &msg;
  payload.nmsgs = 1;
  return ioctl(i2c_fd_, I2C_RDWR, &payload) >= 0;
}

bool GoPiGo3Driver::rpi_i2c(
  uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
  uint8_t read_len, unsigned settle_us)
{
  if (i2c_fd_ < 0 || (write_len == 0 && read_len == 0)) {
    return false;
  }

  // One transfer each, with a STOP in between, never a repeated start: the ATtiny on the
  // Dexter boards stretches the clock, which the Broadcom controller mishandles halfway
  // through a combined transaction. This is what di_i2c does for the "RPI_1" bus.
  if (write_len > 0) {
    if (!rpi_one_shot_i2c(address, const_cast<uint8_t *>(write_bytes), write_len, false)) {
      return false;
    }
  }
  if (read_len > 0) {
    if (write_len > 0) {
      usleep(settle_us);
    }
    return rpi_one_shot_i2c(address, read_bytes, read_len, true);
  }
  return true;
}

bool GoPiGo3Driver::sensor_i2c(
  GroveI2cPort port, uint8_t address, const uint8_t * write_bytes, uint8_t write_len,
  uint8_t * read_bytes, uint8_t read_len, unsigned settle_us)
{
  if (port == GroveI2cPort::Off) {
    return false;
  }
  if (port == GroveI2cPort::I2c) {
    return rpi_i2c(address, write_bytes, write_len, read_bytes, read_len, settle_us);
  }
  return grove_i2c(grove_id(port), address, write_bytes, write_len, read_bytes, read_len);
}

bool GoPiGo3Driver::detect_line_follower()
{
  uint8_t cmd = 0x12;
  uint8_t board[20]{};
  if (sensor_i2c(
        lf_port_, kLineFollowerAddress, &cmd, 1, board, sizeof(board),
        kLineFollowerSettleUs)) {
    if (bytes_to_string(board, sizeof(board)) == "Line Follower") {
      lf_kind_ = LineFollowerKind::Black;
      return true;
    }
  }

  LineFollowerReading probe;
  if (read_black(probe, kDefaultWhiteThreshold)) {
    lf_kind_ = LineFollowerKind::Black;
    return true;
  }
  if (read_red(probe, kDefaultWhiteThreshold)) {
    lf_kind_ = LineFollowerKind::Red;
    return true;
  }

  lf_kind_ = LineFollowerKind::None;
  return false;
}

bool GoPiGo3Driver::read_black(LineFollowerReading & reading, double white_threshold)
{
  uint8_t cmd = 0x01;
  uint8_t raw[8]{};
  if (!sensor_i2c(
        lf_port_, kLineFollowerAddress, &cmd, 1, raw, sizeof(raw), kLineFollowerSettleUs)) {
    return false;
  }

  reading.count = 6;
  for (int s = 0; s < 6; ++s) {
    const int extra = (raw[6 + (s / 4)] >> (2 * (s % 4))) & 0x03;
    const int value = (static_cast<int>(raw[s]) << 2) | extra;
    reading.sensors[static_cast<std::size_t>(5 - s)] = (1023 - value) / 1023.0;
  }
  estimate_line(reading, white_threshold);
  return true;
}

bool GoPiGo3Driver::read_red(LineFollowerReading & reading, double white_threshold)
{
  const uint8_t trigger[5] = {0x01, 0x03, 0x00, 0x00, 0x00};
  if (!sensor_i2c(lf_port_, kLineFollowerAddress, trigger, sizeof(trigger), nullptr, 0)) {
    return false;
  }
  usleep(10000);
  if (!sensor_i2c(lf_port_, kLineFollowerAddress, trigger, sizeof(trigger), nullptr, 0)) {
    return false;
  }

  uint8_t raw[10]{};
  if (!sensor_i2c(lf_port_, kLineFollowerAddress, nullptr, 0, raw, sizeof(raw))) {
    return false;
  }

  reading.count = 5;
  for (int s = 0; s < 5; ++s) {
    const int value = raw[2 * s] * 256 + raw[2 * s + 1];
    reading.sensors[static_cast<std::size_t>(4 - s)] = (1023 - value) / 1023.0;
  }
  estimate_line(reading, white_threshold);
  return true;
}

void GoPiGo3Driver::estimate_line(LineFollowerReading & reading, double white_threshold) const
{
  const auto n = reading.count;
  if (n == 0) {
    reading.state = "unknown";
    return;
  }

  const double threshold = std::clamp(white_threshold, 0.05, 1.0);
  double numerator = 0.0;
  double denominator = 0.0;
  int black_hits = 0;
  for (std::size_t i = 0; i < n; ++i) {
    // Measured against the threshold rather than against pure white, so the bare surface
    // contributes nothing and a line under one sensor pulls the estimate all the way there.
    const double blackness = std::clamp((threshold - reading.sensors[i]) / threshold, 0.0, 1.0);
    numerator += static_cast<double>(i) * blackness;
    denominator += blackness;
    if (blackness > 0.0) {
      ++black_hits;
    }
  }

  if (denominator > 0.0) {
    reading.position = numerator / (denominator * static_cast<double>(n - 1));
  } else {
    reading.position = 0.5;
  }

  if (black_hits == static_cast<int>(n)) {
    reading.lost = 1;
    reading.state = "black";
  } else if (black_hits == 0) {
    reading.lost = 2;
    reading.state = "white";
  } else if (reading.position >= 0.4 && reading.position <= 0.6) {
    reading.lost = 0;
    reading.state = "center";
  } else if (reading.position < 0.4) {
    reading.lost = 0;
    reading.state = "left";
  } else {
    reading.lost = 0;
    reading.state = "right";
  }
}

bool GoPiGo3Driver::tcs_write8(uint8_t reg, uint8_t value)
{
  const uint8_t payload[2] = {static_cast<uint8_t>(kTcsCommand | reg), value};
  return sensor_i2c(color_port_, kTcsAddress, payload, sizeof(payload), nullptr, 0);
}

bool GoPiGo3Driver::tcs_read(uint8_t reg, uint8_t * data, uint8_t len)
{
  const uint8_t auto_inc = (len > 1) ? 0x20 : 0x00;
  const uint8_t cmd = static_cast<uint8_t>(kTcsCommand | auto_inc | reg);
  return sensor_i2c(color_port_, kTcsAddress, &cmd, 1, data, len);
}

bool GoPiGo3Driver::tcs_set_led(bool on)
{
  if (!tcs_write8(kTcsPers, 0)) {
    return false;
  }
  uint8_t enable = 0;
  if (!tcs_read(kTcsEnable, &enable, 1)) {
    return false;
  }
  if (on) {
    enable = static_cast<uint8_t>(enable | kTcsEnableAien);
  } else {
    enable = static_cast<uint8_t>(enable & static_cast<uint8_t>(~kTcsEnableAien));
  }
  return tcs_write8(kTcsEnable, enable);
}

bool GoPiGo3Driver::read_color(ColorReading & reading)
{
  reading = {};
  if (color_port_ == GroveI2cPort::Off) {
    return false;
  }

  uint8_t raw[8]{};
  if (!tcs_read(kTcsCdata, raw, sizeof(raw))) {
    return false;
  }

  const auto channel = [](const uint8_t * bytes) {
    return static_cast<int>(bytes[0]) | (static_cast<int>(bytes[1]) << 8);
  };
  const double scale = 1.0 / static_cast<double>((256 - tcs_atime_) * 1024);
  reading.clear = std::clamp(channel(raw + 0) * scale, 0.0, 1.0);
  reading.red = std::clamp(channel(raw + 2) * scale, 0.0, 1.0);
  reading.green = std::clamp(channel(raw + 4) * scale, 0.0, 1.0);
  reading.blue = std::clamp(channel(raw + 6) * scale, 0.0, 1.0);
  guess_color(reading);
  return true;
}

void GoPiGo3Driver::guess_color(ColorReading & reading) const
{
  if (reading.clear < 1e-6) {
    reading.name = "black";
    return;
  }

  const double r = std::clamp(reading.red / reading.clear, 0.0, 1.0);
  const double g = std::clamp(reading.green / reading.clear, 0.0, 1.0);
  const double b = std::clamp(reading.blue / reading.clear, 0.0, 1.0);
  const double max_c = std::max({r, g, b});
  const double min_c = std::min({r, g, b});
  const double delta = max_c - min_c;
  double h = 0.0;
  double s = (max_c > 0.0) ? (delta / max_c) : 0.0;
  const double v = max_c;
  if (delta > 1e-6) {
    if (max_c == r) {
      h = (g - b) / delta;
    } else if (max_c == g) {
      h = 2.0 + (b - r) / delta;
    } else {
      h = 4.0 + (r - g) / delta;
    }
    h *= 60.0;
    if (h < 0.0) {
      h += 360.0;
    }
  }

  struct NamedHsv
  {
    const char * name;
    double h;
    double s;
    double v;
  };
  constexpr NamedHsv kKnown[] = {
    {"black", 0.0, 0.0, 0.0},     {"white", 0.0, 0.0, 1.0},   {"red", 0.0, 1.0, 1.0},
    {"green", 120.0, 1.0, 1.0},   {"blue", 240.0, 1.0, 1.0},  {"yellow", 60.0, 1.0, 1.0},
    {"cyan", 180.0, 1.0, 1.0},    {"fuchsia", 300.0, 1.0, 1.0}};

  double best = std::numeric_limits<double>::max();
  const char * name = "unknown";
  for (const auto & known : kKnown) {
    double dh = std::abs(h - known.h);
    if (dh > 180.0) {
      dh = 360.0 - dh;
    }
    const double ds = (s - known.s) * 100.0;
    const double dv = (v - known.v) * 100.0;
    const double dist = dh * dh + ds * ds + dv * dv;
    if (dist < best) {
      best = dist;
      name = known.name;
    }
  }
  reading.name = name;
}

void GoPiGo3Driver::set_eye_left(double red, double green, double blue)
{
  gpg_->set_led(LED_EYE_LEFT, to_pwm(red), to_pwm(green), to_pwm(blue));
}

void GoPiGo3Driver::set_eye_right(double red, double green, double blue)
{
  gpg_->set_led(LED_EYE_RIGHT, to_pwm(red), to_pwm(green), to_pwm(blue));
}

void GoPiGo3Driver::set_eyes(double red, double green, double blue)
{
  gpg_->set_led(
    LED_EYE_LEFT | LED_EYE_RIGHT, to_pwm(red), to_pwm(green), to_pwm(blue));
}

void GoPiGo3Driver::set_blinker_left(double brightness)
{
  gpg_->set_led(LED_BLINKER_LEFT, to_pwm(brightness));
}

void GoPiGo3Driver::set_blinker_right(double brightness)
{
  gpg_->set_led(LED_BLINKER_RIGHT, to_pwm(brightness));
}

void GoPiGo3Driver::set_blinkers(double brightness)
{
  gpg_->set_led(LED_BLINKER_LEFT | LED_BLINKER_RIGHT, to_pwm(brightness));
}

void GoPiGo3Driver::set_wheel_speeds(double left_dps, double right_dps)
{
  gpg_->set_motor_dps(MOTOR_LEFT, to_dps(left_dps));
  gpg_->set_motor_dps(MOTOR_RIGHT, to_dps(right_dps));
}

void GoPiGo3Driver::stop()
{
  gpg_->set_motor_dps(MOTOR_LEFT, 0);
  gpg_->set_motor_dps(MOTOR_RIGHT, 0);
}

double GoPiGo3Driver::wheel_radius() const
{
  return gpg_->WHEEL_DIAMETER / 2000.0;
}

double GoPiGo3Driver::wheel_separation() const
{
  return gpg_->WHEEL_BASE_WIDTH / 1000.0;
}
