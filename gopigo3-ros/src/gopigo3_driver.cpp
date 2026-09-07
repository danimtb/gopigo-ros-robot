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

int16_t to_dps(double value)
{
  constexpr double kMax = std::numeric_limits<int16_t>::max();
  return static_cast<int16_t>(std::lround(std::clamp(value, -kMax, kMax)));
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

bool GoPiGo3Driver::init_line_follower(LineFollowerPort port)
{
  lf_port_ = port;
  lf_kind_ = LineFollowerKind::None;
  grove_port_ = 0;
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }

  if (port == LineFollowerPort::Off) {
    return false;
  }

  if (port == LineFollowerPort::I2c) {
    i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
      return false;
    }
  } else {
    grove_port_ = (port == LineFollowerPort::Ad1) ? GROVE_1 : GROVE_2;
    gpg_->set_grove_type(grove_port_, GROVE_TYPE_I2C);
    usleep(10000);
  }

  return detect_line_follower();
}

bool GoPiGo3Driver::read_line_follower(LineFollowerReading & reading)
{
  reading = {};
  if (lf_kind_ == LineFollowerKind::Black) {
    return read_black(reading);
  }
  if (lf_kind_ == LineFollowerKind::Red) {
    return read_red(reading);
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

bool GoPiGo3Driver::rpi_i2c(
  uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
  uint8_t read_len)
{
  i2c_msg msgs[2]{};
  unsigned nmsgs = 0;
  if (write_len > 0) {
    msgs[nmsgs].addr = address;
    msgs[nmsgs].flags = 0;
    msgs[nmsgs].len = write_len;
    msgs[nmsgs].buf = const_cast<uint8_t *>(write_bytes);
    ++nmsgs;
  }
  if (read_len > 0) {
    msgs[nmsgs].addr = address;
    msgs[nmsgs].flags = I2C_M_RD;
    msgs[nmsgs].len = read_len;
    msgs[nmsgs].buf = read_bytes;
    ++nmsgs;
  }
  if (nmsgs == 0) {
    return false;
  }

  i2c_rdwr_ioctl_data payload{};
  payload.msgs = msgs;
  payload.nmsgs = nmsgs;
  return ioctl(i2c_fd_, I2C_RDWR, &payload) >= 0;
}

bool GoPiGo3Driver::sensor_i2c(
  uint8_t address, const uint8_t * write_bytes, uint8_t write_len, uint8_t * read_bytes,
  uint8_t read_len)
{
  if (lf_port_ == LineFollowerPort::I2c) {
    return rpi_i2c(address, write_bytes, write_len, read_bytes, read_len);
  }
  return grove_i2c(grove_port_, address, write_bytes, write_len, read_bytes, read_len);
}

bool GoPiGo3Driver::detect_line_follower()
{
  uint8_t cmd = 0x12;
  uint8_t board[20]{};
  if (sensor_i2c(kLineFollowerAddress, &cmd, 1, board, sizeof(board))) {
    if (bytes_to_string(board, sizeof(board)) == "Line Follower") {
      lf_kind_ = LineFollowerKind::Black;
      return true;
    }
  }

  LineFollowerReading probe;
  if (read_black(probe)) {
    lf_kind_ = LineFollowerKind::Black;
    return true;
  }
  if (read_red(probe)) {
    lf_kind_ = LineFollowerKind::Red;
    return true;
  }

  lf_kind_ = LineFollowerKind::None;
  return false;
}

bool GoPiGo3Driver::read_black(LineFollowerReading & reading)
{
  uint8_t cmd = 0x01;
  uint8_t raw[8]{};
  if (!sensor_i2c(kLineFollowerAddress, &cmd, 1, raw, sizeof(raw))) {
    return false;
  }

  reading.count = 6;
  for (int s = 0; s < 6; ++s) {
    const int extra = (raw[6 + (s / 4)] >> (2 * (s % 4))) & 0x03;
    const int value = (static_cast<int>(raw[s]) << 2) | extra;
    reading.sensors[static_cast<std::size_t>(5 - s)] = (1023 - value) / 1023.0;
  }
  estimate_line(reading);
  return true;
}

bool GoPiGo3Driver::read_red(LineFollowerReading & reading)
{
  const uint8_t trigger[5] = {0x01, 0x03, 0x00, 0x00, 0x00};
  if (!sensor_i2c(kLineFollowerAddress, trigger, sizeof(trigger), nullptr, 0)) {
    return false;
  }
  usleep(10000);
  if (!sensor_i2c(kLineFollowerAddress, trigger, sizeof(trigger), nullptr, 0)) {
    return false;
  }

  uint8_t raw[10]{};
  if (!sensor_i2c(kLineFollowerAddress, nullptr, 0, raw, sizeof(raw))) {
    return false;
  }

  reading.count = 5;
  for (int s = 0; s < 5; ++s) {
    const int value = raw[2 * s] * 256 + raw[2 * s + 1];
    reading.sensors[static_cast<std::size_t>(4 - s)] = (1023 - value) / 1023.0;
  }
  estimate_line(reading);
  return true;
}

void GoPiGo3Driver::estimate_line(LineFollowerReading & reading) const
{
  const auto n = reading.count;
  if (n == 0) {
    reading.state = "unknown";
    return;
  }

  double numerator = 0.0;
  double denominator = 0.0;
  int black_hits = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double blackness = std::clamp(1.0 - reading.sensors[i], 0.0, 1.0);
    numerator += static_cast<double>(i) * blackness;
    denominator += blackness;
    if (blackness > 0.5) {
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
