#include "gopigo3_driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <GoPiGo3.h>

namespace
{
int16_t to_dps(double value)
{
  constexpr double kMax = std::numeric_limits<int16_t>::max();
  return static_cast<int16_t>(std::lround(std::clamp(value, -kMax, kMax)));
}
}  // namespace

GoPiGo3Driver::GoPiGo3Driver() = default;

GoPiGo3Driver::~GoPiGo3Driver()
{
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
