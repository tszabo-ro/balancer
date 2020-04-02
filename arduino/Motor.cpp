#include <Arduino.h>
#include "include/Motor.h"

#include "include/Calibration.h"

#include "include/PWMDriver.h"

Motor::Motor(PWMDriver& pwm, int fwd_pin, int bck_pin)
: fwd_pin_(fwd_pin)
, bck_pin_(bck_pin)
, min_out_(0)
, max_out_(255)
, pwm_(pwm)
{
}

int Motor::initialize(const MotorCalib& calib_data)
{
  max_out_ = calib_data.max_output;
  min_out_ = calib_data.min_output;

  return 0;
}

int16_t Motor::setSpeed(int16_t vel)
{
  int drive_pin = fwd_pin_;
  int off_pin = bck_pin_;
  if (vel < 0)
  {
    drive_pin = bck_pin_;
    off_pin = fwd_pin_;
  }

  vel = constrain(abs(vel), 0, max_out_);
  uint16_t send_vel = static_cast<uint16_t>(vel);

  pwm_.setPin(drive_pin, vel);
  pwm_.setPin(off_pin, 0);

  return send_vel * ((drive_pin == fwd_pin_) ? 1 : -1);
}
