#include <Arduino.h>
#include "include/Motor.h"

#include "include/Calibration.h"

#include "include/PWMDriver.h"

#define MOTOR_CONTROL_RATE_MS 50
#define PULSE_PER_REV         224
#define MOTOR_RPM             220
#define P_gain                30.0f
#define I_gain                10.0f


Motor::Motor(PWMDriver& pwm, int fwd_pin, int bck_pin)
: fwd_pin_(fwd_pin)
, bck_pin_(bck_pin)
, disabled_(true)
, min_out_(0)
, max_out_(4095)
, pwm_(pwm)
, last_cycle_time_(0)
, tick_count_(0)
, speed_target_(0)
, speed_error_integral_(0)
{
}

int Motor::initialize(const MotorCalib& calib_data)
{
  max_out_ = calib_data.max_output;
  min_out_ = calib_data.min_output;

  return 0;
}

void Motor::setSpeed(float vel_rpm)
{
  speed_target_ = vel_rpm;
}

void Motor::encoderTick(bool A, bool B)
{
  if (A && !B)
  {
    ++tick_count_;
  }
  else if (!A && B)
  {
    --tick_count_;
  }
}

void Motor::controlCycle()
{
  auto now = millis();
  if (now < last_cycle_time_ + MOTOR_CONTROL_RATE_MS)
  {
    return;
  }
  last_cycle_time_ = now;

  if (disabled_)
  {
    speed_target_ = 0;
    speed_error_integral_ = 0;
  }

  current_speed_ = static_cast<float>(tick_count_ * 60'000)  / (MOTOR_CONTROL_RATE_MS * PULSE_PER_REV); // [RPM]
  tick_count_ = 0;

  float speed_error = speed_target_ - current_speed_;

  int16_t speed_cmd = speed_error * P_gain + speed_error_integral_ * I_gain;

  if ((speed_error_integral_* speed_error < 0) || (fabs(speed_cmd) < max_out_))
  {
    speed_error_integral_ += speed_error;
  }

  if (fabs(speed_cmd) < min_out_)
  {
    speed_cmd = 0;
  }

  int drive_pin = fwd_pin_;
  int off_pin = bck_pin_;
  if (speed_cmd < 0)
  {
    drive_pin = bck_pin_;
    off_pin = fwd_pin_;
  }

  uint16_t send_vel = static_cast<uint16_t>(constrain(abs(speed_cmd), 0, max_out_));

  pwm_.setPin(drive_pin, send_vel);
  pwm_.setPin(off_pin, 0);
}
