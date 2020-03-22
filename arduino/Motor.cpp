#include "include/Motor.h"

Motor::Motor(int fwd_pin, int bck_pin, int max_output, int min_output)
: fwd_pin_(fwd_pin)
, bck_pin_(bck_pin)
, max_out_(max_output)
, min_out_(min_output)
{
}

void Motor::setSpeed(double vel)
{
  int drive_pin = fwd_pin_;
  int off_pin = bck_pin_;
  if (vel < 0)
  {
    drive_pin = bck_pin_;
    off_pin = fwd_pin_;
  }

  vel = constrain(round(fabs(vel) * 255), 0, max_out_);

  if (vel < min_out_)
  {
      vel = 0;
  }

  uint8_t send_vel = static_cast<uint8_t>(vel);
  Serial.print(send_vel * ((drive_pin == fwd_pin_) ? 1 : -1));
  Serial.print(" ");

  analogWrite(drive_pin, send_vel);
  analogWrite(off_pin, 0);
}
