#include "include/control_loops.h"

#include "include/robot.h"
#include "include/Calibration.h"

#define STABILIZER_RATE_HZ 50

namespace
{
 float angle_i_error = 0;
}


Task stabilizer(STABILIZER_RATE_HZ, [](){
  const Params& params = EEPROM_DATA.params;


  float angle_error = ROBOT.imu_filter.filter(0);
  float angle_d_error = ROBOT.imu_filter.filter(1);

  if (((angle_i_error > 0) && (angle_error < 0)) || ((angle_i_error < 0) && (angle_error > 0)) || (fabs(angle_i_error * params.inner.kI) < 255.0))
  {
    angle_i_error += angle_error * (1.0 / STABILIZER_RATE_HZ);
  }

  float vel = (-1) * (params.inner.kP * angle_error + params.inner.kD * angle_d_error + params.inner.kI * angle_i_error);

  float cmd = constrain(vel, -220, 220);

  // Filter the command vel so that the required tilt angle can be set.
  ROBOT.wheel_vel_filter.push(cmd);

  ROBOT.left_motor.setSpeed(round(cmd));
  ROBOT.right_motor.setSpeed(round(cmd));
}, []() { angle_i_error = 0; });
