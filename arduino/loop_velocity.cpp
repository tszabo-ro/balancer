#include "include/control_loops.h"
#include "include/robot.h"
#include "include/Calibration.h"

#define VELOCITY_LOOP_RATE_HZ 5


namespace
{
 float vel_i_error = 0;
}


Task vel_loop(VELOCITY_LOOP_RATE_HZ, []() {
  const Params& params = EEPROM_DATA.params;

  float vel_error = ROBOT.wheel_vel_filter.filter(0);
  float vel_d_error = ROBOT.wheel_vel_filter.filter(1);

  constexpr float max_tilt_angle = 4 * M_PI/180;

  if (((vel_i_error > 0) && (vel_error < 0)) || ((vel_i_error < 0) && (vel_error > 0)) || (fabs(vel_i_error * params.outer.kI) < max_tilt_angle))
  {
    vel_i_error += vel_error * (1.0 / VELOCITY_LOOP_RATE_HZ);
  }

  /*float ref_angle = (-1) * (params.outer.kP * state.wheel_vel.error + params.outer.kD * state.wheel_vel.d_error + params.outer.kI * state.wheel_vel.i_error);

  if (fabs(ref_angle) < max_tilt_angle)
  {
    state.ref_angle = ref_angle;
  }
  else
  {
    state.ref_angle = (ref_angle > 0) ? max_tilt_angle : -max_tilt_angle;
  }
*/
}, [](){ vel_i_error = 0;});