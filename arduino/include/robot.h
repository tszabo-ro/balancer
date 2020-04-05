#ifndef robot_hardware_h
#define robot_hardware_h

#include "MPU6050Wrapper.h"
#include "PWMDriver.h"
#include "Motor.h"

#include "filters/savitzky_golay.h"

#define MIN_ALLOWED_VOLTAGE_TO_DRIVE 11.3

#define LED_GREEN       5
#define LED_BLUE        6
#define LED_RED         7

#define BUTTON_INPUT_1  8
#define BUTTON_INPUT_2  9

#define ENCODER_L_A     10
#define ENCODER_L_B     11

#define ENCODER_R_A     12
#define ENCODER_R_B     1

struct State
{
  State()
  : angle_reference(-0.025)
  , heartbeat(0)
  {
  }

  float angle_reference;
  bool heartbeat;
};

struct Robot
{
  Robot()
  : mpu(0x68)
  , pwm_board()
  , right_motor(pwm_board, 1, 0)
  , left_motor(pwm_board, 3, 2)
  , imu_filter(1)
  , wheel_vel_filter(1)
  , allowed_to_move(false)
  {}

  MPU6050Wrapper mpu;
  PWMDriver pwm_board;

  Motor right_motor;
  Motor left_motor;

  SavitzkyGolayFilter<double, 2, 3, 0> imu_filter;
  SavitzkyGolayFilter<double, 10, 3, 0> wheel_vel_filter;

  bool allowed_to_move;

  State state;
};

extern Robot ROBOT;
#endif
