
#include <Arduino.h>
#include <EEPROM.h>

#include "include/MPU6050Wrapper.h"
#include "include/Motor.h"
#include "include/primitives/ring_buffer.h"
#include "include/primitives/array.h"

#include "include/filters/savitzky_golay.h"
#include "include/filters/moving_average.h"

#include "include/Calibration.h"

#define MIN_ALLOWED_VOLTAGE_TO_DRIVE 11.3

constexpr unsigned int imu_read_rate_ms = 5;
constexpr unsigned int stabilizer_rate_ms = 25;
constexpr unsigned int comm_loop_rate_ms = 100;
constexpr unsigned int heartbeat_loop_rate_ms = 500;

/////////////////////////////////////////////////////

MPU6050Wrapper mpu(2);
Motor left_motor(6, 9);
Motor right_motor(10, 11);

SavitzkyGolayFilter<double, 3, 3, 0> imu_filter(1);
//float Z0 = 0;
//float Z1 = 0;

MovingAverage<20, float> wheel_vel_filter(0);

/////////////////////////////////////////////////////

struct PIDState
{
  PIDState()
  : error(0)
  , d_error(0)
  , i_error(0)
  {}

  float error;
  float d_error;
  float i_error;
};

struct CurrentState
{
  CurrentState()
  : ref_angle(-0.020)
  , cmd(0)
  , allowed_to_move(true)
  , left_motor_speed(0)
  , right_motor_speed(0)
  , heartbeat_state(false)
  {
  }

  float ref_angle = 0; // At some point this is going to be the control variable for the speed!

  PIDState angle;
  PIDState wheel_vel;

  float cmd;

  bool allowed_to_move;
  int16_t left_motor_speed;
  int16_t right_motor_speed;

  bool heartbeat_state;
};


/////////////////////////////////////////////////////

float getSupplyVoltage()
{
  int ref = analogRead(A0);
  constexpr float shunt_multiplier = 3.0;
  constexpr float ref_voltage = 5.0;
  return (static_cast<float>(ref)/1023.0) * shunt_multiplier * ref_voltage;
}

void sendFeedback(CurrentState& state, const Params& params)
{
  float voltage=getSupplyVoltage();

  state.allowed_to_move = state.allowed_to_move && (voltage > MIN_ALLOWED_VOLTAGE_TO_DRIVE);

  Serial.print(voltage);
  Serial.print(" ");
  Serial.print(params.kP);
  Serial.print(" ");
  Serial.print(params.kD);
  Serial.print(" ");
  Serial.print(params.kI);
  Serial.print(" ");
  Serial.print(state.angle.error * 10);
  Serial.print(" ");
  Serial.print(state.angle.d_error * 10);
  Serial.print(" ");
  Serial.print(state.angle.i_error * 10);
  Serial.print(" ");
  Serial.print(state.cmd * 10);
  Serial.print(" ");
  Serial.print(state.left_motor_speed);
  Serial.print(" ");
  Serial.print(state.right_motor_speed);
  Serial.println("");
}

unsigned long last_imu_read = 0;

void imuReadLoop(unsigned long T, const CurrentState& state, const Params& params)
{
  (void)params; // Not used here

  if (T < last_imu_read + imu_read_rate_ms)
  {
    return;
  }
  last_imu_read  = T;

  if (mpu.read())
  {
	 // Z1 = Z0;
	 // Z0 = state.ref_angle -mpu.getRoll();

    imu_filter.push(state.ref_angle -mpu.getRoll());
  }
}

unsigned long last_stabilizer_exec = 0;

void stabilizerLoop(unsigned long T, CurrentState& state, const Params& params)
{
  if (T < last_stabilizer_exec + stabilizer_rate_ms)
  {
    return;
  }
  last_stabilizer_exec  = T;

  if (!state.allowed_to_move)
  {
    state.angle.i_error = 0;

    state.left_motor_speed = left_motor.setSpeed(0);
    state.right_motor_speed = right_motor.setSpeed(0);
    return;
  }

  //state.angle.error = Z0;
  //state.angle.d_error = Z0 - Z1;

  state.angle.error = imu_filter.filter(0);
  state.angle.d_error = imu_filter.filter(1);

  if (((state.angle.i_error > 0) && (state.angle.error < 0)) || ((state.angle.i_error < 0) && (state.angle.error > 0)) || (fabs(state.angle.i_error * params.kI) < 255.0))
  {
    state.angle.i_error += state.angle.error*stabilizer_rate_ms / 1000.0;
  }

  state.cmd = (-1) * (params.kP * state.angle.error + params.kD * state.angle.d_error + params.kI * state.angle.i_error);

  // Filter the command vel so that the required tilt angle can be set.
  wheel_vel_filter.push(state.cmd);

  state.left_motor_speed = left_motor.setSpeed(round(state.cmd));
  state.right_motor_speed = right_motor.setSpeed(round(state.cmd));
}

unsigned long last_comm_exec = 0;

void commLoop(unsigned long T, CurrentState& state, Params& params)
{
  if (T < last_comm_exec + comm_loop_rate_ms)
  {
    return;
  }
  last_comm_exec  = T;

  sendFeedback(state, params);

  if (Serial.available() > 6)
  {
    float kp = -1;
    float kd = -1;
    float ki = -1;

    String kp_str = Serial.readStringUntil('/');
    Serial.println(kp_str.length());
    if (kp_str.length() != 0) // The terminating \n is one char!
    {
      kp = kp_str.toFloat();

      String kd_str = Serial.readStringUntil('/');
      Serial.println(kd_str.length());
      if (kd_str.length() != 0)
      {
        kd = kd_str.toFloat();

        String ki_str = Serial.readStringUntil('/');
        Serial.println(ki_str.length());
        if (ki_str.length() != 0)
        {
          ki = ki_str.toFloat();
        }
        else
        {
          kp = -1; kd = -1; ki = -1;
        }
      }
      else
      {
        kp = -1; kd = -1; ki = -1;
      }
    }
    else
    {
      kp = -1; kd = -1; ki = -1;
    }

    while (Serial.available())
    {
      Serial.read();
    }

    if ((kp >= 0) && (kd >= 0) && (ki >= 0))
    {
      params.kP = kp;
      params.kD = kd;
      params.kI = ki;

      params.store();
      state.angle.i_error = 0;
    }
  }

}

unsigned long last_heartbeat_exec = 0;

void heartbeatLoop(unsigned long T, CurrentState& state, Params& params)
{
  if (T < last_heartbeat_exec + heartbeat_loop_rate_ms)
  {
    return;
  }
  last_heartbeat_exec  = T;

  (void)params; // Not used here

  state.heartbeat_state = !state.heartbeat_state;
  digitalWrite(LED_BUILTIN, state.heartbeat_state);
}

/////////////////////////////////////////////////////

CurrentState stateStore;
StoredData eeprom_data;

Params& paramStore = eeprom_data.params;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(2);

  DeviceCalibration& calibration = eeprom_data.calib;

  mpu.initialize(calibration.gyro);

  left_motor.initialize(calibration.l_motor);
  right_motor.initialize(calibration.r_motor);

  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  unsigned long now = millis();
  imuReadLoop(now, stateStore, paramStore);
  stabilizerLoop(now, stateStore, paramStore);
  commLoop(now, stateStore, paramStore);
  heartbeatLoop(now, stateStore, paramStore);
}
