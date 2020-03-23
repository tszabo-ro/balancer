
#include <Arduino.h>

#include "include/MPU6050Wrapper.h"
#include "include/Motor.h"
#include "include/primitives/ring_buffer.h"
#include "include/primitives/array.h"

#include "include/filters/savitzky_golay.h"

float getSupplyVoltage()
{
  int ref = analogRead(A0);
  constexpr float shunt_multiplier = 3.0;
  constexpr float ref_voltage = 5.0;
  return (static_cast<float>(ref)/1023.0) * shunt_multiplier * ref_voltage;
}


MPU6050Wrapper mpu(2);

Motor left_motor(6, 9, 254, 0);
Motor right_motor(10, 11, 254, 0);

void setup()
{
  Serial.begin(115200);
  mpu.initialize( -59,    // gyro_x_offset
                  9,      // gyro_y_offset
                  -30,    // gyro_z_offset
                  -284,   // acc_x_offset
                  -1123,  // acc_y_offset
                  927);   // acc_z_offset

  pinMode(A0, INPUT);
}

float angle_ref = 0; // At some point this is going to be the control variable for the speed!


constexpr unsigned int fast_loop_rate_ms = 5;
constexpr unsigned int slow_loop_rate_ms = 25;
constexpr unsigned int comm_loop_rate_ms = 100;

unsigned long last_fast_exec = 0;
unsigned long last_slow_exec = 0;
unsigned long last_comm_exec = 0;


SavitzkyGolayFilter<double, 5, 3, 0> imu_filter(0.01); // This is the rate with which the IMU provides data

void fastLoop(unsigned long T)
{
  if (T < last_fast_exec + fast_loop_rate_ms)
  {
    return;
  }
  last_fast_exec  = T;

  if (mpu.read())
  {
    imu_filter.push(angle_ref - mpu.getRoll());
  }
}

float kP = 1;
float kD = 1;

void slowLoop(unsigned long T)
{
  if (T < last_slow_exec + slow_loop_rate_ms)
  {
    return;
  }
  last_slow_exec  = T;


  double err = imu_filter.filter(0);
  double err_diff = imu_filter.filter(1);

  float cmd = (-1) * (kP * err + kD * err_diff);
  auto left_setspeed = left_motor.setSpeed(cmd);
  auto right_setspeed = right_motor.setSpeed(cmd);

  Serial.print(err * 10);
  Serial.print(" ");
  Serial.print(err_diff * 10);
  Serial.print(" ");
  Serial.print(cmd * 10);
  Serial.print(" ");
  Serial.print(static_cast<float>(left_setspeed)*10/255);
  Serial.print(" ");
  Serial.print(static_cast<float>(right_setspeed)*10/255);

  Serial.println("");
}

void commLoop(unsigned long T)
{
  if (T < last_comm_exec + comm_loop_rate_ms)
  {
    return;
  }
  last_comm_exec  = T;
}

void loop()
{
  unsigned long now = millis();
  fastLoop(now);
  slowLoop(now);
  commLoop(now);
}