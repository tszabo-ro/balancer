
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
  Serial.setTimeout(2);
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
double error_integral = 0;

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

float kP = 5;
float kD = 0.25;
float kI = 0;

void slowLoop(unsigned long T)
{
  if (T < last_slow_exec + slow_loop_rate_ms)
  {
    return;
  }
  last_slow_exec  = T;

  double err = imu_filter.filter(0);
  double err_diff = imu_filter.filter(1);

  if (((error_integral > 0) && (err < 0)) || ((error_integral < 0) && (err > 0)) || (fabs(error_integral * kI) < 255.0))
  {
    double err_delta = err*slow_loop_rate_ms/1000;
    error_integral += err_delta;
  }

  float cmd = (-1) * (kP * err + kD * err_diff + kI * error_integral);
  auto left_setspeed = left_motor.setSpeed(round(cmd));
  auto right_setspeed = right_motor.setSpeed(round(cmd));

  Serial.print(kP);
  Serial.print(" ");
  Serial.print(kD);
  Serial.print(" ");
  Serial.print(kI * 10);
  Serial.print(" ");
  Serial.print(err * 10);
  Serial.print(" ");
  Serial.print(err_diff * 10);
  Serial.print(" ");
  Serial.print(error_integral * 10);
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
      kP = kp;
      kD = kd;
      kI = ki;
      error_integral = 0;
    }
  }
}

void loop()
{
  unsigned long now = millis();
  fastLoop(now);
  slowLoop(now);
  commLoop(now);
}