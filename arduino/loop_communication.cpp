#include "include/control_loops.h"

#include "include/robot.h"
#include "include/Calibration.h"

#define VOLTAGE_READ_PIN    A0
#define SHUNT_MULTIPLIER    3.0
#define REFERENCE_VOLTAGE   5.02


static float getSupplyVoltage()
{
  int ref = analogRead(VOLTAGE_READ_PIN);
  return (static_cast<float>(ref)/1023.0) * SHUNT_MULTIPLIER * REFERENCE_VOLTAGE;
}

void sendFeedback(const Params& params)
{
  float voltage = getSupplyVoltage();

  ROBOT.allowed_to_move = ROBOT.allowed_to_move && (voltage > MIN_ALLOWED_VOLTAGE_TO_DRIVE);
  ROBOT.right_motor.disable(!ROBOT.allowed_to_move);
  ROBOT.left_motor.disable(!ROBOT.allowed_to_move);

  if (ROBOT.allowed_to_move)
  {
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
  else
  {
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
  }

  float angle_error = ROBOT.imu_filter.filter(0);
  float angle_d_error = ROBOT.imu_filter.filter(1);

  float vel_error = ROBOT.wheel_vel_filter.filter(0);
  float vel_d_error = ROBOT.wheel_vel_filter.filter(1);

  Serial.print(voltage);
  Serial.print(" ");
  Serial.print(params.inner.kP);
  Serial.print(" ");
  Serial.print(params.inner.kD);
  Serial.print(" ");
  Serial.print(params.inner.kI);
  Serial.print(" ");
  Serial.print(params.outer.kP);
  Serial.print(" ");
  Serial.print(params.outer.kD);
  Serial.print(" ");
  Serial.print(params.outer.kI);
  Serial.print(" ");
  Serial.print(ROBOT.state.angle_reference * (180/M_PI) * 10);
  Serial.print(" ");
  Serial.print(angle_error * (180/M_PI));
  Serial.print(" ");
  Serial.print(angle_d_error * (180/M_PI));
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(vel_error * 10);
  Serial.print(" ");
  Serial.print(vel_d_error * 10);
  Serial.print(" ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(ROBOT.left_motor.setSpeed());
  Serial.print(" ");
  Serial.print(ROBOT.left_motor.getSpeed());
  Serial.print(" ");
  Serial.print(ROBOT.right_motor.getSpeed());
  Serial.println("");
}

void readParams(Params & params)
{
    String indicator_str = Serial.readStringUntil('/');

    if (indicator_str.length() != 1)
    {
      return;
    }

    PIDParams *controller_params = &params.inner;
    Task* controller_task = &stabilizer;

    if (indicator_str == "o")
    {
      controller_params = &params.outer;
      controller_task = &vel_loop;
    }
    else if (indicator_str != "i")
    {
      return;
    }

    String kp_str = Serial.readStringUntil('/');
    String kd_str = Serial.readStringUntil('/');
    String ki_str = Serial.readStringUntil('/');

    if ( (kp_str.length() == 0) || (kd_str.length() == 0) || (ki_str.length() == 0) )
    {
      return;
    }

    float kp = kp_str.toFloat();
    float kd = kd_str.toFloat();
    float ki = ki_str.toFloat();

    if ((kp >= 0) && (kd >= 0) && (ki >= 0))
    {
      controller_params->kP = kp;
      controller_params->kD = kd;
      controller_params->kI = ki;

      controller_params->store();
      controller_task->reset();
    }
}

Task communication(10, []() {
  Params& params = EEPROM_DATA.params;
  sendFeedback(params);

  if (Serial.available() > 6)
  {
    readParams(params);

    while (Serial.available())
    {
      Serial.read();
    }
  }
});