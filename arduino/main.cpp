
#include <Arduino.h>
#include <EEPROM.h>

#include "include/MPU6050Wrapper.h"
#include "include/PWMDriver.h"
#include "include/Motor.h"

#include "include/primitives/ring_buffer.h"
#include "include/primitives/array.h"

#include "include/filters/savitzky_golay.h"
#include "include/filters/moving_average.h"

#include "include/Calibration.h"

#define MIN_ALLOWED_VOLTAGE_TO_DRIVE 11.3

#define LED_GREEN       5
#define LED_BLUE        6
#define LED_RED         7

#define BUTTON_INPUT_1  8
#define BUTTON_INPUT_2  9

#define ENCODER_L_A     10
#define ENCODER_L_B     11

#define ENCODER_R_A     12
#define ENCODER_R_B     13

constexpr unsigned int imu_read_rate_ms = 5;

constexpr unsigned int stabilizer_rate_ms = 25;
constexpr unsigned int velocity_rate_ms = 200;

constexpr unsigned int comm_loop_rate_ms = 100;
constexpr unsigned int heartbeat_loop_rate_ms = 500;

/////////////////////////////////////////////////////

MPU6050Wrapper mpu(0x68);

PWMDriver pwm_board;

Motor left_motor(pwm_board, 1, 0);
Motor right_motor(pwm_board, 3, 2);

SavitzkyGolayFilter<double, 2, 3, 0> imu_filter(1);
SavitzkyGolayFilter<double, 10, 3, 0> wheel_vel_filter(1);

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
  , allowed_to_move(false)
  , left_motor_speed(0)
  , right_motor_speed(0)
  , heartbeat_state(false)
  {
  }

  PIDState angle;
  PIDState wheel_vel;

  float ref_angle = 0; // At some point this is going to be the control variable for the speed!
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

  if (state.allowed_to_move)
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

  Serial.print(voltage);
  Serial.print(" ");
  Serial.print(params.inner.kP);
  Serial.print(" ");
  Serial.print(params.inner.kD);
  Serial.print(" ");
  Serial.print(params.inner.kI);
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
  Serial.print(" ");
  Serial.print(params.outer.kP);
  Serial.print(" ");
  Serial.print(params.outer.kD);
  Serial.print(" ");
  Serial.print(params.outer.kI);
  Serial.print(" ");
  Serial.print(state.wheel_vel.error * 10);
  Serial.print(" ");
  Serial.print(state.wheel_vel.d_error * 10);
  Serial.print(" ");
  Serial.print(state.wheel_vel.i_error * 10);
  Serial.print(" ");
  Serial.print(state.ref_angle * 10);
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
    imu_filter.push(state.ref_angle -mpu.getRoll());
  }
}

unsigned long last_velocity_exec = 0;

void  velocityLoop(unsigned long T, CurrentState &state, const Params& params)
{
  if (T < last_velocity_exec + velocity_rate_ms)
  {
    return;
  }
  last_velocity_exec = T;

  state.wheel_vel.error = wheel_vel_filter.filter(0);
  state.wheel_vel.d_error = wheel_vel_filter.filter(1);

  constexpr float max_tilt_angle = 4 * M_PI/180;

  if (((state.wheel_vel.i_error > 0) && (state.wheel_vel.error < 0)) || ((state.wheel_vel.i_error < 0) && (state.wheel_vel.error > 0)) || (fabs(state.wheel_vel.i_error * params.outer.kI) < max_tilt_angle))
  {
    state.wheel_vel.i_error += state.wheel_vel.error * velocity_rate_ms / 1000.0;
  }

  float ref_angle = (-1) * (params.outer.kP * state.wheel_vel.error + params.outer.kD * state.wheel_vel.d_error + params.outer.kI * state.wheel_vel.i_error);

  if (fabs(ref_angle) < max_tilt_angle)
  {
    state.ref_angle = ref_angle;
  }
  else
  {
    state.ref_angle = (ref_angle > 0) ? max_tilt_angle : -max_tilt_angle;
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

  state.angle.error = imu_filter.filter(0);
  state.angle.d_error = imu_filter.filter(1);

  if (((state.angle.i_error > 0) && (state.angle.error < 0)) || ((state.angle.i_error < 0) && (state.angle.error > 0)) || (fabs(state.angle.i_error * params.inner.kI) < 255.0))
  {
    state.angle.i_error += state.angle.error*stabilizer_rate_ms / 1000.0;
  }

  float vel = (-4095.0/255.0) * (params.inner.kP * state.angle.error + params.inner.kD * state.angle.d_error + params.inner.kI * state.angle.i_error);

  state.cmd = constrain(vel, -4095.0, 4095.0);

  // Filter the command vel so that the required tilt angle can be set.
  wheel_vel_filter.push(state.cmd/4095);

  if (!state.allowed_to_move)
  {
    state.left_motor_speed = left_motor.setSpeed(0);
    state.right_motor_speed = right_motor.setSpeed(0);
  }
  else
  {
    state.left_motor_speed = left_motor.setSpeed(round(state.cmd));
    state.right_motor_speed = right_motor.setSpeed(round(state.cmd));
  }
}


void readParams(CurrentState& state, Params& params)
{
    String indicator_str = Serial.readStringUntil('/');

    if (indicator_str.length() != 1)
    {
      return;
    }

    PIDParams *controller_params = &params.inner;
    PIDState *controller_state = &state.angle;

    if (indicator_str == "o")
    {
      controller_params = &params.outer;
      controller_state = &state.wheel_vel;
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
      controller_state->i_error = 0;
    }
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
    readParams(state, params);

    while (Serial.available())
    {
      Serial.read();
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
// Interrupt handling

void (*interrupt_fcns[8])() = {nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

uint8_t last_PINB;
ISR (PCINT0_vect)
{
  uint8_t change_B = last_PINB ^ PINB;

  for (uint8_t b = 0; b < 6; ++b)
  {
    if ( (interrupt_fcns[b] != nullptr) && (change_B & bit(b)) && (PINB & bit(b)) )
    {
      interrupt_fcns[b]();
    }
  }
  last_PINB = PINB;
}


/////////////////////////////////////////////////////

CurrentState stateStore;
StoredData eeprom_data;

Params& paramStore = eeprom_data.params;

unsigned long button_1_time = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(2);

  DeviceCalibration& calibration = eeprom_data.calib;

  pwm_board.begin();
  pwm_board.setPWMFreq(1000);

  for (uint8_t pin = 0; pin < 16; ++pin)
  {
    pwm_board.setPin(pin, 0);
  }

  mpu.initialize(calibration.gyro);

  left_motor.initialize(calibration.l_motor);
  right_motor.initialize(calibration.r_motor);

  pinMode(A0, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(LED_GREEN, OUTPUT);       // LED: Green
  pinMode(LED_BLUE, OUTPUT);       // LED: Blue
  pinMode(LED_RED, OUTPUT);       // LED: Red

  pinMode(BUTTON_INPUT_1, INPUT_PULLUP);  // Button 1
  pinMode(BUTTON_INPUT_2, INPUT_PULLUP);  // Button 2

  if (!digitalRead(BUTTON_INPUT_1))
  {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);
    calibration.read();
  }

  interrupt_fcns[0] = []() {
      unsigned long now = micros();
      if (now < (button_1_time + 500000))
      {
        return;
      }
      button_1_time = now;

      stateStore.allowed_to_move = !stateStore.allowed_to_move;
      if (stateStore.allowed_to_move)
      {
        stateStore.angle.i_error = 0;
        stateStore.wheel_vel.i_error = 0;
      }
    };

  unsigned long T = millis();
  uint16_t period = 300;
  pwm_board.setPin(4, 2095);
  while (millis() < (T + 2000))
  {
    auto ref = (millis() - T) % period;
    if (ref < period/3)
    {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, HIGH);
    }
    else if (ref < 2*period/3)
    {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_BLUE, HIGH);
      digitalWrite(LED_GREEN, LOW);
    }
    else
    {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_GREEN, LOW);
    }
  }
  pwm_board.setPin(4, 0);


  PCICR  |= bit(PCIE0);    // enable pin change interrupts for D8 to D13
  PCIFR  &= ~bit(PCIF0);    // clear any outstanding interrupts
  PCMSK0 = 0xFF;

  stateStore.allowed_to_move = false;
 }

void loop()
{
  unsigned long now = millis();
  imuReadLoop(now, stateStore, paramStore);

  stabilizerLoop(now, stateStore, paramStore);
  velocityLoop(now, stateStore, paramStore);

  commLoop(now, stateStore, paramStore);
  heartbeatLoop(now, stateStore, paramStore);
}
