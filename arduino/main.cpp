
#include <Arduino.h>

#include "include/robot.h"
#include "include/control_loops.h"

#include "include/Calibration.h"

/////////////////////////////////////////////////////
// ROBOT is the sole state carrier, within this application

Robot ROBOT;

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

unsigned long button_1_time = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(2);

  DeviceCalibration& calibration = EEPROM_DATA.calib;

  ROBOT.pwm_board.begin();
  ROBOT.pwm_board.setPWMFreq(1000);

  for (uint8_t pin = 0; pin < 16; ++pin)
  {
    ROBOT.pwm_board.setPin(pin, 0);
  }

  ROBOT.mpu.initialize(calibration.gyro);

  ROBOT.left_motor.initialize(calibration.l_motor);
  ROBOT.right_motor.initialize(calibration.r_motor);

  pinMode(A0, INPUT);

  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);
  pinMode(13, INPUT);

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

      ROBOT.allowed_to_move = !ROBOT.allowed_to_move;
      if (ROBOT.allowed_to_move)
      {
        stabilizer.reset();
        vel_loop.reset();
      }
    };

  interrupt_fcns[2] = []() { ROBOT.right_motor.encoderTick(PINB & bit(3), PINB & bit(2)); };
  interrupt_fcns[3] = []() { ROBOT.right_motor.encoderTick(PINB & bit(3), PINB & bit(2)); };

  interrupt_fcns[4] = []() { ROBOT.left_motor.encoderTick(PINB & bit(4), PINB & bit(5)); };
  interrupt_fcns[5] = []() { ROBOT.left_motor.encoderTick(PINB & bit(4), PINB & bit(5)); };

  unsigned long T = millis();
  uint16_t period = 300;
  ROBOT.pwm_board.setPin(4, 2095);
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
  ROBOT.pwm_board.setPin(4, 0);

  PCICR  |= bit(PCIE0);    // enable pin change interrupts for D8 to D13
  PCIFR  &= ~bit(PCIF0);    // clear any outstanding interrupts
  PCMSK0 = 0xFF;

  ROBOT.allowed_to_move = false;
 }

Task read_imu(200, []() {
  if (ROBOT.mpu.read())
  {
    ROBOT.imu_filter.push(ROBOT.state.angle_reference - ROBOT.mpu.getRoll());
  }
});

Task heartbeat(2, [](){ ROBOT.state.heartbeat = !ROBOT.state.heartbeat; });

void loop()
{
  ROBOT.right_motor.controlCycle();
  ROBOT.left_motor.controlCycle();

  unsigned long now = millis();
  read_imu.run(now);

  stabilizer.run(now);
  vel_loop.run(now);
  communication.run(now);
  heartbeat.run(now);
}
