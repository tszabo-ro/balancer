#include "include/Calibration.h"

#include <Arduino.h>
#include <EEPROM.h>

DeviceCalibration::DeviceCalibration(unsigned int start_address)
: eeprom_address(start_address)
{
  load();
}

void DeviceCalibration::load()
{
  uint16_t address = eeprom_address;

  EEPROM.get(address, gyro);
  address += sizeof(GyroCalib);

  EEPROM.get(address, l_motor);
  address += sizeof(MotorCalib);

  EEPROM.get(address, r_motor);
  address += sizeof(MotorCalib);
}
void DeviceCalibration::store()
{
  uint16_t address = eeprom_address;

  EEPROM.put(address, gyro);
  address += sizeof(GyroCalib);

  EEPROM.put(address, l_motor);
  address += sizeof(MotorCalib);

  EEPROM.put(address, r_motor);
  address += sizeof(MotorCalib);
}

void DeviceCalibration::print() const
{
  Serial.println("Calibration: ");
  Serial.println("");
  Serial.println("IMU: ");

  Serial.print("gx: ");
  Serial.println(gyro.gyro_x_offset);
  Serial.print("gy: ");
  Serial.println(gyro.gyro_y_offset);
  Serial.print("gz: ");
  Serial.println(gyro.gyro_z_offset);

  Serial.print("ax: ");
  Serial.println(gyro.acc_x_offset);
  Serial.print("ay: ");
  Serial.println(gyro.acc_y_offset);
  Serial.print("az: ");
  Serial.println(gyro.acc_z_offset);
  Serial.println("");

  Serial.println("Motor L");
  Serial.print("min_out: ");
  Serial.println(l_motor.min_output);
  Serial.print("max_out: ");
  Serial.println(l_motor.max_output);
  Serial.println("");

  Serial.println("Motor R");
  Serial.print("min_out: ");
  Serial.println(r_motor.min_output);
  Serial.print("max_out: ");
  Serial.println(r_motor.max_output);
}


static int16_t readValue(const char* param_name, int16_t current_val)
{
  Serial.print(param_name);
  Serial.print(" [");
  Serial.print(current_val);
  Serial.print("] : ");

  int16_t new_val = current_val;

  String new_val_str = Serial.readStringUntil('\n');
  if (new_val_str.length() > 1)
  {
    new_val = new_val_str.toInt();
  }
  Serial.println(new_val);

  return new_val;
}

static void printVal(const char* str, int16_t val)
{
  Serial.print(str);
  Serial.print(": ");
  Serial.println(val);
}
void DeviceCalibration::read()
{
  Serial.setTimeout(9999000);
  Serial.println("Calibration update started.");

  bool update_read = false;
  while (!update_read)
  {
    int16_t gyro_gyro_x_offset = readValue("gyro_x_offset", gyro.gyro_x_offset);
    int16_t gyro_gyro_y_offset = readValue("gyro_y_offset", gyro.gyro_y_offset);
    int16_t gyro_gyro_z_offset = readValue("gyro_z_offset", gyro.gyro_z_offset);

    int16_t gyro_acc_x_offset = readValue("acc_x_offset", gyro.acc_x_offset);
    int16_t gyro_acc_y_offset = readValue("acc_y_offset", gyro.acc_y_offset);
    int16_t gyro_acc_z_offset = readValue("acc_z_offset", gyro.acc_z_offset);

    int16_t l_motor_min_output = readValue("l_motor.min_output", l_motor.min_output);
    int16_t l_motor_max_output = readValue("l_motor.max_output", l_motor.max_output);

    int16_t r_motor_min_output = readValue("r_motor.min_output", r_motor.min_output);
    int16_t r_motor_max_output = readValue("r_motor.max_output", r_motor.max_output);

    Serial.println("Calibration update finished. Current calibration set:");

    printVal("gyro_x_offset", gyro_gyro_x_offset);
    printVal("gyro_y_offset", gyro_gyro_y_offset);
    printVal("gyro_z_offset", gyro_gyro_z_offset);

    printVal("acc_x_offset", gyro_acc_x_offset);
    printVal("acc_y_offset", gyro_acc_y_offset);
    printVal("acc_z_offset", gyro_acc_z_offset);

    printVal("Left motor min", l_motor_min_output);
    printVal("Left motor max", l_motor_max_output);

    printVal("Right motor min", r_motor_min_output);
    printVal("Right motor max", r_motor_max_output);

    bool valid_response = false;
    while (!valid_response)
    {
      Serial.println("Do you want to store these? [Y/N]");

      String new_val_str = Serial.readStringUntil('\n');
      new_val_str.toUpperCase();
      if (new_val_str.startsWith("Y"))
      {
        gyro.acc_x_offset = gyro_gyro_x_offset;
        gyro.acc_y_offset = gyro_gyro_y_offset;
        gyro.acc_z_offset = gyro_gyro_z_offset;

        gyro.acc_x_offset = gyro_acc_x_offset;
        gyro.acc_y_offset = gyro_acc_y_offset;
        gyro.acc_z_offset = gyro_acc_z_offset;

        l_motor.min_output = l_motor_min_output;
        l_motor.max_output = l_motor_max_output;

        r_motor.min_output = r_motor_min_output;
        r_motor.max_output = r_motor_max_output;

        store();

        Serial.println("Calibration updated. Please reconnect the serial line and restart the controller.");
        while (true)
        {
        }
      }
      else if (new_val_str.startsWith("N"))
      {
        Serial.println("Readings discarded. Restarting the calibration readings. You may cancel this by restarting the device!");
        Serial.println("");
        valid_response = true;
      }

    }
  }
}

/////////////////////////

PIDParams::PIDParams(unsigned int start_address)
: eeprom_address(start_address)
{
  load();
}

void PIDParams::load()
{
  uint16_t address = eeprom_address;

  EEPROM.get(address, kP);
  address += sizeof(float);

  EEPROM.get(address, kD);
  address += sizeof(float);

  EEPROM.get(address, kI);
  address += sizeof(float);
}

void PIDParams::store()
{
  uint16_t address = eeprom_address;

  EEPROM.put(address, kP);
  address += sizeof(float);

  EEPROM.put(address, kD);
  address += sizeof(float);

  EEPROM.put(address, kI);
  address += sizeof(float);
}

void PIDParams::print() const
{
  Serial.println("Params:");
  Serial.print("kP: ");
  Serial.println(kP);
  Serial.print("kD: ");
  Serial.println(kD);
  Serial.print("kI: ");
  Serial.println(kI);
}

/////////////////////////

Params::Params(unsigned int start_address)
: eeprom_address(start_address)
, inner(eeprom_address)
, outer(eeprom_address + sizeof(PIDParams))
{
  load();
}

void Params::load()
{
  inner.load();
  outer.load();
}

void Params::store()
{
  inner.store();
  outer.store();
}

void Params::print() const
{
  Serial.println("Inner gains:");
  inner.print();
  Serial.println("Outer gains:");
  outer.print();
}

/////////////////////////