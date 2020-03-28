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