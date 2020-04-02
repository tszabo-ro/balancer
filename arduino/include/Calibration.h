#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <inttypes.h>

struct GyroCalib
{
  int16_t gyro_x_offset;
  int16_t gyro_y_offset;
  int16_t gyro_z_offset;

  int16_t acc_x_offset;
  int16_t acc_y_offset;
  int16_t acc_z_offset;
};

struct MotorCalib
{
    uint16_t min_output;
    uint16_t max_output;
};


struct DeviceCalibration
{
  DeviceCalibration(unsigned int start_address);
  const unsigned int eeprom_address;

  GyroCalib gyro;
  MotorCalib l_motor;
  MotorCalib r_motor;

  void load();
  void store();
  void print() const;

  void read();
};

struct PIDParams
{
  PIDParams(unsigned int start_address);
  const unsigned int eeprom_address;

  float kP;
  float kD;
  float kI;

  void load();
  void store();
  void print() const;
};

struct Params
{
  Params(unsigned int start_address);
  const unsigned int eeprom_address;

  PIDParams inner;
  PIDParams outer;

  void load();
  void store();
  void print() const;
};

struct StoredData
{
  StoredData()
  : calib(0)
  , params(calib.eeprom_address + sizeof(DeviceCalibration))
  {
  }

  DeviceCalibration calib;
  Params params;
};

#endif