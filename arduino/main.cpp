
#include <Arduino.h>
#include "include/MPU6050Wrapper.h"


MPU6050Wrapper mpu(2);

float getSupplyVoltage()
{
  int ref = analogRead(A0);
  constexpr float shunt_multiplier = 3.0;
  constexpr float ref_voltage = 5.0;
  return (static_cast<float>(ref)/1023.0) * shunt_multiplier * ref_voltage;
}

unsigned long last_print_time = 0;
void setup()
{
  // a/g:	18400	10398	7522	1135	267	-211
  Serial.begin(115200);

  mpu.initialize( -59,    // gyro_x_offset
                  9,      // gyro_y_offset
                  -30,    // gyro_z_offset
                  -284,   // acc_x_offset
                  -1123,  // acc_y_offset
                  927);   // acc_z_offset

  pinMode(A0, INPUT);
  last_print_time = millis();
}


void loop()
{
  if (!mpu.read())
  {
    return;
  }

  float roll  = mpu.getRoll();
  float pitch = mpu.getPitch();
  float yaw   = mpu.getYaw();

  float voltage = getSupplyVoltage();

  unsigned long now = millis();

  if (now - last_print_time > 20)
  {
    Serial.print("ypr\t");
    Serial.print(yaw * 180/M_PI);
    Serial.print("\t");
    Serial.print(pitch * 180/M_PI);
    Serial.print("\t");
    Serial.print(roll * 180/M_PI);
    Serial.print("\t - \t");
    Serial.print(voltage);
    Serial.println("");
  }
}