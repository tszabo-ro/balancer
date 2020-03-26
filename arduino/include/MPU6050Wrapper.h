#ifndef MPU6050WRAPPER_H
#define MPU6050WRAPPER_H

#include <inttypes.h>

extern volatile bool mpuInterrupt;

struct MPU6050;
struct GyroCalib;

class MPU6050Wrapper
{
public:
  // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
  // AD0 high = 0x69
  MPU6050Wrapper(int interrupt_pin, int mpu_id=0x68);
  ~MPU6050Wrapper();
  int initialize(const GyroCalib& calib_data);

  bool read();

  float getRoll() const
  {
    return ypr[2];
  }

  float getPitch() const
  {
    return ypr[1];
  }

  float getYaw() const
  {
    return ypr[0];
  }
private:
  MPU6050* mpu;

  int interrupt_pin;  // use pin 2 on Arduino Uno & most boards
  bool dmp_ready;  // set true if DMP init was successful

  // orientation/motion vars
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

};
#endif
