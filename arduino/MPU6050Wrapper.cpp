#include "include/MPU6050Wrapper.h"

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

volatile bool mpuInterrupt = false;

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050Wrapper::MPU6050Wrapper(int interrupt_pin, int mpu_id)
: mpu(new MPU6050(mpu_id))
, interrupt_pin(interrupt_pin)
, dmp_ready(false)
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}
MPU6050Wrapper::~MPU6050Wrapper()
{
    // delete mpu; // <= Not supported by AVR?
}

int MPU6050Wrapper::initialize(int gyro_x_offset,
                int gyro_y_offset,
                int gyro_z_offset,
                int acc_x_offset,
                int acc_y_offset,
                int acc_z_offset)
{
    mpu->initialize();
    pinMode(interrupt_pin, INPUT);

    // verify connection
    if (!mpu->testConnection())
    {
        return -1;
    }

    int dev_status = mpu->dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu->setXGyroOffset(gyro_x_offset);
    mpu->setYGyroOffset(gyro_y_offset);
    mpu->setZGyroOffset(gyro_z_offset);

    mpu->setXAccelOffset(acc_x_offset);
    mpu->setYAccelOffset(acc_y_offset);
    mpu->setZAccelOffset(acc_z_offset);

    // make sure it worked (returns 0 if so)
    if (dev_status == 0)
    {
        mpu->setDMPEnabled(true);
        attachInterrupt(interrupt_pin, [](){ mpuInterrupt = true; }, RISING);

        dmp_ready = true;
    }
    // 0 = success
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    return dev_status;
}

bool MPU6050Wrapper::read()
{
    uint8_t fifo_buffer[64]; // FIFO storage buffer
    if (!dmp_ready || !mpu->dmpGetCurrentFIFOPacket(fifo_buffer))
    {
        return false;
    }

    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    // VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    // VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector

    mpu->dmpGetQuaternion(&q, fifo_buffer);

    mpu->dmpGetAccel(&aa, fifo_buffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

    return true;
}
