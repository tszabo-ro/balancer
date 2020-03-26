#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>

struct MotorCalib;

class Motor
{
public:
    Motor(int fwd_pin, int bck_pin);
    int initialize(const MotorCalib& calib_data);

    int16_t setSpeed(int16_t vel);

private:
    const int fwd_pin_;
    const int bck_pin_;

    int min_out_;
    int max_out_;
};

#endif