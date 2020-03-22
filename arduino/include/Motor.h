#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
public:
    Motor(int fwd_pin, int bck_pin, int max_output = 255, int min_output = 0);

    int16_t setSpeed(double vel);

private:
    const int fwd_pin_;
    const int bck_pin_;

    const int min_out_;
    const int max_out_;
};

#endif