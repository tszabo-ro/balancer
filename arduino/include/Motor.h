#ifndef MOTOR_H
#define MOTOR_H

#include <inttypes.h>

struct MotorCalib;
struct Adafruit_PWMServoDriver;

using PWMDriver = Adafruit_PWMServoDriver;


class Motor
{
public:
    Motor(PWMDriver& pwm, int fwd_pin, int bck_pin);
    int initialize(const MotorCalib& calib_data);

    void setSpeed(float vel_rpm);
    float setSpeed() const { return speed_target_; }
    float getSpeed() const { return current_speed_; }

    void controlCycle();

    void disable(bool disable) { disabled_ = disable; }

    void encoderTick(bool A, bool B);


private:
    const int fwd_pin_;
    const int bck_pin_;

    bool disabled_;

    int min_out_;
    int max_out_;

    PWMDriver& pwm_;

    unsigned long last_cycle_time_;
    int16_t tick_count_;

    float speed_target_;
    float current_speed_;

    double speed_error_integral_;
};

#endif