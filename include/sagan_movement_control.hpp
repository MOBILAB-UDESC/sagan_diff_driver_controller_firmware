#include <stdio.h>
#include <utility>
#include <stdint.h>
#include "pico/stdlib.h"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
#include "quadrature_encoder.hpp"

class WheelDriver {
public:

    WheelDriver(MotorDriver motor, SpeedControl controller, QuadratureEncoder encoder);

    void set_velocity(float target_velocity);

    float get_velocity();

    float get_current();
private:

    void motor_set_input(float PWM_INPUT);

    void update_velocity();

    float target_velocity;
    float actual_velocity;
    float actual_time;
    float prev_time = 0.0f;

    MotorDriver motor;
    SpeedControl controller;    
    QuadratureEncoder encoder;
};