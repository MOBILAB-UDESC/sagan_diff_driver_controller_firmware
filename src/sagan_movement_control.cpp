#include "sagan_movement_control.hpp"

WheelDriver::WheelDriver(MotorDriver motor, SpeedControl controller, QuadratureEncoder encoder) {
    this->motor = motor;
    this->controller = controller;
    this->encoder = encoder;
};

void WheelDriver::set_velocity(float target_velocity) {
    this->target_velocity = target_velocity;
}

float WheelDriver::get_velocity() {
    return this->actual_velocity; 
}

void motor_loop() {
    actual_time = time_us_64();
    float dt = (float)(actual_time - prev_time) / 1000000.0f;
    this->encoder.update(dt);
    prev_time = actual_time;
    this->update_velocity();
    float control_effort = controller.controlCalcRagazzini(this->target_velocity, this->actual_velocity)
    this->motor_set_input(control_effort);
}

float WheelDriver::get_current() {
    float motor_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
    return motor_current;
}

void WheelDriver::motor_set_input(float PWM_INPUT) {
    float PWM_VALUE = 0.0;
    if (PWM_INPUT >= 100 || PWM_INPUT <= -100)
    {
        if (PWM_INPUT > 0)
        {
            PWM_INPUT = 100;
            PWM_VALUE = round(PWM_INPUT * 255 / 100);
            motor.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
            motor.setMotorOutput(PWM_VALUE);
        }
        else
        {
            PWM_INPUT = -100;
            PWM_VALUE = round(-PWM_INPUT * 255 / 100);
            motor.turnOnMotor(MotorDriver::CLOCKWISE);
            motor.setMotorOutput(PWM_VALUE);
        }
    }
    else if (PWM_INPUT > 0)
    {
        PWM_VALUE = round(PWM_INPUT * 255 / 100);
        motor.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        motor.setMotorOutput(PWM_VALUE);
    }
    else
    {
        PWM_VALUE = round(-PWM_INPUT * 255 / 100);
        motor.turnOnMotor(MotorDriver::CLOCKWISE);
        motor.setMotorOutput(PWM_VALUE);
    }
}

void WheelDriver::update_velocity() {
    this->actual_velocity = encoder.get_velocity();
}






