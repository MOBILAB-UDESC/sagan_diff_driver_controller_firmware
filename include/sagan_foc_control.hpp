#ifndef SAGAN_FOC_CONTROL_HPP
#define SAGAN_FOC_CONTROL_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "magnetic_encoder.h"
#include <cmath>

class SaganFOCControl{
public:
    struct Config {
        uint8_t pwm_pins[3];
        uint8_t enable_pin;      
        int pole_pairs;
        float angle_p_gain;
        float field_p_gain;
    };

    // MODIFICATION: The constructor now takes a pointer to your sensor object
    SaganFOCControl(const Config& config, as5600_t& sensor_obj);

    // MODIFICATION: No longer needs a function pointer
    void calibrate();

    void new_calibrate();

    // MODIFICATION: No longer needs the current angle passed in
    void update();
    
    // NEW: Function to get the motor's current position in radians
    float get_position_rad() const;

    void set_target_angle(float target_rad);

    void space_vector_modulation(float v_alpha, float v_beta);

private:
    void init_pwm();
    

    // --- Member Variables ---
    Config config;
    
    // MODIFICATION: A pointer to hold the sensor object
    as5600_t sensor_obj;

    // PWM hardware details
    uint pwm_slice[3];
    uint pwm_channel[3];

    // ... other member variables are the same ...
    static const int LUT_SIZE = 4096;
    float lut_electrical_angle[LUT_SIZE] = {0.0f};
    float target_angle_rad = 0.0f;
    long continuous_raw_angle = 0;
    int prev_raw_angle = -1;
};

#endif // SAGAN_FOC_CONTROL_HPP