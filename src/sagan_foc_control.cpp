#include "sagan_foc_control.hpp"
#include <algorithm>

// MODIFICATION: The constructor now saves the sensor pointer
SaganFOCControl::SaganFOCControl(const Config& config, as5600_t& sensor_obj) 
    : config(config), sensor_obj(sensor_obj) { // <-- Save the sensor object here
    init_pwm();
    target_angle_rad = 0.0;
    continuous_raw_angle = 0;
    prev_raw_angle = -1;
}

// Method to initialize the PWM outputs for the 3 motor phases
void SaganFOCControl::init_pwm() {
    for (int i = 0; i < 3; i++) {
        gpio_set_function(config.pwm_pins[i], GPIO_FUNC_PWM);
        pwm_slice[i] = pwm_gpio_to_slice_num(config.pwm_pins[i]);
        pwm_channel[i] = pwm_gpio_to_channel(config.pwm_pins[i]);
        
        pwm_config pwm_cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&pwm_cfg, 4.f);
        pwm_config_set_wrap(&pwm_cfg, 1000);
        pwm_init(pwm_slice[i], &pwm_cfg, true);
    }

    gpio_init(config.enable_pin);
    gpio_set_dir(config.enable_pin, GPIO_OUT);
    gpio_put(config.enable_pin, true);
}

// MODIFICATION: Calibrate now uses the internal sensor pointer
void SaganFOCControl::calibrate() {
    printf("Initiating BLDC Motor Calibration...\n");
    sleep_ms(3000);

    for (int i = 0; i < config.pole_pairs; i++) {
        for (float electrical_angle = 0.0; electrical_angle <= 2 * M_PI; electrical_angle += M_PI / 64) {
            space_vector_modulation(0.8 * cosf(electrical_angle), 0.8 * sinf(electrical_angle));
            sleep_ms(100);

            // Reads sensor using the stored pointer
            int raw_sensor_angle = as5600_read_raw_angl(&sensor_obj); // <-- MODIFIED
            printf("Angle Readed: %i rad ; Angle Aplied: %.2f rad ; \n", raw_sensor_angle, electrical_angle);
            
            if (raw_sensor_angle >= 0 && raw_sensor_angle < LUT_SIZE) {
                this->lut_electrical_angle[raw_sensor_angle] = electrical_angle;
            }
        }
    }
    
    space_vector_modulation(0.0f, 0.0f);
    printf("Calibration Finalized.\n");
}

void SaganFOCControl::new_calibrate() {
    printf("Initiating BLDC Motor Calibration...\n");
    sleep_ms(1000);

    float angle_table[64] = {0.0f};

    for (int index = 0; index < 64; index++){
        angle_table[index] = M_PI / 64 * index;
    }

    int up_values[64] = {0};
    int up_values_average[64] = {0};
    
    int down_values[64] = {0};
    int down_values_average[64] = {0};

    int up_down_average[64] = {0};
    int up_down_delta[16] = {0};

    // --- Moving Average Variables ---
    const int WINDOW_SIZE = 10;
    int value_window[WINDOW_SIZE] = {0}; // An array to store the last 10 values
    int current_sum = 0.0f;                 // The running sum for efficiency
    int current_index = 0;  

    for (int index = 63; index >= 0; index--){
            space_vector_modulation(0.8*cosf(angle_table[index]),0.8*sinf(angle_table[index]));
            sleep_ms(100);
            //printf("%.2f;%i \n", angle_table[index], as5600_read_raw_angl(&as5600));
    }

    sleep_ms(1000);
    
    for (int index = 0; index < 64; index++){
        space_vector_modulation(0.8*cosf(angle_table[index]),0.8*sinf(angle_table[index]));
        sleep_ms(100);
        up_values[index] = as5600_read_raw_angl(&sensor_obj);
        printf("%.2f;%i \n", angle_table[index], up_values[index]);
        
    }

    for (int index = 63; index >= 0; index--){
        space_vector_modulation(0.8*cosf(angle_table[index]),0.8*sinf(angle_table[index]));
        sleep_ms(100);
        down_values[index] = as5600_read_raw_angl(&sensor_obj);
        printf("%.2f;%i \n", angle_table[index], down_values[index]);
    }

    for (int index = 0; index < 64; index++){
        int new_value = up_values[index];
        current_sum -= value_window[current_index];
        current_sum += new_value;
        value_window[current_index] = new_value;
        current_index = (current_index + 1) % WINDOW_SIZE;
        up_values_average[index] = current_sum / WINDOW_SIZE;
        sleep_ms(1);
    }

    for (int i = 0; i < 10; i++) value_window[i] = 0;
    current_sum = 0;                
    current_index = 0;  

    for (int index = 0; index < 64; index++){
        int new_value = down_values[index];
        current_sum -= value_window[current_index];
        current_sum += new_value;
        value_window[current_index] = new_value;
        current_index = (current_index + 1) % WINDOW_SIZE;
        down_values_average[index] = current_sum / WINDOW_SIZE;
        sleep_ms(1);
    }

    for (int i = 0; i < 10; i++) value_window[i] = 0;
    current_sum = 0;                
    current_index = 0; 

    for (int index = 0; index < 64; index++){
        up_down_average[index] = (up_values_average[index] + down_values_average[index]) / 2;
        //printf("%i\n", up_down_average[index]);
    }

    int median_point = static_cast<int>((up_down_average[32] + up_down_average[31]) / 2);

    printf("%i \n", median_point);

    this->lut_electrical_angle[median_point] = M_PI/2;

    for (int index = median_point; index < 4096; index++){
        this->lut_electrical_angle[index] = M_PI/2 + ((index - median_point) * (2 * M_PI / (4096 / 7))); 
    }

    for (int index = median_point; index >= 0; index--){
        this->lut_electrical_angle[index] = M_PI/2 + ((index - median_point) * (2 * M_PI / (4096 / 7))); 
    }

    space_vector_modulation(0.0f, 0.0f);
    sleep_ms(1000);
    printf("Calibration Finalized.\n");
}

// MODIFICATION: Update now uses the internal sensor pointer
void SaganFOCControl::update() {
    // Reads sensor using the stored pointer
    int current_raw_angle = as5600_read_raw_angl(&sensor_obj); // <-- MODIFIED

    if (prev_raw_angle == -1) prev_raw_angle = current_raw_angle;

    // --- The rest of the update logic is identical ---
    if (current_raw_angle > prev_raw_angle && (current_raw_angle - prev_raw_angle) > (LUT_SIZE / 2)) {
        continuous_raw_angle += (current_raw_angle - LUT_SIZE) - prev_raw_angle;
    } else if (current_raw_angle < prev_raw_angle && (prev_raw_angle - current_raw_angle) > (LUT_SIZE / 2)) {
        continuous_raw_angle += (current_raw_angle + LUT_SIZE) - prev_raw_angle;
    } else {
        continuous_raw_angle += current_raw_angle - prev_raw_angle;
    }
    prev_raw_angle = current_raw_angle;

    float current_angle_rad = static_cast<float>(continuous_raw_angle) * 2.0f * M_PI / LUT_SIZE;
    float angle_error = target_angle_rad - current_angle_rad;

    int control_offset = static_cast<int>(fabs(angle_error) * config.angle_p_gain);
    if (control_offset >= 200) control_offset = 200;

    float control_offset_field = fabs(angle_error) * config.field_p_gain;
    if (control_offset_field >= 0.6f) control_offset_field = 0.6f;
    else if (control_offset_field <= 0.3f) control_offset_field = 0.3f;

    int final_angle_lookup_index = (angle_error > 0.0) ? (current_raw_angle + control_offset) : (current_raw_angle - control_offset);
    int lut_index = (final_angle_lookup_index + LUT_SIZE) % LUT_SIZE;
    float electrical_angle_to_apply = this->lut_electrical_angle[lut_index];

    space_vector_modulation(control_offset_field * cosf(electrical_angle_to_apply), control_offset_field * sinf(electrical_angle_to_apply));
}

// NEW: Implementation for the position getter function
float SaganFOCControl::get_position_rad() const {
    // Converts the internal continuous angle state to radians and returns it
    return static_cast<float>(this->continuous_raw_angle) * 2.0f * M_PI / LUT_SIZE;
}

// Method to update the controller's target
void SaganFOCControl::set_target_angle(float target_rad) {
    this->target_angle_rad = target_rad;
}

// Low-level SVM function (copied from your old library)
void SaganFOCControl::space_vector_modulation(float v_alpha, float v_beta) {
    // Space Vector Modulation (SVM)
    float v_u = v_alpha;
    float v_v = -0.5f * v_alpha + 0.86602540378f * v_beta; // sqrt(3)/2
    float v_w = -0.5f * v_alpha - 0.86602540378f * v_beta;
    
    // Normalize and scale to PWM range
    float max_val = std::max(std::max(std::abs(v_u), std::abs(v_v)), std::abs(v_w));
    if (max_val > 1.0f) {
        v_u /= max_val;
        v_v /= max_val;
        v_w /= max_val;
    }
    
    // Convert to PWM duty cycles (0-1 range)
    float duty_u = 0.5f + 0.5f * v_u;
    float duty_v = 0.5f + 0.5f * v_v;
    float duty_w = 0.5f + 0.5f * v_w;
    
    // Set PWM outputs
    pwm_set_chan_level(pwm_slice[0], pwm_channel[0], static_cast<uint16_t>(duty_u * 1000));
    pwm_set_chan_level(pwm_slice[1], pwm_channel[1], static_cast<uint16_t>(duty_v * 1000));
    pwm_set_chan_level(pwm_slice[2], pwm_channel[2], static_cast<uint16_t>(duty_w * 1000));
}