#include <stdio.h>
#include <utility>
#include <stdint.h>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"

#include "magnetic_encoder.h"
//#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
#include "sagan_foc_control.hpp"
#include "sagan_movement_control.hpp"

#define SPACES "                              "

// Front Motor Pins
#define FRONT_ENC_A_PIN 16 // B channel needs to be connected to the following pin (in this case pin 11)
#define FRONT_ENABLE_WHEEL_DRIVER_PIN 20
#define FRONT_CS_WHEEL_DRIVER_PIN 27
#define FRONT_INPUT_A_WHEEL_DRIVER_PIN 10
#define FRONT_INPUT_B_WHEEL_DRIVER_PIN 11
#define FRONT_PWM_WHEEL_DRIVER_PIN 12

// Rear Motor Pins
#define REAR_ENC_A_PIN 14 // B channel needs to be connected to the following pin (in this case pin 11)
#define REAR_ENABLE_WHEEL_DRIVER_PIN 21
#define REAR_CS_WHEEL_DRIVER_PIN 26
#define REAR_INPUT_A_WHEEL_DRIVER_PIN 6
#define REAR_INPUT_B_WHEEL_DRIVER_PIN 7
#define REAR_PWM_WHEEL_DRIVER_PIN 8


#define motor_data_acquisition()                                                                                 \
    for (int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20)                                                   \
    {                                                                                                            \
        for (int i = 0; i < 1000; i++)                                                                           \
        {                                                                                                        \
            encoder.update(sampling_time);                                                                       \
            PWM_PERCENTAGE_WHEEL = pwm_valor;                                                                    \
            printf("Actual Vel: %.4f | Wheel PWM %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                                                    \
            sleep_ms(5);                                                                                         \
        }                                                                                                        \
    }

#define motor_current_data_acquisition()                                                                                                                     \
    for (int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20)                                                                                               \
    {                                                                                                                                                        \
        for (int i = 0; i < 5000; i++)                                                                                                                       \
        {                                                                                                                                                    \
            encoder.update(sampling_time);                                                                                                                   \
            PWM_PERCENTAGE_WHEEL = pwm_valor;                                                                                                                \
            wheel_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;                                                               \
            printf("Actual Vel: %.4f | Wheel PWM %.2f | Wheel Motor Current : %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, wheel_current, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                                                                                                \
            sleep_ms(1);                                                                                                                                     \
        }                                                                                                                                                    \
    }

// Master -> Slave
struct ControlData {
    uint8_t cmd;         // Command (e.g., 0xA1)
    int16_t front_velocity;   // Target velocity for motor 1 (e.g., 15.5 rad/s becomes 1550)
    int16_t rear_velocity;   // Target velocity for motor 2
};

// Slave -> Master
struct SensorData {
    int16_t front_velocity;     // Actual velocity of motor 1
    int16_t front_current;      // Current draw of motor 1 (in mA to avoid floats)
    int16_t rear_velocity;     // Actual velocity of motor 2
    int16_t rear_current;      // Current draw of motor 2
};

// Struct to hold sensor data that the master can request
volatile SensorData sensor_data_to_master;

// Target velocities received from the master
volatile float target_front_velocity = 0.0f;
volatile float target_rear_velocity = 0.0f; // Add when you have a second motor

// A state variable to know what the master last asked for
volatile uint8_t last_command_received = 0;

// Conversion factor for velocity/current data
const float DATA_SCALE_FACTOR = 100.0f;

// A buffer to store the bytes as they arrive from the master
volatile uint8_t i2c_rx_buffer[sizeof(ControlData)];
// A counter for how many bytes we've received in the current transaction
volatile uint8_t i2c_rx_index = 0;

int16_t vel_front_raw = 0;
int16_t vel_rear_raw  = 0;

// Add these new global variables
volatile bool new_data_from_master = false;
volatile ControlData received_control_data;

// Corrected, non-blocking handler
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: {
        // This event fires for EACH byte received.
        if (i2c_rx_index < sizeof(ControlData)) {
            uint8_t byte = i2c_read_byte_raw(i2c);
            // If this is the FIRST byte of a transaction, it's our command.
            if (i2c_rx_index == 0) {
                last_command_received = byte;
            }
            i2c_rx_buffer[i2c_rx_index++] = byte;
        } else {
            // Buffer overflow, discard
            (void)i2c_read_byte_raw(i2c);
        }
        break;
    }
    case I2C_SLAVE_REQUEST: {
        // Now this check will work correctly, even during a repeated start
        if (last_command_received == 0xB1) {
            i2c_write_raw_blocking(i2c, (const uint8_t*)&sensor_data_to_master, sizeof(SensorData));
        }
        break;
    }
    case I2C_SLAVE_FINISH: {
        // This case is now ONLY for processing multi-byte commands like SET_VELOCITIES
        if (last_command_received == 0xA1 && i2c_rx_index == sizeof(ControlData)) {
            memcpy((void*)&received_control_data, (void*)i2c_rx_buffer, sizeof(ControlData));
            new_data_from_master = true;
        }
        // Always reset the index for the next transaction
        i2c_rx_index = 0;
        break;
    }
    default:
        break;
    }
}


float sampling_time = 10e-3;

QuadratureEncoder encoder_front_wheel(FRONT_ENC_A_PIN, 16, 30.0); // 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)
QuadratureEncoder encoder_rear_wheel(REAR_ENC_A_PIN, 16, 30.0); // 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)

MotorDriver driver_front_wheel(FRONT_ENABLE_WHEEL_DRIVER_PIN, FRONT_CS_WHEEL_DRIVER_PIN, FRONT_INPUT_A_WHEEL_DRIVER_PIN, FRONT_INPUT_B_WHEEL_DRIVER_PIN, FRONT_PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);
MotorDriver driver_rear_wheel(REAR_ENABLE_WHEEL_DRIVER_PIN, REAR_CS_WHEEL_DRIVER_PIN, REAR_INPUT_A_WHEEL_DRIVER_PIN, REAR_INPUT_B_WHEEL_DRIVER_PIN, REAR_PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);

SpeedControl control_front_wheel(sampling_time, 100);
SpeedControl control_rear_wheel(sampling_time, 100);

WheelDriver front_wheel(driver_front_wheel, control_front_wheel, encoder_front_wheel);
WheelDriver rear_wheel(driver_rear_wheel, control_rear_wheel, encoder_rear_wheel);

int main()
{
    float sampling_time = 10e-3;
    
    stdio_init_all();

    // Setup i2c slave   
    i2c_inst_t *i2c = i2c0;

    const uint SDA_PIN = 4;
    const uint SCL_PIN = 5;

    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(i2c0, 400 * 1000);
    i2c_slave_init(i2c_default, 0x15, &i2c_slave_handler);

    //Setting the board LED ON
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("Initializing code...\n");
    sleep_ms(2000);
 

    while (true) {
        int16_t vel_front_raw = received_control_data.front_velocity;
        int16_t vel_rear_raw  = received_control_data.rear_velocity;

        target_front_velocity = (float)vel_front_raw / DATA_SCALE_FACTOR;
        target_rear_velocity  = (float)vel_rear_raw / DATA_SCALE_FACTOR;

        sensor_data_to_master.front_velocity = (int16_t)(front_wheel.get_velocity() * DATA_SCALE_FACTOR);
        sensor_data_to_master.front_current = (int16_t)(front_wheel.get_current() * 1000.0f);
        sensor_data_to_master.rear_velocity = (int16_t)(rear_wheel.get_velocity() * DATA_SCALE_FACTOR);
        sensor_data_to_master.rear_current = (int16_t)(rear_wheel.get_current() * 1000.0f);

        // Set the velocities based on the latest targets
        front_wheel.set_velocity(target_front_velocity);
        rear_wheel.set_velocity(target_rear_velocity);

        // Run the control loops
        front_wheel.motor_loop();
        rear_wheel.motor_loop();
        
        // Optional: Print current status
        printf("Target Vels: F=%.2f, R=%.2f | Actual Vels: F=%.2f, R=%.2f\n", target_front_velocity, target_rear_velocity, front_wheel.get_velocity(), rear_wheel.get_velocity());
        sleep_ms(10);
    }  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//            MOTOR POLE TEST
//  This test is runned to know the motor poles number, if the correct value is asigned to the NUM_POLES variable
//  the motor will do one complete turn
// for (int i = 1; i <= POLES_NUM; i++)
// {
//     printf("Initialazing %i turn \n", i);
//     for (float intra_angle = 0.0; intra_angle <= 2*M_PI; intra_angle += M_PI/8)
//     {
//         alpha = intra_angle;
//         beta = intra_angle;
//         motor.space_vector_modulation(0.5 * cosf(alpha), 0.5 * sinf(beta));
//         sleep_ms(100);
//         angle = static_cast<float>(as5600_read_raw_angl(&as5600)) * 2 * M_PI / 4096.0;
//         printf("Position: %.2f rad (%.1f°), Electrical Angle: %.2f rad (%.1f°) \n", angle, angle * 180.0f / M_PI, intra_angle, intra_angle * 180.0f / M_PI);
//     }
// }
// printf("Finalized \n");
// sleep_ms(10000);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 // std::vector<float> velocity_vector = {0, 20, 30, 20, 10, 30, 10, 15};
        // printf("TargetVel;ActualVel;ControllerEffort,\n", SPACES);
        // for (int index = 0; index < velocity_vector.size(); index++)
        // {
        //     for (int tempo = 0; tempo <= 200; tempo++)
        //     {
        //         actual_time = time_us_64();
        //         encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //         prev_time = actual_time;
                
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcRagazzini(velocity_vector[index], encoder.get_velocity());
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);
        //         printf("%.2f;%.2f;%.2f\n",velocity_vector[index], encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES);

        //         sleep_ms(10);
        //     }
        // }
        
        
        // std::vector<int> PWM_vector = {0, 20, 60, 80, 40, 80, 60, 10, 90, 10};
        // printf("PWM;ActualVel;\n", SPACES);
        // for (int index = 0; index < PWM_vector.size(); index++)
        // {
        //     PWM_PERCENTAGE_WHEEL = PWM_vector[index];
        //     for (int tempo = 0; tempo <= 200; tempo++)
        //     {
        //         actual_time = time_us_64();
        //         encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //         prev_time = actual_time;
                
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);

        //         printf("%.2f;%.2f;\n",PWM_PERCENTAGE_WHEEL, encoder.get_velocity(), SPACES);

        //         sleep_ms(10);
        //     }
        // }

        // printf("TargeVel;ActualVel;EstimatedVel;ControllerEffort;Current;CurrentMedian;\n", SPACES);
        // for (float vel = 0; vel <= 35; vel += 5)
        // {
        //     targetVel = vel;
        //     for (int tempo = 0; tempo <= 500; tempo++)
        //     {
        //         for (int count = 0; count < 10; count++)
        //         {
        //             actual_time = time_us_64();
        //             encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //             prev_time = actual_time;

        //             wheel_current = wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
        //             wheel_current_median = MotorA.movingMedian(wheel_current - 0.40);
        //             wheel_velocity = MotorA.conversorCurrent2Velocity(wheel_current_median);
        //             //printf("Target Vel: %.2f | Actual Vel: %.2f | Estimated vel: %.2f | Wheel Controller Effort %.2f | Wheel motor current: %.2f | Wheel motor current median: %.2f \n", targetVel, encoder.get_velocity(), wheel_velocity, PWM_PERCENTAGE_WHEEL, wheel_current, wheel_current_median, SPACES);
        //             printf("%.2f;%.2f;%.2f;%.2f;%.2f;%.2f; \n", targetVel, encoder.get_velocity(), wheel_velocity, PWM_PERCENTAGE_WHEEL, wheel_current, wheel_current_median, SPACES);
        //             sleep_ms(1);
        //         }
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcPI(targetVel, wheel_velocity);
        //         //printf("%.2f,%.2f,%.2f\n",PWM_PERCENTAGE_WHEEL, wheel_current, encoder.get_velocity(), SPACES);
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);
        //     }
        // }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // angle_target = -1.0;
        // BLDC.set_target_angle(angle_target); // Set initial target
        // BLDC2.set_target_angle(angle_target); // Set initial target
        // for (int index = 0; index <= 5000; index++){
        //     BLDC.update();
        //     BLDC2.update();
        //     printf("BLDC Position %.2f | BLDC Target %.2f\n", BLDC.get_position_rad(), angle_target, SPACES);
        //     sleep_ms(1);
        // }
