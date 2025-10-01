#include <stdio.h>
#include <utility>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"

#include "magnetic_encoder.h"
#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
#include "sagan_foc_control.hpp"

#define SPACES "                              "

// Pins
#define ENCA_PIN 10 // B channel needs to be connected to the following pin (in this case pin 11)
// Wheel Motor Driver Pins
#define ENABLE_WHEEL_DRIVER_PIN 18
#define CS_WHEEL_DRIVER_PIN 28
#define INPUT_A_WHEEL_DRIVER_PIN 21
#define INPUT_B_WHEEL_DRIVER_PIN 20
#define PWM_WHEEL_DRIVER_PIN 19

void motor_update(float PWM_INPUT, MotorDriver Motor_select)
{
    float PWM_VALUE = 0.0;
    if (PWM_INPUT >= 100 || PWM_INPUT <= -100)
    {
        if (PWM_INPUT > 0)
        {
            PWM_INPUT = 100;
            PWM_VALUE = round(PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        }
        else
        {
            PWM_INPUT = -100;
            PWM_VALUE = round(-PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        }
    }
    else if (PWM_INPUT > 0)
    {
        PWM_VALUE = round(PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    }
    else
    {
        PWM_VALUE = round(-PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    }
}

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
    int16_t velocity1;   // Target velocity for motor 1 (e.g., 15.5 rad/s becomes 1550)
    int16_t velocity2;   // Target velocity for motor 2
};

// Slave -> Master
struct SensorData {
    int16_t velocity1;     // Actual velocity of motor 1
    int16_t current1;      // Current draw of motor 1 (in mA to avoid floats)
    int16_t velocity2;     // Actual velocity of motor 2
    int16_t current2;      // Current draw of motor 2
};

// Struct to hold sensor data that the master can request
volatile SensorData sensor_data_to_master;

// Target velocities received from the master
volatile float target_velocity_1 = 0.0f;
volatile float target_velocity_2 = 0.0f; // Add when you have a second motor

// A state variable to know what the master last asked for
volatile uint8_t last_command_received = 0;

// Conversion factor for velocity/current data
const float DATA_SCALE_FACTOR = 100.0f;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
    case I2C_SLAVE_RECEIVE: { // Master has written data to us
        // The master is sending a command and its data
        uint8_t received_data[sizeof(ControlData)];
        i2c_read_raw_blocking(i2c, received_data, sizeof(ControlData));

        // First byte is the command
        uint8_t cmd = received_data[0];
        last_command_received = cmd; // Store the command

        if (cmd == 0xA1) { // SET_VELOCITIES command
            // Extract the two 16-bit velocity values from the byte array
            int16_t vel1_raw = (int16_t)(received_data[1] | (received_data[2] << 8));
            // int16_t vel2_raw = (int16_t)(received_data[3] | (received_data[4] << 8)); // For motor 2

            // Convert back to float and update the target velocity for the main loop
            target_velocity_1 = (float)vel1_raw / DATA_SCALE_FACTOR;
            // target_velocity_2 = (float)vel2_raw / DATA_SCALE_FACTOR; // For motor 2
        }
        // The GET_SENSOR_DATA (0xB1) command requires no data, it just sets the state
        break;
    }
    case I2C_SLAVE_REQUEST: { // Master is requesting data from us
        if (last_command_received == 0xB1) { // GET_SENSOR_DATA
            // The main loop is continuously updating sensor_data_to_master.
            // We just need to send it.
            i2c_write_raw_blocking(i2c, (const uint8_t*)&sensor_data_to_master, sizeof(SensorData));
        }
        break;
    }
    case I2C_SLAVE_FINISH: // Master has signalled Stop / Restart
        last_command_received = 0; // Reset state
        break;
    default:
        break;
    }
}

int main()
{
    float sampling_time = 10e-3;
    QuadratureEncoder encoder(ENCA_PIN, 16, 30.0); // 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)

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

    // Setup motor drivers
    float kp = 5;
    float ki = 8;
    float kd = 0;
    float N = 0;
    float targetVel = 0;

    uint64_t actual_time = 0;
    uint64_t prev_time = 0;

    float wheel_current = 0;
    float wheel_current_median = 0;
    float wheel_velocity = 0;

    MotorDriver wheel_driver(ENABLE_WHEEL_DRIVER_PIN, CS_WHEEL_DRIVER_PIN, INPUT_A_WHEEL_DRIVER_PIN, INPUT_B_WHEEL_DRIVER_PIN, PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);
    SpeedControl MotorA(kp, ki, kd, N, sampling_time, 100);

    //Initializing Code
    wheel_driver.turnOnMotor(MotorDriver::BRAKETOGND);
    wheel_driver.setMotorOutput(0);
    motor_update(0, wheel_driver);

    sleep_ms(1000);
    printf("Initializing code...\n");
    sleep_ms(1000);
    printf("3\n");
    sleep_ms(1000);
    printf("2\n");
    sleep_ms(1000);
    printf("1\n");
    sleep_ms(1000);
    printf("Finalized \n");
    sleep_ms(1000);

    while (true)
{
    // 1. Update Sensor Readings
    actual_time = time_us_64();
    float dt = (float)(actual_time - prev_time) / 1000000.0f;
    encoder.update(dt);
    prev_time = actual_time;

    float actual_velocity = encoder.get_velocity();
    float motor_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;

    // 2. Update the shared data structure for the master
    // Convert floats to scaled integers for transmission
    sensor_data_to_master.velocity1 = (int16_t)(actual_velocity * DATA_SCALE_FACTOR);
    sensor_data_to_master.current1 = (int16_t)(motor_current * 1000.0f); // Send as mA
    // sensor_data_to_master.velocity2 = ... // Update for motor 2
    // sensor_data_to_master.current2 = ... // Update for motor 2

    // 3. Run the Control Logic
    // Use the target velocity set by the master via I2C
    float pwm_output = MotorA.controlCalcPI(target_velocity_1, actual_velocity);

    // 4. Update the Motor
    motor_update(pwm_output, wheel_driver);

    // 5. Loop Delay
    // Ensure a consistent loop rate for your PID controller
    sleep_ms(10); // e.g., for a 100 Hz control loop
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
