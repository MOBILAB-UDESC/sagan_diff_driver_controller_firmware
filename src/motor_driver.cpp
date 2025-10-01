#include "motor_driver.hpp"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

MotorDriver::MotorDriver( 
                        uint8_t en, 
                        uint8_t cs, 
                        uint8_t inA,
                        uint8_t inB, 
                        uint8_t pwm, 
                        DriverConfiguration mode )
{ 
    this->en = en;
    this->cs = cs;
    this->inA = inA;
    this->inB = inB;
    this->pwm = pwm;
    this->mode = mode;
    this->state = BRAKETOGND;

    
    // Config output A
    gpio_init(this->inA);
    gpio_set_dir(this->inA, GPIO_OUT);
    gpio_put(this->inA, false);

    // Config output B
    gpio_init(this->inB);
    gpio_set_dir(this->inB, GPIO_OUT);
    if ( mode == FULLBRIDGE )
    {
        gpio_put(this->inB, false);
    }
    

    gpio_init(this->en);
    gpio_set_dir(this->en, GPIO_IN);

    // Config PWM
    gpio_set_function(this->pwm, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(this->pwm);
    // Motor Driver VNH2SP30 has maximum frequency of 20kHz
    // Set PWM frequency to 480kHz/25 = 19.2kHz 
    pwm_set_clkdiv(slice_num, 25); 
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(this->pwm), 1);
    pwm_set_enabled(slice_num, true);
    
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc/
    // cs number should only be from 26 to 29
    adc_gpio_init(this->cs);
    // Select ADC input
    adc_select_input(this->cs-26);
}


// Change the driver configuration (HALFBRIDGE OR FULLBRIDGE)
void MotorDriver::changeBridgeMode( DriverConfiguration mode )
{
    if ( this->state &&        // if no fault, i.e. is true and
         this->mode != mode )  // different mode specified
                               
    {
        MotorState current_state = this->state;   // save current state
        this->turnOffMotor();                     // turn off motor
        this->mode = mode;                        // update mode
        this->turnOnMotor( current_state );
    }

    // otherwise, do nothing
}

// Check motor driver status
// Returns: true if fault is detected (shutdown), false if none 
bool MotorDriver::motorDriverIsFaulty()
{
    // en pin is already pulling HIGH in hardware
    // but if a fault causes shutdown,
    // this pin will output LOW
    return !gpio_get(this->en);
}


// Check motor current draw
// Returns: current draw value mapped from 0 to 1024
float MotorDriver::checkMotorCurrentDraw()
{
    adc_select_input(this->cs-26);
    const float conversion_factor = 3.3f / 4096;
    float result = conversion_factor * (float)adc_read();
    return result;
}

// Turn off motor
// Returns: nothing
void MotorDriver::turnOffMotor()
{
    gpio_put(this->inA, false);
    
    if ( this->mode == FULLBRIDGE )
    {
        gpio_put(this->inB, false);
    }
    
    uint slice_num = pwm_gpio_to_slice_num(this->pwm);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(this->pwm), 0);

    this->state = BRAKETOGND;
}

// Make motor operate in any of four states (two if HALFBRIDGE MODE)
// If returns true, the chosen state is saved to instance
// Precondition: state < CLOCKWISE || state > BRAKETOGND
//               mode = HALFBRIDGE || FULLBRIDGE
// Parameter: state - one of four(two) states - see enum motorState above
// Returns: true if operation is successful, false if state is incompatible
bool MotorDriver::turnOnMotor( MotorState state )
{    
    if ( this->mode == HALFBRIDGE )
    {
        if ( state == CLOCKWISE )
        {
            gpio_put(this->inA, true);
        }
            
        else if ( state == BRAKETOGND )
        {
            gpio_put(this->inA, false);
        }

        else
        {
            return false;
        }
    }

    else // if mode == FULLBRIDGE
    {
        ( state == CLOCKWISE || state == BRAKETOVCC ) ? 
            gpio_put(this->inA, true): 
            gpio_put(this->inA, false);

        ( state == COUNTERCLOCKWISE || state == BRAKETOVCC )?
            gpio_put(this->inB, true): 
            gpio_put(this->inB, false);
    }

    this->state = state;
    return true;
}

void MotorDriver::setMotorOutput(uint16_t duty_cycle)
{
    uint slice_num = pwm_gpio_to_slice_num(this->pwm);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(this->pwm),  duty_cycle);
}

// Check operating state of Motor Driver
MotorDriver::MotorState MotorDriver::getMotorState()
{
    return this->state;
}

