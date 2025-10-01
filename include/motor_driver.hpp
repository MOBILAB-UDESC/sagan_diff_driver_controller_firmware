#ifndef MOTORDRIVER_HPP
#define MOTORDRIVER_HPP

#include "pico/stdlib.h"

class MotorDriver
{
public:

    // Two modes as defined in VNH2SP30 datasheet. 
    typedef enum driverConfiguration
    {
        HALFBRIDGE,
        FULLBRIDGE
    } DriverConfiguration;

    // Four states as defined in VNH2SP30 datasheet.
    // If configured as half bridge, the only available
    // modes are CLOCKWISE and BRAKETOGND  
    typedef enum motorState
    {
        CLOCKWISE,
        COUNTERCLOCKWISE,
        BRAKETOVCC,
        BRAKETOGND  // this will be the default case
    } MotorState;


    //------------------------------------------------------------------------------
    // function declarations
    //------------------------------------------------------------------------------

    MotorDriver(
        uint8_t en, uint8_t cs, uint8_t inA, 
        uint8_t inB, uint8_t pwm, DriverConfiguration mode );

    ~MotorDriver() = default;

    void changeBridgeMode( DriverConfiguration mode );

    bool motorDriverIsFaulty();

    float checkMotorCurrentDraw();

    void turnOffMotor();

    bool turnOnMotor( MotorState state );

    void setMotorOutput(uint16_t duty_cycle);

    MotorState getMotorState();

private:

    uint8_t en;                 // enable input/output (ANALOG)
    uint8_t cs;                 // current sense output (ANALOG)
    uint8_t inA;                // clockwise input
    uint8_t inB;                // counterclockwise input
    uint8_t pwm;                // pwm input
    DriverConfiguration mode;   // HALFBRIDGE or FULLBRIDGE
    MotorState state;           // state of connected motor

};

#endif // MOTORDRIVER_HPP