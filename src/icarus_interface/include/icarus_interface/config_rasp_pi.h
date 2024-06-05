#ifndef ICARUSINTERFACE_CONFIG_RASP_PI_H
#define ICARUSINTERFACE_CONFIG_RASP_PI_H    
#include "wheel.h"
//===============================================================
//  STRUCTS
//===============================================================
struct encoder{
    const uint8_t OPTGPIOA; //
    const uint8_t OPTGPIOB; //
    const uint8_t OPTGLITCH; //Constant
};

struct motor{
    const uint16_t PWM_MAX;
    const uint8_t ENA; //enableMotor Pin
    const uint8_t IN1; //forwardPin
    const uint8_t IN2; //backwardPin
    const uint8_t FREQ; //
    const uint16_t RANGE; //
};

//===============================================================
//  CONSTANTS
//===============================================================
//encoder PID value
const double PROPORTIONAL_GAIN = 5.0;
const double INTERGRAL_GAIN = .1;
const double DERIVATIVE_GAIN = .005;
const double I_MIN = 0.3;
const double I_MAX = -0.3;
const bool ANTIWINDUP = true;

// motor = {PWM_MAX, ENA, IN1, IN2, FREQ, RANGE}
const uint16_t PWM_MAX = 1000;
const uint8_t FREQ = 200;
const uint16_t RANGE = 1000;

const motor MOTOR_FL = {PWM_MAX, 21, 20, 16, FREQ, RANGE};
const motor MOTOR_FR = {PWM_MAX, 13, 6, 5, FREQ, RANGE};
const motor MOTOR_BL = {PWM_MAX, 2, 3, 4, FREQ, RANGE};
const motor MOTOR_BR = {PWM_MAX, 25, 24, 23, FREQ, RANGE};

// encoder = {OptA, OptB, Glitch}
const uint8_t OPTGLITCH_DEFAULT = 250;

const encoder ENCODER_FL = {7, 8, OPTGLITCH_DEFAULT};
const encoder ENCODER_FR = {27, 17, OPTGLITCH_DEFAULT};
const encoder ENCODER_BL = {12, 1, OPTGLITCH_DEFAULT};
const encoder ENCODER_BR = {26, 19, OPTGLITCH_DEFAULT};


//===============================================================
//  GLOBAL VARIABLES
//===============================================================
extern int pi_;

extern Wheel fl_wheel_;
extern Wheel fr_wheel_;
extern Wheel bl_wheel_;
extern Wheel br_wheel_;
extern int effortFL;

#endif // ICARUSINTERFACE_CONFIG_RASP_PI_H