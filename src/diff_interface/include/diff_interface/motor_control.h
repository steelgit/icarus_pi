#ifndef DIFFDRIVE_MYBOT_MOTOR_CONTROL_H
#define DIFFDRIVE_MYBOT_MOTOR_CONTROL_H

#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

using namespace std;

#include "diff_interface/rotary_encoder.h"

class motor_control
{
    public:
        motor_control();
        ~motor_control();

        int start_encoders();
        int stop_encoders();
        int read_encoders();
        
        int start_motors();
        void setMotorMode(const string &mode);
        void setMotor(const double &power); 

    private:

        rclcpp::Logger logger_;
        int pi_;
        char *optHost   = NULL;
        char *optPort   = NULL;

        RED_t *renc;
        int optGpioA = 2;
        int optGpioB = 3;
        int optGlitch = 1000;
        int optMode = RED_MODE_DETENT;

        const uint8_t PWM_MAX = 240;
        const uint8_t ENA = 21; // Pin 40   //enableMotor Pin
        const uint8_t IN1 = 20; // Pin 38    //forwardPin
        const uint8_t IN2 = 16; // Pin 36   //backwardPin
        const uint8_t FREQ = 60; //
        const uint8_t RANGE = 255; //

};




#endif // DIFFDRIVE_MYBOT_MOTOR_CONTROL_H
