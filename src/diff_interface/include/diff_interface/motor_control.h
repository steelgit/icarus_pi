#ifndef DIFFDRIVE_MYBOT_MOTOR_CONTROL_H
#define DIFFDRIVE_MYBOT_MOTOR_CONTROL_H

#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

using namespace std;

#include "diff_interface/rotary_encoder.h"

struct motor{
    const uint8_t PWM_MAX;
    const uint8_t ENA; //enableMotor Pin
    const uint8_t IN1; //forwardPin
    const uint8_t IN2; //backwardPin
    const uint8_t FREQ; //
    const uint8_t RANGE; //
};


class motor_control
{
    public:
        motor_control();
        ~motor_control();

        int start_encoders();
        int stop_encoders();
        int read_encoders();
        void motor_config(motor m);
        
        int start_motors(motor FL, motor BL, motor FR, motor BR);
        void setMotorMode(const string &mode, motor m);
        void setMotor(const double &power, motor m); 

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
      

};




#endif // DIFFDRIVE_MYBOT_MOTOR_CONTROL_H
