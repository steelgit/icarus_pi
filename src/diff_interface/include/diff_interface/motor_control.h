#ifndef DIFFDRIVE_MYBOT_MOTOR_CONTROL_H
#define DIFFDRIVE_MYBOT_MOTOR_CONTROL_H

#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <pigpiod_if2.h>

#include "diff_interface/rotary_encoder.h"

class motor_control
{
    public:
        motor_control();
        ~motor_control();

        int start_encoders();
        int stop_encoders();
        int read_encoders();

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
