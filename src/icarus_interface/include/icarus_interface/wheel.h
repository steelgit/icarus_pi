#ifndef ICARUSINTERFACE_WHEEL_H
#define ICARUSINTERFACE_WHEEL_H

#include <string>
#include "rclcpp/rclcpp.hpp"


class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;
    double desired_speed = 0;
    double pos_prev = 0;
    double curr_pwm = 0;
    double time_difference = 10;
    double error = 0;
    double oldError = 0;
    double integralError = 0;
    double oldIntegralError = 0;
    double proportionalGain = 0.025;
    double integralGain = 0;
    double derivativeGain = 0;



    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev);
    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calcEncAngle(int enc);

    double calculatePID(double desiredValue, double currentValue);



};


#endif // ICARUSINTERFACE_WHEEL_H
