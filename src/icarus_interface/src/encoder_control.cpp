#include "icarus_interface/encoder_control.h"

//Global varaibles
int currentLeftPosition = 0;
auto EncoderClock = std::make_shared<rclcpp::Node>("EncoderClock");
rclcpp::Time last_time = EncoderClock->get_clock()->now();
double effort;
int desiredPosition;
control_toolbox::Pid pidFL;
control_toolbox::Pid pidBL;
control_toolbox::Pid pidFR;
control_toolbox::Pid pidBR;

void callback1(int currentPosition)
{   
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_time).nanoseconds();
    effort = pid.computeCommand(desiredPosition - currentPosition, time_difference);
    last_time = EncoderClock->get_clock()->now();
    currentLeftPosition = currentPosition;
}

void callback2(int way)
{
   //static int pos = 0;

   //pos += way;
   //std::cout << "pos2=" << pos << std::endl;
   //RCLCPP_INFO(logger_, "pos2= %i \n", pos);
}

encoder_control::encoder_control()
    : logger_(rclcpp::get_logger("encoder_control"))
{}


int encoder_control::start_encoders()
{
    RCLCPP_INFO(logger_, ("----Starting encoders" ));

    pidFL.initPid(PROPORTIONAL_GAIN, INTERGRAL_GAIN, DERIVATIVE_GAIN, I_MAX, I_MIN, ANTIWINDUP);
    pidBL.initPid(PROPORTIONAL_GAIN, INTERGRAL_GAIN, DERIVATIVE_GAIN, I_MAX, I_MIN, ANTIWINDUP);
    pidFR.initPid(PROPORTIONAL_GAIN, INTERGRAL_GAIN, DERIVATIVE_GAIN, I_MAX, I_MIN, ANTIWINDUP);
    pidBR.initPid(PROPORTIONAL_GAIN, INTERGRAL_GAIN, DERIVATIVE_GAIN, I_MAX, I_MIN, ANTIWINDUP);
    
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize (encoder)" ));
        return 1;
    }
    else
        RCLCPP_INFO(logger_, ("----PiGPIO version::  %i" ), get_pigpio_version(pi_));


    renc = RED(pi_, optGpioA, optGpioB, optMode, callback1);
    RED_set_glitch_filter(renc, optGlitch);
    return 0;
}

int encoder_control::read_encoders(){
    int out = currentLeftPosition;
    return out;
}

encoder_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up encoder GPIO"));

    if (pi_ >= 0)
    {
        RED_cancel(renc);
        RCLCPP_INFO(logger_, ("----encoder GPIO Stopped"));
    }

}