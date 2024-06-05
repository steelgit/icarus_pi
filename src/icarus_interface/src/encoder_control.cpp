#include "icarus_interface/encoder_control.h"

//Global varaibles
auto EncoderClock = std::make_shared<rclcpp::Node>("EncoderClock");
rclcpp::Time last_timeFL = EncoderClock->get_clock()->now();
control_toolbox::Pid pidFL;
control_toolbox::Pid pidBL;
control_toolbox::Pid pidFR;
control_toolbox::Pid pidBR;
int desSpdFL_ = 0;  //TODO remove assignment and link to motor

void callbackFL(int currentPosition)
{   
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_time).nanoseconds();
    static double effortFL = pid.computeCommand(desSpdFL_ - currentPosition, time_difference);
    last_timeFL = EncoderClock->get_clock()->now();
    fl_wheel_.enc= fl_wheel_.enc + currentPosition;
}

void callbackFR(int way)
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

    REDencFL = RED(pi_, ENCODER_FL.OPTGPIOA, ENCODER_FL.OPTGPIOB, optMode, callbackFL);
    REDencFR = RED(pi_, ENCODER_FR.OPTGPIOA, ENCODER_FR.OPTGPIOB, optMode, callbackFR);
    REDencBL = RED(pi_, ENCODER_BL.OPTGPIOA, ENCODER_BL.OPTGPIOB, optMode, callbackBL);
    REDencBR = RED(pi_, ENCODER_BR.OPTGPIOA, ENCODER_BR.OPTGPIOB, optMode, callbackBR);
    RED_set_glitch_filter(rencFL, ENCODER_FL.OPTGLITCH);
    RED_set_glitch_filter(rencFR, ENCODER_FR.OPTGLITCH);
    RED_set_glitch_filter(rencBL, ENCODER_BL.OPTGLITCH);
    RED_set_glitch_filter(rencBR, ENCODER_BR.OPTGLITCH);
    return 0;
}

encoder_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up encoder GPIO"));

    if (pi_ >= 0)
    {
        RED_cancel(REDencFL);
        RED_cancel(REDencFR);
        RED_cancel(REDencBL);
        RED_cancel(REDencBR);
        RCLCPP_INFO(logger_, ("----encoder GPIO Stopped"));
    }

}