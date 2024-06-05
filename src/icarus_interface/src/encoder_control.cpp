#include "icarus_interface/encoder_control.h"

//Global varaibles
auto EncoderClock = std::make_shared<rclcpp::Node>("EncoderClock");
rclcpp::Time last_timeFL = EncoderClock->get_clock()->now();
rclcpp::Time last_timeBL = EncoderClock->get_clock()->now();
rclcpp::Time last_timeFR = EncoderClock->get_clock()->now();
rclcpp::Time last_timeBR = EncoderClock->get_clock()->now();
control_toolbox::Pid pidFL;
control_toolbox::Pid pidBL;
control_toolbox::Pid pidFR;
control_toolbox::Pid pidBR;
int desSpdFL_ = 0;  //TODO remove assignment and link to motor
int desSpdBL_ = 0;
int desSpdFR_ = 0;
int desSpdBR_ = 0;

void callbackFL(int currentPosition)
{   
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_timeFL).nanoseconds();
    static double effortFL = pidFL.computeCommand(desSpdFL_ - currentPosition, time_difference);
    last_timeFL = EncoderClock->get_clock()->now();
    fl_wheel_.enc = -currentPosition; //negative to fix
}

void callbackFR(int currentPosition)
{
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_timeFR).nanoseconds();
    static double effortFR = pidFR.computeCommand(desSpdFR_ - currentPosition, time_difference);
    last_timeFR = EncoderClock->get_clock()->now();
    fr_wheel_.enc = currentPosition;
}
void callbackBL(int currentPosition)
{
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_timeBL).nanoseconds();
    static double effortBL = pidBL.computeCommand(desSpdBL_ - currentPosition, time_difference);
    last_timeBL = EncoderClock->get_clock()->now();
    bl_wheel_.enc = -currentPosition;
}
void callbackBR(int currentPosition)
{
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_timeBR).nanoseconds();
    static double effortBR = pidBR.computeCommand(desSpdBR_ - currentPosition, time_difference);
    last_timeBR = EncoderClock->get_clock()->now();
    br_wheel_.enc = currentPosition;
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
    RED_set_glitch_filter(REDencFL, ENCODER_FL.OPTGLITCH);
    RED_set_glitch_filter(REDencFR, ENCODER_FR.OPTGLITCH);
    RED_set_glitch_filter(REDencBL, ENCODER_BL.OPTGLITCH);
    RED_set_glitch_filter(REDencBR, ENCODER_BR.OPTGLITCH);
    return 0;
}

encoder_control::~encoder_control()
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