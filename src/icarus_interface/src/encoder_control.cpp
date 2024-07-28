#include "icarus_interface/encoder_control.h"
#include "icarus_interface/motor_control.h"

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
double Error = 0;
double Proportional_error = 0;
double Integral_error = 0;
double Derivative_error = 0;




  //setup motor encoder
  //fl_wheel_.enc = enc_ctr.read_encoders();
  //RCLCPP_INFO(logger_, "  Read Encoder Values:  %f", val);


void callbackFL(int currentPosition)
{   
    double pos_prev = fl_wheel_.pos;
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto deltaSeconds = (now_time - last_timeFL).nanoseconds();
    auto time_difference = (now_time - last_timeFL).nanoseconds();
    fl_wheel_.pos = fl_wheel_.calcEncAngle();
    fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;
    static double effortFL = pidFL.computeCommand(fl_wheel_.desired_speed - fl_wheel_.vel, time_difference);
    last_timeFL = EncoderClock->get_clock()->now();
    fl_wheel_.enc = currentPosition; //negative to fix
    fl_wheel_.eff = effortFL;
}

void callbackFR(int currentPosition)
{   

    double pos_prev = fr_wheel_.pos;
    pos_prev = fr_wheel_.pos;
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto deltaSeconds = (now_time - last_timeFL).nanoseconds();
    auto time_difference = (now_time - last_timeFR).nanoseconds();
    fr_wheel_.pos = fr_wheel_.calcEncAngle();
    fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;
    static double effortFR = pidFR.computeCommand(fr_wheel_.desired_speed - fr_wheel_.vel, time_difference);
    last_timeFR = EncoderClock->get_clock()->now();
    fr_wheel_.enc = -currentPosition;
}
void callbackBL(int currentPosition)
{

    double pos_prev = bl_wheel_.pos;

    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_timeBL).nanoseconds();
    auto deltaSeconds = (now_time - last_timeFL).nanoseconds();
    bl_wheel_.pos = bl_wheel_.calcEncAngle();
    bl_wheel_.vel = (bl_wheel_.pos - pos_prev) / deltaSeconds;
    static double effortBL = pidBL.computeCommand(bl_wheel_.desired_speed - bl_wheel_.vel, time_difference);
    last_timeBL = EncoderClock->get_clock()->now();
    bl_wheel_.enc = currentPosition;
    bl_wheel_.eff = effortBL;
}
void callbackBR(int currentPosition)
{


    double pos_prev = br_wheel_.pos;

    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    auto deltaSeconds = (now_time - last_timeFL).nanoseconds();
    auto time_difference = (now_time - last_timeBR).nanoseconds();
    br_wheel_.pos = br_wheel_.calcEncAngle();
    br_wheel_.vel = (br_wheel_.pos - pos_prev) / deltaSeconds;
    static double effortBR = pidBR.computeCommand(br_wheel_.desired_speed - br_wheel_.vel, time_difference);
    last_timeBR = EncoderClock->get_clock()->now();
    br_wheel_.enc = -currentPosition;
    br_wheel_.eff = effortBR;
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