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

auto targetVelocityFL = EncoderClock->create_publisher<sensor_msgs::msg::JointState>("encoder_fl", 10);
auto targetVelocityBL = EncoderClock->create_publisher<sensor_msgs::msg::JointState>("encoder_bl", 10);
auto targetVelocityFR = EncoderClock->create_publisher<sensor_msgs::msg::JointState>("encoder_fr", 10);
auto targetVelocityBR = EncoderClock->create_publisher<sensor_msgs::msg::JointState>("encoder_br", 10);
  //setup motor encoder
  //fl_wheel_.enc = enc_ctr.read_encoders();
  //RCLCPP_INFO(logger_, "  Read Encoder Values:  %f", val);



void callbackFL(int currentPosition)
{
    static double old_positionFL;
    static double old_timeFL;
    double currentTimeFL;
    double currentPositionFL;
    rclcpp::Time currentTime = EncoderClock->get_clock()->now();
    currentTimeFL = currentTime.seconds();

    fl_wheel_.pos = fl_wheel_.calcEncAngle(currentPosition);
    currentPositionFL = fl_wheel_.calcEncAngle(currentPosition);
    double deltaDistanceFL =currentPositionFL - old_positionFL;
    old_positionFL = currentPositionFL;

    fl_wheel_.time_difference = currentTimeFL - old_timeFL;
    //RCLCPP_INFO(EncoderClock->get_logger()," current time: %f , old time: %f delta time: %f", currentTimeFL, old_timeFL, fl_wheel_.time_difference);
    //RCLCPP_INFO(EncoderClock->get_logger()," current position: %f , old position: %f delta position: %f", currentPositionFL, old_positionFL, deltaDistanceFL);
    old_timeFL = currentTimeFL;

    fl_wheel_.vel = -deltaDistanceFL/fl_wheel_.time_difference;

    fl_wheel_.eff = pidFR.computeCommand(fl_wheel_.desired_speed - fl_wheel_.vel, fl_wheel_.time_difference);
    RCLCPP_INFO(EncoderClock ->get_logger()," current effort: %f", fl_wheel_.eff);

    fl_wheel_.enc = currentPosition; //negative to fix
}

void callbackFR(int currentPosition)
{   
    static double old_positionFR;
    static double old_timeFR;
    double currentTimeFR;
    double currentPositionFR;
    rclcpp::Time currentTime = EncoderClock->get_clock()->now();
    currentTimeFR = currentTime.seconds();

    fr_wheel_.pos = fr_wheel_.calcEncAngle(currentPosition);
    currentPositionFR = fr_wheel_.calcEncAngle(currentPosition);
    double deltaDistanceFR =currentPositionFR - old_positionFR;
    old_positionFR = currentPositionFR;

    fr_wheel_.time_difference = currentTimeFR - old_timeFR;
    //RCLCPP_INFO(EncoderClock->get_logger()," current time: %f , old time: %f delta time: %f", currentTimeFR, old_timeFR, fl_wheel_.time_difference);
    //RCLCPP_INFO(EncoderClock->get_logger()," current position: %f , old position: %f delta position: %f", currentPositionFR, old_positionFR, deltaDistanceFR);
    old_timeFR = currentTimeFR;

    fr_wheel_.vel = deltaDistanceFR/fr_wheel_.time_difference;

    fr_wheel_.eff = pidFR.computeCommand(fr_wheel_.desired_speed - fr_wheel_.vel, fr_wheel_.time_difference);

    fr_wheel_.enc = currentPosition; //negative to fix
    fr_wheel_.eff = 0;
}

void callbackBL(int currentPosition)
{
static double old_positionBL;
static double old_timeBL;
double currentTimeBL;
double currentPositionBL;
rclcpp::Time currentTime = EncoderClock->get_clock()->now();
currentTimeBL = currentTime.seconds();

bl_wheel_.pos = bl_wheel_.calcEncAngle(currentPosition);
currentPositionBL = bl_wheel_.calcEncAngle(currentPosition);
double deltaDistanceBL = currentPositionBL - old_positionBL;
old_positionBL = currentPositionBL;

bl_wheel_.time_difference = currentTimeBL - old_timeBL;
//RCLCPP_INFO(EncoderClock->get_logger()," current time: %f , old time: %f delta time: %f", currentTimeBL, old_timeBL, fl_wheel_.time_difference);
//RCLCPP_INFO(EncoderClock->get_logger()," current position: %f , old position: %f delta position: %f", currentPositionBL, old_positionBL, deltaDistanceBL);
old_timeBL = currentTimeBL;

bl_wheel_.vel = -deltaDistanceBL / bl_wheel_.time_difference;

bl_wheel_.eff = pidBL.computeCommand(bl_wheel_.desired_speed - bl_wheel_.vel, bl_wheel_.time_difference);

bl_wheel_.enc = currentPosition; //negative to fix
bl_wheel_.eff = 0;

}

void callbackBR(int currentPosition)
{
static double old_positionBR;
static double old_timeBR;
double currentTimeBR;
double currentPositionBR;
rclcpp::Time currentTime = EncoderClock->get_clock()->now();
currentTimeBR = currentTime.seconds();

br_wheel_.pos = br_wheel_.calcEncAngle(currentPosition);
currentPositionBR = br_wheel_.calcEncAngle(currentPosition);
double deltaDistanceBR = currentPositionBR - old_positionBR;
old_positionBR = currentPositionBR;

br_wheel_.time_difference = currentTimeBR - old_timeBR;
//RCLCPP_INFO(EncoderClock->get_logger()," current time: %f , old time: %f delta time: %f", currentTimeBR, old_timeBR, fl_wheel_.time_difference);
//RCLCPP_INFO(EncoderClock->get_logger()," current position: %f , old position: %f delta position: %f", currentPositionBR, old_positionBR, deltaDistanceBR);
old_timeBR = currentTimeBR;

br_wheel_.vel = deltaDistanceBR / br_wheel_.time_difference;

br_wheel_.eff = pidBR.computeCommand(br_wheel_.desired_speed - br_wheel_.vel, br_wheel_.time_difference);

br_wheel_.enc = currentPosition; //negative to fix
br_wheel_.eff = 0;

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