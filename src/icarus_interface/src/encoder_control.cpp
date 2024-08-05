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

    double deltaSecondsFL = currentTimeFL - old_timeFL;
    //RCLCPP_INFO(EncoderClock->get_logger()," current time: %f , old time: %f delta time: %f", currentTimeFL, old_timeFL, deltaSecondsFL);
    //RCLCPP_INFO(EncoderClock->get_logger()," current position: %f , old position: %f delta position: %f", currentPositionFL, old_positionFL, deltaDistanceFL);
    old_timeFL = currentTimeFL;

    fl_wheel_.vel = -deltaDistanceFL/deltaSecondsFL;

    fl_wheel_.eff = pidFR.computeCommand(fl_wheel_.desired_speed - fl_wheel_.vel, deltaSecondsFL);

    fl_wheel_.enc = currentPosition; //negative to fix
}

void callbackFR(int currentPosition)
{   
    static double pos_prev = fr_wheel_.pos;

    pos_prev = fr_wheel_.pos;
    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    static auto deltaSeconds = (now_time - last_timeFR).nanoseconds();
    fr_wheel_.time_difference = (now_time - last_timeFR).nanoseconds();
    fr_wheel_.pos = fr_wheel_.calcEncAngle(currentPosition);
    fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;
    fr_wheel_.eff = pidFR.computeCommand(fr_wheel_.desired_speed - fr_wheel_.vel, fr_wheel_.time_difference);
    last_timeFR = EncoderClock->get_clock()->now();
    fr_wheel_.enc = -currentPosition;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now_time;
    msg.name = {"fr_wheel"};
    msg.position = {fr_wheel_.pos};
    msg.velocity = {fr_wheel_.vel};
    msg.effort = {fr_wheel_.eff};

    targetVelocityFR->publish(msg);
}
void callbackBL(int currentPosition)
{
    static double pos_prev = bl_wheel_.pos;

    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    bl_wheel_.time_difference = (now_time - last_timeBL).nanoseconds();
    auto deltaSeconds = (now_time - last_timeBL).nanoseconds();
    bl_wheel_.pos = bl_wheel_.calcEncAngle(currentPosition);
    bl_wheel_.vel = (bl_wheel_.pos - pos_prev) / deltaSeconds;
    bl_wheel_.eff = pidBL.computeCommand(bl_wheel_.desired_speed - bl_wheel_.vel, bl_wheel_.time_difference);
    last_timeBL = EncoderClock->get_clock()->now();
    bl_wheel_.enc = currentPosition;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now_time;
    msg.name = {"bl_wheel"};
    msg.position = {bl_wheel_.pos};
    msg.velocity = {bl_wheel_.vel};
    msg.effort = {bl_wheel_.eff};

    targetVelocityBL->publish(msg);
}
void callbackBR(int currentPosition)
{
    static double pos_prev = br_wheel_.pos;

    static rclcpp::Time now_time = EncoderClock->get_clock()->now();
    static auto deltaSeconds = (now_time - last_timeBR).nanoseconds();
    br_wheel_.time_difference = (now_time - last_timeBR).nanoseconds();
    br_wheel_.pos = br_wheel_.calcEncAngle(currentPosition);
    br_wheel_.vel = (br_wheel_.pos - pos_prev) / deltaSeconds;
    br_wheel_.eff = pidBR.computeCommand(br_wheel_.desired_speed - br_wheel_.vel, br_wheel_.time_difference);
    last_timeBR = EncoderClock->get_clock()->now();
    br_wheel_.enc = -currentPosition;

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now_time;
    msg.name = {"br_wheel"};
    msg.position = {br_wheel_.pos};
    msg.velocity = {br_wheel_.vel};
    msg.effort = {br_wheel_.eff};

    targetVelocityBR->publish(msg);
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