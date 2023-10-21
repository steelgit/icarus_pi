#include "diff_interface/motor_control.h"

motor_control::motor_control()
    : logger_(rclcpp::get_logger("motor_control"))
{}

static int pos1 = 0;

void callback1(int way)
{
   //static int pos = 0;

   pos1 += way;
   //std::cout << "pos=" << pos << std::endl;
   
}

void callback2(int way)
{
   //static int pos = 0;

   //pos += way;
   //std::cout << "pos2=" << pos << std::endl;
   //RCLCPP_INFO(logger_, "pos2= %i \n", pos);
}

int motor_control::start_encoders()
{
    //gpioInitialise();
    //gpioTerminate();
    RCLCPP_INFO(logger_, ("----Starting encoders" ));
    pi_ = pigpio_start(optHost, optPort);
    
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize" ));
        return 1;
    }
    else
        RCLCPP_INFO(logger_, ("----PiGPIO version::  %i" ), get_pigpio_version(pi_));


    renc = RED(pi_, optGpioA, optGpioB, optMode, callback1);
    RED_set_glitch_filter(renc, optGlitch);

    return 0;
}

int motor_control::read_encoders(){
    return pos1;
}



motor_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up encoders"));

    if (pi_ >= 0)
    {
        RED_cancel(renc);
        pigpio_stop(pi_);
    }


}

