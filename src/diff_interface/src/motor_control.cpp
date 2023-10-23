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
    //pi_ = pigpio_start(optHost, optPort);
    
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

int motor_control::start_motors(){
    RCLCPP_INFO(logger_, ("----Starting motors" ));
    pi_ = pigpio_start(optHost, optPort);
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize (motor)" ));
        return 1;
    }
    set_mode(pi_,ENA, PI_OUTPUT);
    set_mode(pi_,IN1, PI_OUTPUT);
    set_mode(pi_,IN2, PI_OUTPUT);
    gpio_write(pi_,IN1, PI_LOW);
    gpio_write(pi_,IN2, PI_LOW);
    set_PWM_frequency(pi_,ENA, FREQ);
    set_PWM_range(pi_,ENA, RANGE);
    set_PWM_dutycycle(pi_,ENA, 0);

    return 0;
}

void motor_control::setMotorMode(const string &mode) {
    //RCLCPP_INFO(logger_, ("set motor mode to %s \n"), mode.c_str());
    if(mode == "forward") {
        gpio_write(pi_, IN1, PI_LOW);
        gpio_write(pi_, IN2, PI_HIGH);
    } else if(mode == "reverse") {
        gpio_write(pi_, IN1, PI_HIGH);
        gpio_write(pi_, IN2, PI_LOW);
    } else {
        gpio_write(pi_, IN1, PI_LOW);
        gpio_write(pi_, IN2, PI_LOW);
    }
}

void motor_control::setMotor(const double &power) {
    uint16_t pwm;  //was uint8.  keep???
    if(power > 5) {
        setMotorMode("forward");
        pwm = (int)(power);
    } else if(power < -5) {
        setMotorMode("reverse");
        pwm = -(int)(power);
    } else {
        setMotorMode("stop");
        pwm = 0;
    }

    if(pwm > PWM_MAX) {
        pwm = PWM_MAX;
    }

    //RCLCPP_INFO(logger_, ("set pwm to %d \n"), pwm);
    set_PWM_dutycycle(pi_,ENA, pwm);
}

int motor_control::read_encoders(){
    return pos1;
}

motor_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up GPIO"));

    if (pi_ >= 0)
    {
        RED_cancel(renc);
        pigpio_stop(pi_);
    }

}

