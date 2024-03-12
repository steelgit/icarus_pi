#include "diff_interface/motor_control.h"

motor_control::motor_control()
    : logger_(rclcpp::get_logger("motor_control"))
{}

//Global varaibles
int currentLeftPosition = 0;
auto EncoderClock = std::make_shared<rclcpp::Node>("EncoderClock");
rclcpp::Time now_time;
rclcpp::Time last_time = EncoderClock->get_clock()->now();
control_toolbox::Pid pid;
double effort;

void callback1(int currentPosition)
{   
    now_time = EncoderClock->get_clock()->now();
    auto time_difference = (now_time - last_time).nanoseconds();
    effort = pid.computeCommand(desiredPosition - currentPosition, time_difference);
    last_time = EncoderClock->get_clock()->now();
    currentLeftPosition = currentPosition;

    //RCLCPP_INFO(rclcpp::get_logger("encoder_control"), "effort= %f        currentPosition= %i \n", effort, currentPosition);

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
    RCLCPP_INFO(logger_, ("----Starting encoders" ));
    //pi_ = pigpio_start(optHost, optPort);

    pid.initPid(PROPORTIONAL_GAIN, INTERGRAL_GAIN, DERIVATIVE_GAIN, I_MAX, I_MIN, ANTIWINDUP);
    
    
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

int motor_control::start_motors(motor FL, motor BL, motor FR, motor BR){
    RCLCPP_INFO(logger_, ("----Starting motors" ));
    pi_ = pigpio_start(optHost, optPort);
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize (motor)" ));
        return 1;
    }

    motor_config(FL);
    motor_config(BL);
    motor_config(FR);
    motor_config(BR);

    return 0;
}

void motor_control::motor_config(motor m){
    set_mode(pi_,m.ENA, PI_OUTPUT);
    set_mode(pi_,m.IN1, PI_OUTPUT);
    set_mode(pi_,m.IN2, PI_OUTPUT);
    gpio_write(pi_,m.IN1, PI_LOW);
    gpio_write(pi_,m.IN2, PI_LOW);
    set_PWM_frequency(pi_,m.ENA, m.FREQ);
    set_PWM_range(pi_,m.ENA, m.RANGE);
    set_PWM_dutycycle(pi_,m.ENA, 0);
    RCLCPP_INFO(logger_, ("ENA: %i, IN1: %i, IN2: %i, FREQ: %i, RANGE: %i"), m.ENA,m.IN1,m.IN2,m.FREQ,m.RANGE);
}

void motor_control::setMotorMode(const string &mode, motor m) {
    //RCLCPP_INFO(logger_, ("set motor mode to %s \n"), mode.c_str());
    if(mode == "forward") {
        gpio_write(pi_, m.IN1, PI_LOW);
        gpio_write(pi_, m.IN2, PI_HIGH);
    } else if(mode == "reverse") {
        gpio_write(pi_, m.IN1, PI_HIGH);
        gpio_write(pi_, m.IN2, PI_LOW);
    } else {
        gpio_write(pi_, m.IN1, PI_LOW);
        gpio_write(pi_, m.IN2, PI_LOW);
    }
}

void motor_control::setMotor(const double &power, motor m) {
    uint16_t pwm;  //was uint8.  keep???
    if(power > 10) {
        setMotorMode("forward", m);
        pwm = (int)(power);
    } else if(power < -10) {
        setMotorMode("reverse", m);
        pwm = -(int)(power);
    } else {
        setMotorMode("stop", m);
        pwm = 0;
    }

    if(pwm > m.PWM_MAX) {
        pwm = m.PWM_MAX;
    }

    //RCLCPP_INFO(logger_, ("set pwm to %d \n"), pwm);
    set_PWM_dutycycle(pi_,m.ENA, pwm);
}

int motor_control::read_encoders(){
    int out = currentLeftPosition;
    return out;
}

motor_control::~motor_control()
{
    RCLCPP_INFO(logger_, ("----Cleaning up GPIO"));

    if (pi_ >= 0)
    {
        RED_cancel(renc);
        pigpio_stop(pi_);
        RCLCPP_INFO(logger_, ("----GPIO Stopped"));
    }

}

