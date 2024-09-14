#include "icarus_interface/Icarus_HW_Interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pid_messages/msg/pid.hpp"

//GLOBALS
int pi_;
Wheel fl_wheel_;
Wheel fr_wheel_;
Wheel bl_wheel_;
Wheel br_wheel_;

static rclcpp::Time now_time;

double deltaPositionFL = 0;
double deltaPositionFR = 0;
double deltaPositionBL = 0;
double deltaPositionBR = 0;

//create a node for tracking PID response
auto pidTracker = std::make_shared<rclcpp::Node>("pidTracker");
auto pidTrackingFl =  pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
auto pidTrackingFr =  pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
auto pidTrackingBl =  pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);
auto pidTrackingBr =  pidTracker->create_publisher<pid_messages::msg::Pid>("pidTracker", 10);

IcarusInterface::IcarusInterface()
    : logger_(rclcpp::get_logger("IcarusInterface")),
    currentTime_(rclcpp::Clock().now()),
    previousTime_(currentTime_)
{}

IcarusInterface::~IcarusInterface()
{
}

return_type IcarusInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  //taken from URDF information published by robot_satte_publisher
  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Set up the wheels
  fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  pi_ = pigpio_start(OPTHOST, OPTPORT);
    if (pi_ < 0) 
    {
        RCLCPP_INFO(logger_, ("----gpio failed to initialize (motor)" ));
        return return_type::ERROR;
    }


  //setup motor_control
  motor_ctr.start_motors();
  enc_ctr.start_encoders();


  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> IcarusInterface::export_state_interfaces()
{
  //set up state interfaces to monitor wheel velocity and position
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //hardware_interface::StateInterface(name, interface type, value)
  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_POSITION, &bl_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_POSITION, &br_wheel_.pos));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IcarusInterface::export_command_interfaces()
{
  //set up a command interface to set desired velocity for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  //hardware_interface::CommandInterface(name, interface type, value)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}


return_type IcarusInterface::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type IcarusInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;
  
  return return_type::OK;
}

hardware_interface::return_type IcarusInterface::read()
{
  return return_type::OK;
}

hardware_interface::return_type IcarusInterface::write()
{

  if (1==0)  //check connection
  {
    return return_type::ERROR;
  }
  
  //track position for each wheel
  //current position is already tracked by encoder control file
  fl_wheel_.pos = fl_wheel_.calcEncAngle(fl_wheel_.enc);
  deltaPositionFL = fl_wheel_.pos - fl_wheel_.pos_prev;
  fl_wheel_.pos_prev = fl_wheel_.pos;

  fr_wheel_.pos = fr_wheel_.calcEncAngle(fr_wheel_.enc);
  deltaPositionFR = fr_wheel_.pos - fr_wheel_.pos_prev;
  fr_wheel_.pos_prev = fr_wheel_.pos;

  bl_wheel_.pos = bl_wheel_.calcEncAngle(bl_wheel_.enc);
  deltaPositionBL = bl_wheel_.pos - bl_wheel_.pos_prev;
  bl_wheel_.pos_prev = bl_wheel_.pos;

  br_wheel_.pos = br_wheel_.calcEncAngle(br_wheel_.enc);
  deltaPositionBR = br_wheel_.pos - br_wheel_.pos_prev;
  br_wheel_.pos_prev = br_wheel_.pos;

  //track elapsed time between measurements
  /*currentTime_ = rclcpp::Clock().now();
  fr_wheel_.time_difference = (currentTime_ - previousTime_).seconds();
  fl_wheel_.time_difference = (currentTime_ - previousTime_).seconds();
  br_wheel_.time_difference = (currentTime_ - previousTime_).seconds();
  bl_wheel_.time_difference = (currentTime_ - previousTime_).seconds();
  previousTime_ = currentTime_;*/

  //track velocity for each wheel
  fl_wheel_.vel = deltaPositionFL/ fl_wheel_.time_difference;
  fr_wheel_.vel = deltaPositionFR/ fr_wheel_.time_difference;
  bl_wheel_.vel = deltaPositionBL/ bl_wheel_.time_difference;
  br_wheel_.vel = deltaPositionBR/ br_wheel_.time_difference;

  //constrain our desired speed to be between two values
  fl_wheel_.desired_speed = std::clamp( fl_wheel_.cmd , -10.0 , 10.0);
  bl_wheel_.desired_speed = std::clamp( bl_wheel_.cmd , -10.0 , 10.0);
  fr_wheel_.desired_speed = std::clamp( fr_wheel_.cmd , -10.0 , 10.0);
  br_wheel_.desired_speed = std::clamp( br_wheel_.cmd , -10.0 , 10.0);

  //calcualte the effort needed to reach desired speed
  fl_wheel_.eff = fl_wheel_.calculatePID(fl_wheel_.desired_speed,fl_wheel_.vel);
  bl_wheel_.eff = bl_wheel_.calculatePID(bl_wheel_.desired_speed,bl_wheel_.vel);
  fr_wheel_.eff = fr_wheel_.calculatePID(fr_wheel_.desired_speed,fr_wheel_.vel);
  br_wheel_.eff = br_wheel_.calculatePID(br_wheel_.desired_speed,br_wheel_.vel);
  
  //create topics to track and tune PID response
  auto msgFl = pid_messages::msg::Pid();
  msgFl.header.stamp = currentTime_;
  msgFl.name = {"fl_wheel"};
  msgFl.desired_value = {fl_wheel_.desired_speed};
  msgFl.measured_value = {fl_wheel_.vel};
  msgFl.effort = {fl_wheel_.eff};
  
  auto msgFr = pid_messages::msg::Pid();
  msgFr.header.stamp = currentTime_;
  msgFr.name = {"fr_wheel"};
  msgFr.desired_value = {fr_wheel_.desired_speed};
  msgFr.measured_value = {fr_wheel_.vel};
  msgFr.effort = {fr_wheel_.eff};

  auto msgBl = pid_messages::msg::Pid();
  msgBl.header.stamp = currentTime_;
  msgBl.name = {"bl_wheel"};
  msgBl.desired_value = {bl_wheel_.desired_speed};
  msgBl.measured_value = {bl_wheel_.vel};
  msgBl.effort = {bl_wheel_.eff};

  auto msgBr = pid_messages::msg::Pid();
  msgBr.header.stamp = currentTime_;
  msgBr.name = {"br_wheel"};
  msgBr.desired_value = {br_wheel_.desired_speed};
  msgBr.measured_value = {br_wheel_.vel};
  msgBr.effort = {br_wheel_.eff};

  //publish topic messages
  pidTrackingFl->publish(msgFl);
  pidTrackingFr->publish(msgFr);
  pidTrackingBl->publish(msgBl);
  pidTrackingBr->publish(msgBr);

  //apply PID calculations to motor pwm
  fl_wheel_.curr_pwm += fl_wheel_.eff; 
  motor_ctr.setMotor(fl_wheel_.curr_pwm, MOTOR_FL); 
  
  bl_wheel_.curr_pwm += bl_wheel_.eff;
  motor_ctr.setMotor((bl_wheel_.curr_pwm), MOTOR_BL);

  fr_wheel_.curr_pwm += fr_wheel_.eff;
  motor_ctr.setMotor((fr_wheel_.curr_pwm), MOTOR_FR);  
  
  br_wheel_.curr_pwm += br_wheel_.eff;
  motor_ctr.setMotor((br_wheel_.curr_pwm), MOTOR_BR);
  
  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  IcarusInterface,
  hardware_interface::SystemInterface
) 
