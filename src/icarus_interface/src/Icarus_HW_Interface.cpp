#include "icarus_interface/Icarus_HW_Interface.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

//GLOBALS
int pi_;
Wheel fl_wheel_;
Wheel fr_wheel_;
Wheel bl_wheel_;
Wheel br_wheel_;

static rclcpp::Time now_time;

static double oldPositionFL = 0;
double deltaPosition = 0;

IcarusInterface::IcarusInterface()
    : logger_(rclcpp::get_logger("IcarusInterface")),
    currentTime_(rclcpp::Clock().now()),
    previousTime_(currentTime_)
{}

IcarusInterface::~IcarusInterface()
{
  /*
  for(unsigned long int iter = 1; iter < debug.size(); iter=iter+2) 
  { 
    double input = debug[iter];
    double output = debug[iter-1];
    RCLCPP_INFO(logger_, "  Speed - input:  %f    output: %f", input, output);
  }
  */
}

return_type IcarusInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

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
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

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
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

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

  // Calculate time delta
 /*  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  //setup motor encoder
  //fl_wheel_.enc = enc_ctr.read_encoders();
  //RCLCPP_INFO(logger_, "  Read Encoder Values:  %f", val);

  double pos_prev = fl_wheel_.pos;
  fl_wheel_.pos = fl_wheel_.calcEncAngle();
  fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = bl_wheel_.pos;
  bl_wheel_.pos = bl_wheel_.calcEncAngle();
  bl_wheel_.vel = (bl_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = fr_wheel_.pos;
  fr_wheel_.pos = fr_wheel_.calcEncAngle();
  fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = br_wheel_.pos;
  br_wheel_.pos = br_wheel_.calcEncAngle();
  br_wheel_.vel = (br_wheel_.pos - pos_prev) / deltaSeconds;
 */
  //debug.push_back(fl_wheel_.vel);
  //debug.push_back(fl_wheel_.cmd);
  //debug.push_back(fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate);
  //debug.push_back(fl_wheel_.enc);
  

  return return_type::OK;

  
}

hardware_interface::return_type IcarusInterface::write()
{

  if (1==0)  //check connection
  {
    return return_type::ERROR;
  }


  //RCLCPP_INFO(logger_, "  Front Left motor velocity:  %f", fl_wheel_.vel);
  RCLCPP_INFO(logger_, "  back Left motor pos:  %f, front Left motor pos:  %f, back right motor pos: %f, front right motor pos:  %f ", bl_wheel_.pos,fl_wheel_.pos, br_wheel_.pos, fr_wheel_.pos);
  RCLCPP_INFO(logger_, "  back Left motor vel:  %f, front Left motor vel:  %f, back right motor vel: %f, front right motor vel:  %f ", bl_wheel_.vel,fl_wheel_.vel, br_wheel_.vel, fr_wheel_.vel);
  //RCLCPP_INFO(logger_, "  back Left motor dsp:  %f, front Left motor dsp:  %f, back right motor dsp: %f, front right motor dsp:  %f ", bl_wheel_.desired_speed,fl_wheel_.desired_speed, br_wheel_.desired_speed, fr_wheel_.desired_speed);
  RCLCPP_INFO(logger_, "  back Left motor eff:  %f, front Left motor eff:  %f, back right motor eff: %f, front right motor eff:  %f ", bl_wheel_.eff,fl_wheel_.eff, br_wheel_.eff, fr_wheel_.eff);
  RCLCPP_INFO(logger_, "  back Left motor pwm:  %f, front Left motor pwm:  %f, back right motor pwm: %f, front right motor pwm:  %f ", bl_wheel_.curr_pwm,fl_wheel_.curr_pwm, br_wheel_.curr_pwm, fr_wheel_.curr_pwm);
  //RCLCPP_INFO(logger_, "  Front Left motor rads per count %f, Front left motor loop rate %f", fl_wheel_.rads_per_count, cfg_.loop_rate);


  //track position
  //current position is already traked by encoder cojntrol file
  deltaPosition = fl_wheel_.pos - oldPositionFL;
  oldPositionFL = fl_wheel_.pos;

  //track Time
  currentTime_ = rclcpp::Clock().now();
  fl_wheel_.time_difference = (currentTime_ - previousTime_).seconds();
  previousTime_ = currentTime_;


  //track velocity
  fl_wheel_.vel = deltaPosition/fl_wheel_.time_difference;


  //wire to motors
  fl_wheel_.desired_speed = fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate;
  bl_wheel_.desired_speed = bl_wheel_.cmd / bl_wheel_.rads_per_count / cfg_.loop_rate;
  fr_wheel_.desired_speed = fr_wheel_.cmd / fr_wheel_.rads_per_count / cfg_.loop_rate;
  br_wheel_.desired_speed = br_wheel_.cmd / br_wheel_.rads_per_count / cfg_.loop_rate;

  fl_wheel_.eff = pidFL.computeCommand(fl_wheel_.desired_speed - fl_wheel_.vel, fl_wheel_.time_difference);
  bl_wheel_.eff = pidBL.computeCommand(bl_wheel_.desired_speed - bl_wheel_.vel, bl_wheel_.time_difference);
  fr_wheel_.eff = pidFR.computeCommand(fr_wheel_.desired_speed - fr_wheel_.vel, fr_wheel_.time_difference);
  br_wheel_.eff = pidBR.computeCommand(br_wheel_.desired_speed - br_wheel_.vel, br_wheel_.time_difference);
  
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now_time;
  msg.name = {"fl_wheel"};
  msg.position = {fl_wheel_.pos};
  msg.velocity = {fl_wheel_.vel};
  msg.effort = {fl_wheel_.eff};

  motor_ctr.setMotor(fl_wheel_.curr_pwm, MOTOR_FL); 
  fl_wheel_.curr_pwm += fl_wheel_.eff; 
  
  motor_ctr.setMotor(-(bl_wheel_.eff + bl_wheel_.curr_pwm), MOTOR_BL);

  bl_wheel_.curr_pwm = bl_wheel_.eff + bl_wheel_.curr_pwm;

  motor_ctr.setMotor(-(fr_wheel_.eff + fr_wheel_.curr_pwm), MOTOR_FR);  
  
  fr_wheel_.curr_pwm = fr_wheel_.eff + fr_wheel_.curr_pwm;
  
  motor_ctr.setMotor(-(br_wheel_.eff + br_wheel_.curr_pwm), MOTOR_BR);
  
  br_wheel_.curr_pwm = br_wheel_.eff + br_wheel_.curr_pwm;

  //RCLCPP_INFO(logger_, "  Write Motor Value:  %f", (fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate));
  //RCLCPP_INFO(logger_, "  Write Motor raw:  %f", fl_wheel_.cmd);

  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  IcarusInterface,
  hardware_interface::SystemInterface
) 