#include "diff_interface/diffdrive_MyBot.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"




DiffDriveMyBot::DiffDriveMyBot()
    : logger_(rclcpp::get_logger("DiffDriveMyBot"))
{}

return_type DiffDriveMyBot::configure(const hardware_interface::HardwareInfo & info)
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


  //setup motor_control
  motor_ctr.start_motors(FL, BL, FR, BR);
  //motor_ctr.start_encoders();


  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveMyBot::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> DiffDriveMyBot::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}


return_type DiffDriveMyBot::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  //arduino_.sendEmptyMsg();
  //arduino_.setPidValues(30, 20, 0, 100);

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveMyBot::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveMyBot::read()
{

  // TODO fix chrono duration

  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;


  if (1==0) //check connection
  {
    return return_type::ERROR;
  }

  //setup motor encoder
  //int val = motor_ctr.read_encoders();
  //RCLCPP_INFO(logger_, "  Read Encoder Values:  %i", val);

  double pos_prev = fl_wheel_.pos;
  fl_wheel_.pos = fl_wheel_.calcEncAngle();
  fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = bl_wheel_.pos;
  bl_wheel_.pos = fr_wheel_.calcEncAngle();
  bl_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = fr_wheel_.pos;
  fr_wheel_.pos = fr_wheel_.calcEncAngle();
  fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = br_wheel_.pos;
  br_wheel_.pos = fr_wheel_.calcEncAngle();
  br_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;



  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveMyBot::write()
{

  if (1==0)  //check connection
  {
    return return_type::ERROR;
  }

  //wire to motors
  motor_ctr.setMotor(fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate, FL);
  motor_ctr.setMotor(bl_wheel_.cmd / bl_wheel_.rads_per_count / cfg_.loop_rate, BL);
  motor_ctr.setMotor(fr_wheel_.cmd / fr_wheel_.rads_per_count / cfg_.loop_rate, FR);
  motor_ctr.setMotor(br_wheel_.cmd / br_wheel_.rads_per_count / cfg_.loop_rate, BR);
  //RCLCPP_INFO(logger_, "  Write Motor Value:  %f", (fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate));

  return return_type::OK;


  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveMyBot,
  hardware_interface::SystemInterface
)