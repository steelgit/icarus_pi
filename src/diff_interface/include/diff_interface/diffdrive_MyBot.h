#ifndef DIFFDRIVE_MYBOT_REAL_ROBOT_H
#define DIFFDRIVE_MYBOT_REAL_ROBOT_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.h"
#include "wheel.h"
#include "motor_control.h"


using hardware_interface::return_type;

class DiffDriveMyBot : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{


public:
  DiffDriveMyBot();
  ~DiffDriveMyBot();
  
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type start() override;

  return_type stop() override;

  return_type read() override;

  return_type write() override;



private:

  Config cfg_;

  Wheel fl_wheel_;
  Wheel fr_wheel_;
  Wheel bl_wheel_;
  Wheel br_wheel_;
  motor_control motor_ctr;

  rclcpp::Logger logger_;

  std::chrono::time_point<std::chrono::system_clock> time_;
  
  // motor = {PWM_MAX, ENA, IN1, IN2, FREQ, RANGE}
  uint8_t PWM_MAX = 255;
  uint8_t FREQ = 200;
  uint16_t RANGE = 255;

  motor FL = {PWM_MAX, 21, 20, 16, FREQ, RANGE};
  motor FR = {PWM_MAX, 13, 6, 5, FREQ, RANGE};
  motor BL = {PWM_MAX, 2, 3, 4, FREQ, RANGE};
  motor BR = {PWM_MAX, 25, 24, 23, FREQ, RANGE};

  std::vector<double> debug;
  
};


#endif // DIFFDRIVE_MYBOT_REAL_ROBOT_H