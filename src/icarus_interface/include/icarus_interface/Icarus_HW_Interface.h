#ifndef ICARUSINTERFACE_REAL_ROBOT_H
#define ICARUSINTERFACE_REAL_ROBOT_H

#include <cstring>
#include <pigpiod_if2.h>
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
#include "encoder_control.h"
#include "config_rasp_pi.h"


using hardware_interface::return_type;

class IcarusInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
  public:
    IcarusInterface();
    ~IcarusInterface();
    
    return_type configure(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type start() override;

    return_type stop() override;

    return_type read() override;

    return_type write() override;


  private:

    Config cfg_;

    motor_control motor_ctr;
    encoder_control enc_ctr;

    rclcpp::Logger logger_;

    rclcpp::Time currentTime_;
    rclcpp::Time previousTime_ ;
    std::chrono::time_point<std::chrono::system_clock> time_;

    std::vector<double> debug;

    char* OPTHOST = NULL;
    char* OPTPORT = NULL;
  
};


#endif // ICARUSINTERFACE_REAL_ROBOT_H