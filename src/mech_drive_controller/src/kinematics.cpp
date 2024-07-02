//DONE
// mecanum wheel equations => https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <cmath>
#include <memory>

#include "mech_drive_controller/kinematics.hpp"

#include "mech_drive_controller/types.hpp"

namespace mech_drive_controllers {

Kinematics::Kinematics(RobotParams robot_params)
  : robot_params_(robot_params) {
  this->initializeParams();
}

Kinematics::Kinematics() {
  this->initializeParams();
}

//forward kinematics (given wheel velocities, find base velocity)
RobotVelocity Kinematics::getBodyVelocity(const std::vector<double> & wheels_vel) {
  RobotVelocity vel;
  double vel_front_left = wheels_vel.at(0);
  double vel_front_right = wheels_vel.at(1);
  double vel_back_right = wheels_vel.at(2);
  double vel_back_left = wheels_vel.at(3);

  vel.vx = ((vel_front_left + vel_front_right + vel_back_right + vel_back_left) * robot_params_.wheel_radius / 4);
  vel.vy = ((-vel_front_left + vel_front_right - vel_back_right + vel_back_left) * robot_params_.wheel_radius / 4);
  vel.omega = ((-vel_front_left + vel_front_right + vel_back_right - vel_back_left) * robot_params_.wheel_radius / (4 * (robot_params_.wheel_separation_width + robot_params_.wheel_separation_length)));

  return vel;
}

//inverse kinematics (given base velocity find wheel velocities)
std::vector<double> Kinematics::getWheelsAngularVelocities(RobotVelocity vel) {
  double x_linear_vel = vel.vx;
  double y_linear_vel = vel.vy;
  double yaw = vel.omega;

  angular_vel_vec_[0] = ((x_linear_vel - y_linear_vel - (yaw * (robot_params_.wheel_separation_length + robot_params_.wheel_separation_width))) * (1 / robot_params_.wheel_radius)); //front left 
  angular_vel_vec_[1] = ((x_linear_vel + y_linear_vel + (yaw * (robot_params_.wheel_separation_length + robot_params_.wheel_separation_width))) * (1 / robot_params_.wheel_radius)); //front right
  angular_vel_vec_[2] = ((x_linear_vel - y_linear_vel + (yaw * (robot_params_.wheel_separation_length + robot_params_.wheel_separation_width))) * (1 / robot_params_.wheel_radius)); //back right
  angular_vel_vec_[3] = ((x_linear_vel + y_linear_vel - (yaw * (robot_params_.wheel_separation_length + robot_params_.wheel_separation_width))) * (1 / robot_params_.wheel_radius)); //back left

  return angular_vel_vec_;
}

void Kinematics::setRobotParams(RobotParams robot_params) {
  this->robot_params_ = robot_params;
  this->initializeParams();
}

void Kinematics::initializeParams() {
  angular_vel_vec_.reserve(MECH_ROBOT_MAX_WHEELS);
  angular_vel_vec_ = {0, 0, 0, 0};
}

Kinematics::~Kinematics() {}

}  // namespace mech_drive_controllers
