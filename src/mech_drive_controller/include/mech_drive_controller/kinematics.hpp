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

#ifndef MECH_DRIVE_CONTROLLER__KINEMATICS_HPP_
#define MECH_DRIVE_CONTROLLER__KINEMATICS_HPP_

#include <memory>
#include <vector>

#include "mech_drive_controller/types.hpp"

namespace mech_drive_controllers {

constexpr double MECH_ROBOT_MAX_WHEELS = 4;

class Kinematics {
 public:
  explicit Kinematics(RobotParams robot_params);
  Kinematics();
  ~Kinematics();
  // Forward kinematics
  RobotVelocity getBodyVelocity(const std::vector<double> & wheels_vel);
  // Inverse kinematics
  std::vector<double> getWheelsAngularVelocities(RobotVelocity vel);
  void setRobotParams(RobotParams robot_params);
 private:
  void initializeParams();
  RobotParams robot_params_;
  std::vector<double> angular_vel_vec_;
  double cos_gamma_;
  double sin_gamma_;
  double alpha_;
  double beta_;
};

}  // namespace mech_drive_controllers

#endif  // MECH_DRIVE_CONTROLLER__KINEMATICS_HPP_
