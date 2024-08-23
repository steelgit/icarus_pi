
// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Enrique Fernández
 */

#include "mech_drive_controller/odometry.hpp"
#include "iostream"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"



auto logger = rclcpp::get_logger("my_logger");
namespace mech_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_length_(0.0),
  wheel_separation_width_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  back_left_wheel_old_pos_(0.0),
  back_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_x(velocity_rolling_window_size),
  linear_accumulator_y(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size),
  calc_x(0.0),
  calc_y(0.0),
  calc_ang(0.0)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double front_left_pos, double front_right_pos, double back_left_pos, double back_right_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  // Get current wheel joint positions:
  const double front_left_wheel_cur_pos = front_left_pos * left_wheel_radius_;
  const double front_right_wheel_cur_pos = front_right_pos * right_wheel_radius_;
  const double back_left_wheel_cur_pos = back_left_pos * left_wheel_radius_;
  const double back_right_wheel_cur_pos = back_right_pos * right_wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double front_left_wheel_est_vel = (front_left_wheel_cur_pos - front_left_wheel_old_pos_);
  const double front_right_wheel_est_vel = (front_right_wheel_cur_pos - front_right_wheel_old_pos_);
  const double back_left_wheel_est_vel = (back_left_wheel_cur_pos - back_left_wheel_old_pos_);
  const double back_right_wheel_est_vel = (back_right_wheel_cur_pos - back_right_wheel_old_pos_);

  // Update old position with current:
  front_left_wheel_old_pos_ = front_left_wheel_cur_pos;
  front_right_wheel_old_pos_ = front_right_wheel_cur_pos;
  back_left_wheel_old_pos_ = back_left_wheel_cur_pos;
  back_right_wheel_old_pos_ = back_right_wheel_cur_pos;
  

  // Compute linear and angular diff:
  //x[0], y[1]
  const double linear_x = -(front_left_wheel_est_vel + front_right_wheel_est_vel + back_left_wheel_est_vel + back_right_wheel_est_vel) / 4;
  const double linear_y = -(-front_left_wheel_est_vel + front_right_wheel_est_vel + back_left_wheel_est_vel - back_right_wheel_est_vel) / 4;
  // Now there is a bug about scout angular velocity
  const double angular = (-front_left_wheel_est_vel + front_right_wheel_est_vel - back_left_wheel_est_vel + back_right_wheel_est_vel) / (4  * (wheel_separation_width_ + wheel_separation_length_) / 2);
  
  //calc_x += linear_x * dt;
  //calc_y += linear_y * dt;
  //calc_ang += angular;
  //RCLCPP_INFO(logger, "x: %f, y: %f, angular: %f", front_right_wheel_cur_pos, linear_y, calc_ang);
  // Integrate odometry:
  integrateExact(linear_x, linear_y, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  // TODO: Make linear accumaltor definition into a vector to store x and y vals 
  linear_accumulator_x.accumulate(linear_x); 
  linear_x_ = linear_accumulator_x.getRollingMean();

  linear_accumulator_y.accumulate(linear_y);
  linear_y_ = linear_accumulator_y.getRollingMean();
  
  angular_accumulator_.accumulate(angular);
  angular_ = angular_accumulator_.getRollingMean();

  //RCLCPP_WARN(logger, "linear_x: %f linear_x_: %f", linear_x, linear_x_);

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  angular_ = angular;
  linear_x_ = linear_x;
  linear_y_ = linear_y;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;

  integrateExact(linear_x * dt, linear_y * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation_length, double wheel_separation_width, double left_wheel_radius, double right_wheel_radius)
{
  wheel_separation_width_ = wheel_separation_width;
  wheel_separation_length_ = wheel_separation_length;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
{
  const double direction = heading_ + angular;
  /// Runge-Kutta 2nd order integration:
  x_ += (linear_x * cos(direction) - linear_y * sin(direction));
  y_ += (linear_x * sin(direction) + linear_y * cos(direction));
  //x_ += linear_x;
  //y_ += linear_y;
  heading_ += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear_x, linear_y, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double rx = linear_x / angular;
    const double ry = linear_y / angular;
    //RCLCPP_INFO(logger, "rx: %f, ry: %f, heading_: %f, heading old: %f", rx, ry, heading_, heading_old);
    heading_ += angular;
    x_ += rx * (sin(heading_) - sin(heading_old));
    y_ += ry * (cos(heading_) - cos(heading_old));

  }
}

void Odometry::resetAccumulators()
{
  linear_accumulator_x = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_accumulator_y = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller
