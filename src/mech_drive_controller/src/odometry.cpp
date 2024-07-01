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

namespace mech_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x(0.0),
  linear_y(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  left_wheel_radius_(0.0),
  right_wheel_radius_(0.0),
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  back_left_wheel_old_pos_(0.0),
  back_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_x(velocity_rolling_window_size),
  linear_accumulator_y(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
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
  const double front_left_wheel_est_vel = front_left_wheel_cur_pos - front_left_wheel_old_pos_;
  const double front_right_wheel_est_vel = front_right_wheel_cur_pos - front_right_wheel_old_pos_;
  const double back_left_wheel_est_vel = back_left_wheel_cur_pos - back_left_wheel_old_pos_;
  const double back_right_wheel_est_vel = back_right_wheel_cur_pos - back_right_wheel_old_pos_;

  // Update old position with current:
  front_left_wheel_old_pos_ = front_left_wheel_cur_pos;
  front_right_wheel_old_pos_ = front_right_wheel_cur_pos;
  back_left_wheel_old_pos_ = back_left_wheel_cur_pos;
  back_right_wheel_old_pos_ = back_right_wheel_cur_pos;
  

  // Compute linear and angular diff:
  //x[0], y[1]
  std::vector<double> linear({0,0});
  linear[0] = (front_left_wheel_est_vel + front_right_wheel_est_vel + back_left_wheel_est_vel + back_right_wheel_est_vel) * left_wheel_radius_ / 4  ;
  linear[1] = (-front_left_wheel_est_vel + front_right_wheel_est_vel - back_left_wheel_est_vel + front_right_wheel_est_vel) * left_wheel_radius_ / 4; 
  // Now there is a bug about scout angular velocity
  const double angular = (front_right_wheel_est_vel - back_left_wheel_est_vel - front_left_wheel_est_vel + back_right_wheel_est_vel) * left_wheel_radius_ / (4* wheel_separation_);

  // Integrate odometry:
  integrateExact(linear, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  // TODO: Make linear accumaltor definition into a vector to store x and y vals 
  linear_accumulator_x.accumulate(linear[0] / dt); 
  linear_x = linear_accumulator_x.getRollingMean();
  linear_accumulator_y.accumulate(linear[1] / dt);
  linear_y = linear_accumulator_y.getRollingMean();
  angular_accumulator_.accumulate(angular / dt);

 

  angular_ = angular_accumulator_.getRollingMean();

  std::cout << "linear_x: " << linear_x << ", linear_y: " << linear_y << ", angular: " << angular_ << std::endl;
  std::cout << "Position: x = " << x_ << ", y = " << y_ << ", heading = " << heading_ << std::endl;
  return true;
}

void Odometry::updateOpenLoop(std::vector<double> & linear, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  double linear_x = linear[0];
  double linear_y = linear[1];
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;

  std::vector<double> linear_dt = {linear_x * dt, linear_y * dt};
  integrateExact(linear_dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation, double left_wheel_radius, double right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  left_wheel_radius_ = left_wheel_radius;
  right_wheel_radius_ = right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(std::vector<double> linear, double angular)
{
  const double direction = heading_ + angular ;

  /// Runge-Kutta 2nd order integration:
  x_ += (linear[0] * cos(direction) - linear[1]*sin(direction));
  y_ += (linear[0] * sin(direction) + linear[1]*cos(direction));
  heading_ += angular;
}

void Odometry::integrateExact(std::vector<double> linear, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(linear, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double rx = linear[0] / angular;
    const double ry = linear[1] / angular;
    heading_ += angular;
    x_ += rx* (sin(heading_) - sin(heading_old));
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
