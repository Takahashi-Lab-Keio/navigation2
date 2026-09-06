// Copyright (c) 2026 Takahashi Laboratory
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CONTROLLER__CONTROL_LOOP_RATE_HPP_
#define NAV2_CONTROLLER__CONTROL_LOOP_RATE_HPP_

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"

namespace nav2_controller
{

// Simulation ticks follow the plant clock. Wall polling keeps action shutdown
// and cancellation responsive while Gazebo is paused; no catch-up ticks occur.
class ControlLoopRate
{
public:
  ControlLoopRate(double frequency, rclcpp::Clock::SharedPtr clock, bool use_sim_time)
  : wall_rate_(frequency), clock_(clock), use_sim_time_(use_sim_time),
    period_(rclcpp::Duration::from_seconds(1.0 / frequency)), last_tick_(clock_->now())
  {}

  bool sleep(const std::function<bool()> & interrupted)
  {
    if (!use_sim_time_) {
      return wall_rate_.sleep();
    }
    const auto target = last_tick_ + period_;
    auto current = clock_->now();
    if (current >= target || current < last_tick_) {
      last_tick_ = current;
      return false;
    }
    while (current < target) {
      if (interrupted()) {
        return true;
      }
      if (current < last_tick_) {
        last_tick_ = current;
        return false;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(1));
      current = clock_->now();
    }
    last_tick_ = current;
    return true;
  }

  void reset()
  {
    wall_rate_.reset();
    last_tick_ = clock_->now();
  }

private:
  rclcpp::WallRate wall_rate_;
  rclcpp::Clock::SharedPtr clock_;
  bool use_sim_time_;
  rclcpp::Duration period_;
  rclcpp::Time last_tick_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__CONTROL_LOOP_RATE_HPP_
