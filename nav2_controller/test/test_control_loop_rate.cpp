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

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <memory>

#include "nav2_controller/control_loop_rate.hpp"

using namespace std::chrono_literals;

class ControlLoopRateTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ControlLoopRateTest, PausedAndSlowClockDoNotAdvanceController)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000), RCL_RET_OK);
  nav2_controller::ControlLoopRate rate(20.0, clock, true);
  std::atomic<bool> cancel{false};
  const auto interrupted = [&]() {return cancel.load();};
  auto wait = std::async(std::launch::async, [&]() {return rate.sleep(interrupted);});
  EXPECT_EQ(wait.wait_for(150ms), std::future_status::timeout);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1049000000), RCL_RET_OK);
  EXPECT_EQ(wait.wait_for(100ms), std::future_status::timeout);
  EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1050000000), RCL_RET_OK);
  const auto ready = wait.wait_for(200ms);
  cancel = true;
  EXPECT_EQ(ready, std::future_status::ready);
  EXPECT_TRUE(wait.get());
}

TEST_F(ControlLoopRateTest, PausedClockCanBeInterruptedWithoutAnotherTick)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  nav2_controller::ControlLoopRate rate(20.0, clock, true);
  EXPECT_TRUE(rate.sleep([]() {return true;}));
}

TEST_F(ControlLoopRateTest, ClockJumpsReanchorWithoutCatchUpTicks)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  nav2_controller::ControlLoopRate rate(20.0, clock, true);
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000), RCL_RET_OK);
  EXPECT_FALSE(rate.sleep([]() {return false;}));
  EXPECT_TRUE(rate.sleep([]() {return true;}));
  ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 500000000), RCL_RET_OK);
  EXPECT_FALSE(rate.sleep([]() {return false;}));
  EXPECT_TRUE(rate.sleep([]() {return true;}));
}

TEST_F(ControlLoopRateTest, RealModeDoesNotWaitForRosClock)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
  nav2_controller::ControlLoopRate rate(20.0, clock, false);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(rate.sleep([]() {return false;}));
  EXPECT_GE(std::chrono::steady_clock::now() - start, 40ms);
  EXPECT_EQ(clock->now().nanoseconds(), 0);
}
