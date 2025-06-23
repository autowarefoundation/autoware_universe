// Copyright 2020 Tier IV, Inc.
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

#include "system_monitor/cpu_monitor/unknown_cpu_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

class TestCPUMonitor : public CPUMonitor
{
  friend class CPUMonitorTestSuite;

public:
  TestCPUMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : CPUMonitor(node_name, options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    std::lock_guard<std::mutex> lock_diagnostic(mutex_diagnostic_);
    array_ = *diag_msg;
  }

  void update() { updater_.force_update(); }

  void forceTimerEvent() { this->onTimer(); }

  void disableTimer() { timer_->cancel(); }

private:
  std::mutex mutex_diagnostic_;  // Protects the diagnostic array.
  diagnostic_msgs::msg::DiagnosticArray array_;
};

class CPUMonitorTestSuite : public ::testing::Test
{
public:
  CPUMonitorTestSuite() : monitor_(nullptr), sub_(nullptr) {}

protected:
  std::unique_ptr<TestCPUMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestCPUMonitor>("test_cpu_monitor", node_options);
    // If the timer is enabled, it will interfere with the test.
    // NOTE:
    //   Though disabling the timer is necessary for the test,
    //   it makes the test run in a single thread context,
    //   different from the real case.
    monitor_->disableTimer();

    // The queue size is set to 1 so that the result of the changes made by the tests
    // can be delivered to the topic subscriber immediately.
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1, std::bind(&TestCPUMonitor::diagCallback, monitor_.get(), _1));
  }

  void TearDown() { rclcpp::shutdown(); }
};

TEST_F(CPUMonitorTestSuite, test)
{
  ASSERT_TRUE(true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
