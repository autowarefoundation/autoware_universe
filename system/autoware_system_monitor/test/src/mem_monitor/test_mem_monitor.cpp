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

#include "system_monitor/mem_monitor/mem_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestMemMonitor : public MemMonitor
{
  friend class MemMonitorTestSuite;

public:
  TestMemMonitor(const std::string & /*node_name*/, const rclcpp::NodeOptions & options)
  : MemMonitor(options)
  {
  }

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  void changeUsageWarn(size_t bytes) { warning_available_size_ = bytes; }
  void changeUsageError(size_t bytes) { error_available_size_ = bytes; }

  void update() { updater_.force_update(); }
  void setPeriod(const double period)
  {
    updater_.setPeriod(rclcpp::Duration::from_seconds(period));
  }

  const std::string removePrefix(const std::string & name)
  {
    return boost::algorithm::erase_all_copy(name, prefix_);
  }

  bool findDiagStatus(const std::string & name, DiagStatus & status)  // NOLINT
  {
    for (size_t i = 0; i < array_.status.size(); ++i) {
      if (removePrefix(array_.status[i].name) == name) {
        status = array_.status[i];
        return true;
      }
    }
    return false;
  }

  void clearDiagArray() { array_ = diagnostic_msgs::msg::DiagnosticArray(); }

private:
  diagnostic_msgs::msg::DiagnosticArray array_;
  const std::string prefix_ = std::string(this->get_name()) + ": ";
};

class MemMonitorTestSuite : public ::testing::Test
{
public:
  MemMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
    // Get dummy executable path
    free_ = exe_dir_ + "/free";
  }

protected:
  std::unique_ptr<TestMemMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;
  std::string free_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestMemMonitor>("test_mem_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestMemMonitor::diagCallback, monitor_.get(), _1));
    monitor_->setPeriod(10000.0);

    // Remove dummy executable if exists
    if (fs::exists(free_)) {
      fs::remove(free_);
    }
  }

  void TearDown()
  {
    // Remove dummy executable if exists
    if (fs::exists(free_)) {
      fs::remove(free_);
    }
    rclcpp::shutdown();
  }

  bool findValue(const DiagStatus status, const std::string & key, std::string & value)  // NOLINT
  {
    for (auto itr = status.values.begin(); itr != status.values.end(); ++itr) {
      if (itr->key == key) {
        value = itr->value;
        return true;
      }
    }
    return false;
  }

  void modifyPath()
  {
    // Modify PATH temporarily
    auto env = boost::this_process::environment();
    std::string new_path = env["PATH"].to_string();
    new_path.insert(0, fmt::format("{}:", exe_dir_));
    env["PATH"] = new_path;
  }

  bool waitForDiagStatus(
    const std::string & name, DiagStatus & status, int timeout_ms = 1000)  // NOLINT
  {
    // Clear previous diagnostic data before waiting for new one
    monitor_->clearDiagArray();

    // Trigger update
    monitor_->update();

    // Wait for the diagnostic message to be received
    auto start = std::chrono::steady_clock::now();
    while (true) {
      rclcpp::spin_some(monitor_->get_node_base_interface());
      if (monitor_->findDiagStatus(name, status)) {
        return true;
      }
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
        return false;
      }
      rclcpp::WallRate(100).sleep();  // 10ms sleep between spins
    }
  }
};

TEST_F(MemMonitorTestSuite, usageWarnTest)
{
  // Get baseline level (may be WARN if swap is in use)
  DiagStatus baseline_status;
  ASSERT_TRUE(waitForDiagStatus("Memory Usage", baseline_status));
  ASSERT_NE(baseline_status.level, DiagStatus::ERROR);

  // Verify warning
  {
    // Change warning level
    monitor_->changeUsageWarn(std::numeric_limits<size_t>::max());

    DiagStatus status;
    ASSERT_TRUE(waitForDiagStatus("Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::WARN);
  }

  // Verify return to baseline
  {
    // Change back to normal
    monitor_->changeUsageWarn(0);

    DiagStatus status;
    ASSERT_TRUE(waitForDiagStatus("Memory Usage", status));
    ASSERT_EQ(status.level, baseline_status.level);
  }
}

TEST_F(MemMonitorTestSuite, usageErrorTest)
{
  // Get baseline level (may be WARN if swap is in use)
  DiagStatus baseline_status;
  ASSERT_TRUE(waitForDiagStatus("Memory Usage", baseline_status));
  ASSERT_NE(baseline_status.level, DiagStatus::ERROR);

  // Verify error
  {
    // Change error level
    monitor_->changeUsageError(std::numeric_limits<size_t>::max());

    DiagStatus status;
    ASSERT_TRUE(waitForDiagStatus("Memory Usage", status));
    ASSERT_EQ(status.level, DiagStatus::ERROR);
  }

  // Verify return to baseline
  {
    // Change back to normal
    monitor_->changeUsageError(0);

    DiagStatus status;
    ASSERT_TRUE(waitForDiagStatus("Memory Usage", status));
    ASSERT_EQ(status.level, baseline_status.level);
  }
}

TEST_F(MemMonitorTestSuite, usageFreeErrorTest)
{
  // Symlink free1 to free
  fs::create_symlink(exe_dir_ + "/free1", free_);

  // Modify PATH temporarily
  modifyPath();

  // Verify
  DiagStatus status;
  std::string value;

  ASSERT_TRUE(waitForDiagStatus("Memory Usage", status));
  ASSERT_EQ(status.level, DiagStatus::ERROR);
  ASSERT_STREQ(status.message.c_str(), "free error");
  ASSERT_TRUE(findValue(status, "free", value));
}

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
