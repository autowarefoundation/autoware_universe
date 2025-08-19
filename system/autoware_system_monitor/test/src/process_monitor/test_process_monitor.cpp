// Copyright 2020,2025 Tier IV, Inc.
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

#include "system_monitor/process_monitor/process_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

namespace fs = boost::filesystem;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

char ** argv_;

class TestProcessMonitor : public ProcessMonitor
{
  friend class ProcessMonitorTestSuite;

public:
  TestProcessMonitor(const std::string & node_name, const rclcpp::NodeOptions & options)
  : ProcessMonitor(node_name, options)
  {
  }

  virtual ~TestProcessMonitor() {}

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
  {
    array_ = *diag_msg;
  }

  void update() { updater_.force_update(); }

  void forceTimerEvent() { this->onTimer(); }

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

  void dumpDiagArray(const std::string & file_path)
  {
    // The file will be closed by the destructor.
    std::ofstream out_stream(file_path);
    if (!out_stream) {
      return;
    }
    for (size_t i = 0; i < array_.status.size(); ++i) {
      DiagStatus status = array_.status[i];
      auto values = status.values;
      out_stream << "- level:       " << std::to_string(status.level) << std::endl;
      out_stream << "  name:        " << status.name << std::endl;
      out_stream << "  message:     " << status.message << std::endl;
      out_stream << "  hardware_id: " << status.hardware_id << std::endl;
      out_stream << "  values:" << std::endl;
      for (size_t j = 0; j < values.size(); ++j) {
        out_stream << "  - key: " << fmt::format("{:10}", values[j].key)
                   << " value: " << values[j].value << std::endl;
      }
    }
  }

  bool areLinesDifferent(const std::string & expected_line, const std::string & result_line)
  {
    if (expected_line.compare(result_line) == 0) {
      // Lines are the same.
      return false;
    }
    // These 3 lines depend on testing environment and they can be different.
    if (
      (expected_line.find("key: execution time value") != std::string::npos) ||
      (expected_line.find("key: USER") != std::string::npos) ||
      (expected_line.find("hardware_id:") != std::string::npos)) {
      return false;
    }
    std::cerr << "File content mismatch." << std::endl;
    std::cerr << "  Expected: " << expected_line << std::endl;
    std::cerr << "  Result:   " << result_line << std::endl;
    return true;
  }

  bool compareStatusFiles(const std::string & expected, const std::string & result)
  {
    // The files will be closed by the destructors.
    std::ifstream expected_file(expected);
    if (!expected_file) {
      std::cerr << "File 'expected' does not exist." << std::endl;
      return false;
    }
    std::ifstream result_file(result);
    if (!result_file) {
      std::cerr << "File 'result' does not exist." << std::endl;
      return false;
    }

    bool eof_expected = false;
    bool eof_result = false;
    while (true) {
      std::string expected_line;
      if (!std::getline(expected_file, expected_line, '\n')) {
        eof_expected = true;
      }
      std::string result_line;
      if (!std::getline(result_file, result_line, '\n')) {
        eof_result = true;
      }
      if (eof_expected && eof_result) {
        // expected and result reached EOF at the same time.
        break;
      }
      if (eof_expected && !eof_result) {
        std::cerr << "File 'result' has extra lines." << std::endl;
        return false;
      }
      if (!eof_expected && eof_result) {
        std::cerr << "File 'result' has insufficient lines." << std::endl;
        return false;
      }
      if (areLinesDifferent(expected_line, result_line)) {
        return false;
      }
    }
    // Both files reached EOF without any difference.
    return true;
  }

  int prepareTestData(const std::string & file_set, const std::string & dst);
  void cleanupTestData(const std::string & dir);

private:
  diagnostic_msgs::msg::DiagnosticArray array_;
  const std::string prefix_ = std::string(this->get_name()) + ": ";
};

int TestProcessMonitor::prepareTestData(const std::string & file_set, const std::string & dst)
{
  // dst must be absolute path.
  if (dst.empty() || dst.at(0) != '/') {
    std::cerr << "Invalid destination directory: " << dst << std::endl;
    return -1;
  }
  std::string cmdLine =
    fmt::format("mkdir -p {}; cd {}; tar xjf {} --overwrite", dst, dst, file_set);
  int result = ::system(cmdLine.c_str());
  if (result == 0) {
    // setRoot() should be called after the test data is prepared.
    setRoot(dst);
  }
  return result;
}

void TestProcessMonitor::cleanupTestData(const std::string & dir)
{
  // "rm -rf" can be dangerous.
  // Make it sure that it doesn't destroy unrelated files and directories by accident.
  if (dir.find("/test_data") == std::string::npos) {
    return;
  }
  const std::string cmd_line = fmt::format("rm -rf {}", dir);
  int result = ::system(cmd_line.c_str());
  if (result != 0) {
    std::cerr << "Failed to cleanup test data directory: " << dir << std::endl;
  }
  setRoot("/");
}

class ProcessMonitorTestSuite : public ::testing::Test
{
public:
  ProcessMonitorTestSuite()
  {
    // Get directory of executable
    const fs::path exe_path(argv_[0]);
    exe_dir_ = exe_path.parent_path().generic_string();
  }

protected:
  std::unique_ptr<TestProcessMonitor> monitor_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  std::string exe_dir_;

  void SetUp()
  {
    using std::placeholders::_1;
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    monitor_ = std::make_unique<TestProcessMonitor>("test_process_monitor", node_options);
    sub_ = monitor_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 1000, std::bind(&TestProcessMonitor::diagCallback, monitor_.get(), _1));
  }

  void TearDown() { rclcpp::shutdown(); }

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
};

struct DummyProcFile
{
  std::string file_name;
};

class ProcessMonitorTestSuiteWithDummyProc : public ProcessMonitorTestSuite,
                                             public ::testing::WithParamInterface<DummyProcFile>
{
};

DummyProcFile dummy_proc_files[] = {
  {"dummy_proc_base.tar.bz2"},  // Original test data. Used as the base for the variations.
  {"dummy_proc_stat_comm_variations.tar.bz2"},   // Test data with variations in the comm field of
                                                 // /proc/stat.
  {"dummy_proc_extreme_data.tar.bz2"},           // Test data with extreme values.
  {"dummy_proc_stat_state_variations.tar.bz2"},  // Test data with variations in the state field of
                                                 // /proc/stat.
  {"dummy_proc_missing_files.tar.bz2"},          // Test data with missing files.
  {"dummy_proc_incomplete_files.tar.bz2"},       // Test data with incomplete files.
  {"dummy_proc_overflow_values.tar.bz2"},        // Test data with overflow values.
  {"dummy_proc_nonnumeric_files.tar.bz2"},  // Test data with non-numeric values in numeric fields.
  {"dummy_proc_memory_size_format.tar.bz2"},  // Test data with memory size formatting. Based on
                                              // "top" command.
  {"dummy_proc_unreadable.tar.bz2"},          // Test data with unreadable /proc file system.
  {"dummy_proc_not_mounted.tar.bz2"},         // Test data with /proc not mounted.
  {"dummy_proc_no_uptime.tar.bz2"},           // Test data with no /proc/uptime.
  {"dummy_proc_negative_priority.tar.bz2"},   // Test data with negative priority and real-time
                                              // priority.
};

struct DummyProcFilePair
{
  std::string file_name1;
  std::string file_name2;
};

// Test data with two dummy files. Used to test changes in a time period.
class ProcessMonitorTestSuiteWithDummyProcPair
: public ProcessMonitorTestSuite,
  public ::testing::WithParamInterface<DummyProcFilePair>
{
};

DummyProcFilePair dummy_proc_pairs[] = {
  {"dummy_proc_base.tar.bz2",
   "dummy_proc_base_1s_after.tar.bz2"},  // 1 second after the original test data.
  {"dummy_proc_base.tar.bz2",
   "dummy_proc_appearing_process.tar.bz2"},  // A process appears after 1 second.
  {"dummy_proc_base.tar.bz2",
   "dummy_proc_disappearing_process.tar.bz2"},  // A process disappears after 1 second.
  {"dummy_proc_base.tar.bz2",
   "dummy_proc_replaced_processes.tar.bz2"},  // A process appears, while another process
                                              // disappears.
};

TEST_F(ProcessMonitorTestSuite, tasksSummaryTest)
{
  monitor_->forceTimerEvent();
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  ASSERT_TRUE(monitor_->findDiagStatus("Tasks Summary", status));
  ASSERT_EQ(status.level, DiagStatus::OK);
}

TEST_F(ProcessMonitorTestSuite, highLoadProcTest)
{
  monitor_->forceTimerEvent();
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-load Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_F(ProcessMonitorTestSuite, highMemProcTest)
{
  monitor_->forceTimerEvent();
  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  // Verify
  DiagStatus status;
  std::string value;

  for (int i = 0; i < monitor_->getNumOfProcs(); ++i) {
    ASSERT_TRUE(monitor_->findDiagStatus(fmt::format("High-mem Proc[{}]", i), status));
    ASSERT_EQ(status.level, DiagStatus::OK);
  }
}

TEST_P(ProcessMonitorTestSuiteWithDummyProc, dummyProcTest)
{
  const DummyProcFile test_param = GetParam();
  const std::string test_data_dir = exe_dir_ + "/test_data";
  const std::string fileSetPath = exe_dir_ + "/" + test_param.file_name;

  // Make it sure that the dummy files are deleted when an error occurs and this test aborts.

  // Place_holder is just a dummy to make the custom destructor work (nullptr doesn't work).
  // Do NOT delete/free it as it is allocated on the stack, not on the heap.
  int place_holder;
  std::unique_ptr<int, std::function<void(int *)>> watch_dog(
    &place_holder, [&](int *) { monitor_->cleanupTestData(test_data_dir); });

  int result = monitor_->prepareTestData(fileSetPath, test_data_dir);
  ASSERT_EQ(result, 0);

  monitor_->forceTimerEvent();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  const std::string resultFilePath = test_data_dir + "/result";
  const std::string expectedFilePath = test_data_dir + "/expected";
  monitor_->dumpDiagArray(resultFilePath);

  ASSERT_TRUE(monitor_->compareStatusFiles(expectedFilePath, resultFilePath));
}

// Instantiate the test for each dummy file.
INSTANTIATE_TEST_SUITE_P(
  FileVariationsTest, ProcessMonitorTestSuiteWithDummyProc, ::testing::ValuesIn(dummy_proc_files));

TEST_P(ProcessMonitorTestSuiteWithDummyProcPair, dummyProcPairTest)
{
  const DummyProcFilePair test_param = GetParam();
  const std::string test_data_dir = exe_dir_ + "/test_data";
  const std::string fileSetPath1 = exe_dir_ + "/" + test_param.file_name1;
  const std::string fileSetPath2 = exe_dir_ + "/" + test_param.file_name2;

  // Make it sure that the dummy files are deleted when an error occurs and this test aborts.

  // Place_holder is just a dummy to make the custom destructor work (nullptr doesn't work).
  // Do NOT delete/free it as it is allocated on the stack, not on the heap.
  int place_holder;
  std::unique_ptr<int, std::function<void(int *)>> watch_dog(
    &place_holder, [&](int *) { monitor_->cleanupTestData(test_data_dir); });

  int result1 = monitor_->prepareTestData(fileSetPath1, test_data_dir);
  ASSERT_EQ(result1, 0);
  monitor_->forceTimerEvent();

  // Need to clean up the test data before preparing the next test data.
  // Otherwise, files of disappeared processes will be left in the test data directory.
  monitor_->cleanupTestData(test_data_dir);

  int result2 = monitor_->prepareTestData(fileSetPath2, test_data_dir);
  ASSERT_EQ(result2, 0);
  monitor_->forceTimerEvent();

  // Publish topic
  monitor_->update();

  // Give time to publish
  rclcpp::WallRate(2).sleep();
  rclcpp::spin_some(monitor_->get_node_base_interface());

  const std::string resultFilePath = test_data_dir + "/result";
  const std::string expectedFilePath = test_data_dir + "/expected";
  monitor_->dumpDiagArray(resultFilePath);

  ASSERT_TRUE(monitor_->compareStatusFiles(expectedFilePath, resultFilePath));
}

// Instantiate the test for each dummy file pair.
INSTANTIATE_TEST_SUITE_P(
  TimeLapseTest, ProcessMonitorTestSuiteWithDummyProcPair, ::testing::ValuesIn(dummy_proc_pairs));

int main(int argc, char ** argv)
{
  argv_ = argv;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
