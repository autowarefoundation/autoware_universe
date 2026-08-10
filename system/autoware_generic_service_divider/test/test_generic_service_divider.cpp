// Copyright 2025 TIER IV, Inc.
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

#include "generic_service_divider/service_divider_plugin_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_system_msgs/srv/change_operation_mode.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using ChangeOperationMode = autoware_system_msgs::srv::ChangeOperationMode;
using std::chrono_literals::operator""s;

class TestGenericServiceDivider : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    main_called_.store(false);
    sub_called_.store(false);

    mock_node_ = std::make_shared<rclcpp::Node>("mock_servers");
    client_node_ = std::make_shared<rclcpp::Node>("test_client");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(mock_node_);
    executor_->add_node(client_node_);
  }

  void TearDown() override
  {
    if (spin_thread_.joinable()) {
      executor_->cancel();
      spin_thread_.join();
    }
    client_.reset();
    main_server_.reset();
    sub_server_.reset();
    plugin_.reset();
    plugin_loader_.reset();
    executor_.reset();
    divider_node_.reset();
    mock_node_.reset();
    client_node_.reset();
  }

  void create_main_server(bool succeeds = true)
  {
    main_server_ = mock_node_->create_service<ChangeOperationMode>(
      "/test/main/change_op", [this, succeeds](
                                const ChangeOperationMode::Request::SharedPtr,
                                ChangeOperationMode::Response::SharedPtr response) {
        main_called_.store(true);
        response->status.success = succeeds;
        response->status.code = succeeds ? 0 : 50000;
        response->status.message = succeeds ? "main ok" : "main failed";
      });
  }

  void create_sub_server(bool succeeds = true)
  {
    sub_server_ = mock_node_->create_service<ChangeOperationMode>(
      "/test/sub/change_op", [this, succeeds](
                               const ChangeOperationMode::Request::SharedPtr,
                               ChangeOperationMode::Response::SharedPtr response) {
        sub_called_.store(true);
        response->status.success = succeeds;
        response->status.code = succeeds ? 0 : 50000;
        response->status.message = succeeds ? "sub ok" : "sub failed";
      });
  }

  void create_mock_servers(bool main_succeeds = true, bool sub_succeeds = true)
  {
    create_main_server(main_succeeds);
    create_sub_server(sub_succeeds);
  }

  void create_divider_node(int timeout_ms = 3000)
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override(
      "plugins", std::vector<std::string>{"generic_service_divider::ChangeOperationModeDivider"});
    options.append_parameter_override("change_operation_mode.input_service", "/test/change_op");
    options.append_parameter_override(
      "change_operation_mode.output_services.names",
      std::vector<std::string>{"/test/main/change_op", "/test/sub/change_op"});
    options.append_parameter_override(
      "change_operation_mode.output_services.primaries", std::vector<bool>{true, false});
    options.append_parameter_override(
      "change_operation_mode.output_services.timeouts_ms",
      std::vector<int64_t>{timeout_ms, timeout_ms});

    divider_node_ = std::make_shared<rclcpp::Node>("divider_host", options);

    plugin_loader_ =
      std::make_shared<pluginlib::ClassLoader<generic_service_divider::ServiceDividerPluginBase>>(
        "autoware_generic_service_divider", "generic_service_divider::ServiceDividerPluginBase");

    auto divider_shared = std::shared_ptr<rclcpp::Node>(divider_node_);
    plugin_ =
      plugin_loader_->createSharedInstance("generic_service_divider::ChangeOperationModeDivider");
    plugin_->initialize(divider_shared);
    plugin_->setup_service_division();

    executor_->add_node(divider_node_);
  }

  void create_client()
  {
    client_ = client_node_->create_client<ChangeOperationMode>("/test/change_op");
  }

  void start_spinning()
  {
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  ChangeOperationMode::Response::SharedPtr call_service(uint16_t mode = 2)
  {
    auto request = std::make_shared<ChangeOperationMode::Request>();
    request->mode = mode;

    EXPECT_TRUE(client_->wait_for_service(5s));

    auto future = client_->async_send_request(request);
    auto status = future.wait_for(10s);
    EXPECT_EQ(status, std::future_status::ready) << "Service call timed out";
    if (status != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  /// Bring the divider up, call the input service once and return the response.
  ChangeOperationMode::Response::SharedPtr run_division(int timeout_ms = 3000)
  {
    create_divider_node(timeout_ms);
    create_client();
    start_spinning();
    return call_service();
  }

  /// Single place holding the assertions shared by the division scenarios.
  void expect_division_result(
    const ChangeOperationMode::Response::SharedPtr & response, bool expected_success,
    bool expected_main_called, bool expected_sub_called)
  {
    ASSERT_NE(response, nullptr);
    EXPECT_EQ(response->status.success, expected_success);
    EXPECT_EQ(main_called_.load(), expected_main_called);
    EXPECT_EQ(sub_called_.load(), expected_sub_called);
  }

  /// Poll `predicate` until it holds or `timeout` elapses (the executor spins in its own thread).
  bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return predicate();
  }

  std::shared_ptr<rclcpp::Node> mock_node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> divider_node_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr main_server_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr sub_server_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<pluginlib::ClassLoader<generic_service_divider::ServiceDividerPluginBase>>
    plugin_loader_;
  std::shared_ptr<generic_service_divider::ServiceDividerPluginBase> plugin_;
  std::thread spin_thread_;
  std::atomic<bool> main_called_;
  std::atomic<bool> sub_called_;
};

// Both servers succeed -> client gets success
TEST_F(TestGenericServiceDivider, BothServersSucceed)
{
  create_mock_servers(true, true);
  expect_division_result(run_division(), true, true, true);
}

// Main succeeds but sub fails -> client gets error
TEST_F(TestGenericServiceDivider, SubServerFails)
{
  create_mock_servers(true, false);
  expect_division_result(run_division(), false, true, true);
}

// Main fails but sub succeeds -> client gets error
TEST_F(TestGenericServiceDivider, MainServerFails)
{
  create_mock_servers(false, true);
  expect_division_result(run_division(), false, true, true);
}

// Both servers fail -> client gets error
TEST_F(TestGenericServiceDivider, BothServersFail)
{
  create_mock_servers(false, false);
  expect_division_result(run_division(), false, true, true);
}

// Two consecutive calls both get answered (the pending division must be cleaned up in between)
TEST_F(TestGenericServiceDivider, ConsecutiveCallsSucceed)
{
  create_mock_servers(true, true);
  expect_division_result(run_division(), true, true, true);
  expect_division_result(call_service(), true, true, true);
}

// The input service must not be advertised until every output server is available
TEST_F(TestGenericServiceDivider, InputServiceWaitsForOutputServers)
{
  create_main_server(true);
  create_divider_node();
  create_client();
  start_spinning();

  EXPECT_FALSE(client_->wait_for_service(1s)) << "input service advertised too early";

  const auto info_while_waiting = plugin_->get_startup_diagnostic_info();
  EXPECT_FALSE(info_while_waiting.input_service_started);
  EXPECT_EQ(info_while_waiting.ready_output_service_count, 1u);
  EXPECT_EQ(info_while_waiting.total_output_service_count, 2u);
  ASSERT_EQ(info_while_waiting.waiting_output_services.size(), 1u);
  EXPECT_EQ(info_while_waiting.waiting_output_services.front(), "/test/sub/change_op");
  EXPECT_EQ(info_while_waiting.input_service_name, "/test/change_op");

  // Once the missing server shows up, the retry timer advertises the input service.
  create_sub_server(true);
  EXPECT_TRUE(client_->wait_for_service(5s));
  EXPECT_TRUE(wait_until(
    [this]() { return plugin_->get_startup_diagnostic_info().input_service_started; }, 5s));

  const auto info_when_ready = plugin_->get_startup_diagnostic_info();
  EXPECT_EQ(info_when_ready.ready_output_service_count, 2u);
  EXPECT_TRUE(info_when_ready.waiting_output_services.empty());
}

// Sub server disappears after the input service is advertised -> timeout -> client gets error.
// Note: the sub server must exist first, otherwise the input service is never advertised
// (see try_start_input_service()), and the timeout path could not be reached at all.
TEST_F(TestGenericServiceDivider, SubServerTimeout)
{
  create_mock_servers(true, true);
  create_divider_node(1000);  // 1 second timeout
  create_client();
  start_spinning();

  ASSERT_TRUE(client_->wait_for_service(5s));
  ASSERT_TRUE(wait_until(
    [this]() { return plugin_->get_startup_diagnostic_info().input_service_started; }, 5s));

  sub_server_.reset();
  expect_division_result(call_service(), false, true, false);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
