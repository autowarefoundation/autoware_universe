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
#include <memory>
#include <string>
#include <thread>
#include <vector>

using ChangeOperationMode = autoware_system_msgs::srv::ChangeOperationMode;
using namespace std::chrono_literals;

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

  void create_mock_servers(bool main_succeeds = true, bool sub_succeeds = true)
  {
    main_server_ = mock_node_->create_service<ChangeOperationMode>(
      "/test/main/change_op", [this, main_succeeds](
                                const ChangeOperationMode::Request::SharedPtr,
                                ChangeOperationMode::Response::SharedPtr response) {
        main_called_.store(true);
        response->status.success = main_succeeds;
        response->status.code = main_succeeds ? 0 : 50000;
        response->status.message = main_succeeds ? "main ok" : "main failed";
      });

    sub_server_ = mock_node_->create_service<ChangeOperationMode>(
      "/test/sub/change_op", [this, sub_succeeds](
                               const ChangeOperationMode::Request::SharedPtr,
                               ChangeOperationMode::Response::SharedPtr response) {
        sub_called_.store(true);
        response->status.success = sub_succeeds;
        response->status.code = sub_succeeds ? 0 : 50000;
        response->status.message = sub_succeeds ? "sub ok" : "sub failed";
      });
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
  create_divider_node();
  create_client();
  start_spinning();

  auto response = call_service();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->status.success);
  EXPECT_TRUE(main_called_.load());
  EXPECT_TRUE(sub_called_.load());
}

// Main succeeds but sub fails -> client gets error
TEST_F(TestGenericServiceDivider, SubServerFails)
{
  create_mock_servers(true, false);
  create_divider_node();
  create_client();
  start_spinning();

  auto response = call_service();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->status.success);
  EXPECT_TRUE(main_called_.load());
  EXPECT_TRUE(sub_called_.load());
}

// Main fails but sub succeeds -> client gets error
TEST_F(TestGenericServiceDivider, MainServerFails)
{
  create_mock_servers(false, true);
  create_divider_node();
  create_client();
  start_spinning();

  auto response = call_service();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->status.success);
  EXPECT_TRUE(main_called_.load());
  EXPECT_TRUE(sub_called_.load());
}

// Sub server not available -> timeout -> client gets error
TEST_F(TestGenericServiceDivider, SubServerTimeout)
{
  // Only create main server, no sub
  main_server_ = mock_node_->create_service<ChangeOperationMode>(
    "/test/main/change_op", [this](
                              const ChangeOperationMode::Request::SharedPtr,
                              ChangeOperationMode::Response::SharedPtr response) {
      main_called_.store(true);
      response->status.success = true;
      response->status.code = 0;
      response->status.message = "main ok";
    });

  create_divider_node(1000);  // 1 second timeout
  create_client();
  start_spinning();

  auto response = call_service();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->status.success);
  EXPECT_TRUE(main_called_.load());
  EXPECT_FALSE(sub_called_.load());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
