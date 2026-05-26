// Copyright 2026 The Autoware Contributors
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

#include "trajectory_gate.hpp"

#include "core/monitor.hpp"
#include "core/publisher.hpp"
#include "core/subscription.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::trajectory_gate
{

TrajectoryGate::TrajectoryGate(const rclcpp::NodeOptions & options)
: Node("trajectory_gate", options), diag_(this, 0.1)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  diag_.setHardwareID("none");

  pub_source_ =
    create_publisher<TrajectorySourceStatus>("~/source/status", rclcpp::QoS(1).transient_local());
  srv_source_ = create_service<ChangeTrajectorySource>(
    "~/source/change", std::bind(&TrajectoryGate::on_change_source, this, _1, _2));

  // Create input streams.
  {
    using autoware_utils_diagnostics::TimeoutDiag;

    TimeoutDiag::Params param;
    param.warn_duration_ = declare_parameter<double>("trajectory_warn_duration");
    param.error_duration_ = declare_parameter<double>("trajectory_error_duration");

    const auto ids = declare_parameter<std::vector<int>>("source_ids");
    for (const auto id : ids) {
      const auto ns = "source." + std::to_string(id);
      const auto name = declare_parameter<std::string>(ns + ".name");

      auto timeout = std::make_unique<TimeoutDiag>(param, *this->get_clock(), name);
      diag_.add(*timeout);

      auto subscription = std::make_unique<TrajectorySubscription>(name, *this);
      auto monitor = std::make_unique<TrajectoryMonitor>(std::move(timeout));

      subscription->set_output(monitor.get());
      monitor->set_output(nullptr);
      selector_.add_input(monitor.get(), id);

      subscriptions_.push_back(std::move(subscription));
      receivers_.push_back(std::move(monitor));
    }
  }

  // Create output stream.
  {
    auto publisher = std::make_unique<TrajectoryPublisher>(*this);
    selector_.set_output(publisher.get());
    receivers_.push_back(std::move(publisher));
  }

  publish_source();
}

void TrajectoryGate::publish_source() const
{
  TrajectorySourceStatus msg;
  msg.source = selector_.source();
  pub_source_->publish(msg);
}

void TrajectoryGate::on_change_source(
  const ChangeTrajectorySource::Request::SharedPtr req,
  const ChangeTrajectorySource::Response::SharedPtr res)
{
  const auto success = selector_.select(req->source);
  publish_source();
  res->success = success;

  if (!success) {
    const auto source = std::to_string(req->source);
    RCLCPP_ERROR_STREAM(get_logger(), "source select failed: " << source);
  }
}

}  // namespace autoware::trajectory_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_gate::TrajectoryGate)
