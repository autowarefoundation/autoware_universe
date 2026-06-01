//  Copyright 2025 The Autoware Contributors
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
#ifndef NON_REDUNDANT_ADAPTER_HPP_
#define NON_REDUNDANT_ADAPTER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>

namespace autoware::redundancy_switcher
{

class NonRedundantSwitcherAdapter : public IAdapterPlugin
{
public:
  NonRedundantSwitcherAdapter() = default;
  ~NonRedundantSwitcherAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  // All diag callbacks return OK unconditionally — no hardware switcher to check.
  static void diag_skipped(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::unique_ptr<diagnostic_updater::Updater> updater_;
};

}  // namespace autoware::redundancy_switcher
#endif  // NON_REDUNDANT_ADAPTER_HPP_
