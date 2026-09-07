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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__COMMAND_BUS_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__COMMAND_BUS_HPP_

#include "redundancy_switcher_interface/ir/output_commands.hpp"
#include "redundancy_switcher_interface/plugin/i_adapter_plugin.hpp"

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>

namespace autoware::redundancy_switcher
{

class CommandBus
{
public:
  void add_handler(std::weak_ptr<IAdapterPlugin> adapter)
  {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    handlers_.push_back(std::move(adapter));
  }

  void dispatch(const std::vector<OutputCommand> & commands)
  {
    std::vector<std::shared_ptr<IAdapterPlugin>> live_handlers;
    {
      std::lock_guard<std::mutex> lock(handlers_mutex_);

      // Prune expired entries periodically to amortize the per-dispatch overhead.
      ++dispatch_count_;
      if (dispatch_count_ % kPruneInterval == 0) {
        handlers_.erase(
          std::remove_if(
            handlers_.begin(), handlers_.end(),
            [](const std::weak_ptr<IAdapterPlugin> & wp) { return wp.expired(); }),
          handlers_.end());
      }

      live_handlers.reserve(handlers_.size());
      for (auto & wp : handlers_) {
        if (auto adapter = wp.lock()) {
          live_handlers.push_back(std::move(adapter));
        }
      }
    }

    for (const auto & command : commands) {
      for (auto & adapter : live_handlers) {
        adapter->execute(command);
      }
    }
  }

private:
  static constexpr std::size_t kPruneInterval = 16;
  mutable std::mutex handlers_mutex_;
  std::size_t dispatch_count_{0};
  std::vector<std::weak_ptr<IAdapterPlugin>> handlers_;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__COMMAND_BUS_HPP_
