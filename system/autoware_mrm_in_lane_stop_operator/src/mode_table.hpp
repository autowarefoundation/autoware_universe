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

#ifndef MODE_TABLE_HPP_
#define MODE_TABLE_HPP_

#include "mode_config.hpp"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mrm_in_lane_stop_operator
{

/// Owns the configured ModeConfig entries and the name/id lookups over them, so
/// MrmInLaneStopOperator doesn't repeat the same linear search/loop in on_info(),
/// find_mode_by_id(), publish_driving_mode_active(), and publish_mrm_state().
class ModeTable
{
public:
  void add(ModeConfig mode) { modes_.push_back(std::move(mode)); }

  /// Records driving_mode_manager's numeric id for the configured mode named `name`.
  /// A no-op if no configured mode has that name.
  void bind_id(const std::string & name, uint32_t id);

  /// Returns the configured mode currently bound to `id`, or nullptr if none is.
  ModeConfig * find_by_id(uint32_t id);
  const ModeConfig * find_by_id(uint32_t id) const;

  auto begin() const { return modes_.begin(); }
  auto end() const { return modes_.end(); }

private:
  std::vector<ModeConfig> modes_;
};

}  // namespace autoware::mrm_in_lane_stop_operator

#endif  // MODE_TABLE_HPP_
