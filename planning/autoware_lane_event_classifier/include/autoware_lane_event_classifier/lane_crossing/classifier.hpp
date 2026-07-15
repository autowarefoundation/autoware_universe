// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_

#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/types.hpp>

#include <cstdint>
#include <string>

namespace autoware::lane_event_classifier
{

/**
 * @brief Recognises an intentional lane crossing and reports its state.
 *
 * @note Stub: the classification logic (and its tracker inputs) are added in a follow-up PR. Every
 * cycle it reports LANE_FOLLOWING, i.e. no event, so it never overrides the node's published
 * state. It exists here to exercise the classifier-loading path end-to-end.
 */
class IntentionalCrossingClassifier : public LaneEventClassifierBase
{
public:
  explicit IntentionalCrossingClassifier(bool enabled) : enabled_{enabled} {}

  void update(const LaneEventInput & /*input*/) override {}

  [[nodiscard]] uint8_t get_state() const override { return DrivingState::LANE_FOLLOWING; }

  [[nodiscard]] bool is_enabled() const override { return enabled_; }

  [[nodiscard]] std::string name() const override { return "lane_crossing"; }

private:
  bool enabled_;
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
