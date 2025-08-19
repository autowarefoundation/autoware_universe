// Copyright 2025 TIER IV, inc.
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
#include "test_bench_merge.hpp"

#include <string>
#include <unordered_map>
#include <vector>
void MergeTestBench::initializeObjects(const TrackingScenarioConfig & params)
{
  // lanes_y could be [-2.0, 0.0, 2.0] for example
  std::vector<float> lanes_y = {-4.0f, 0.0f, 4.0f};

  int car_id = 0;
  for (float lane_y : lanes_y) {
    // For each lane: spawn 3 cars with different speeds (static, medium, fast)
    std::vector<float> speeds = {0.0f, 5.0f, 15.0f};
    int idx = 0;
    for (float v : speeds) {
      std::string cid = "car_" + std::to_string(car_id);

      ObjectState car_state;
      car_state.pose.position.x = -20.0f + idx * 15.0f;  // stagger start x
      car_state.pose.position.y = lane_y;
      car_state.pose.position.z = 0.0f;
      car_state.twist.linear.x = v;
      car_state.twist.linear.y = 0.0f;
      car_state.shape.x = 4.5f;
      car_state.shape.y = 1.8f;

      car_states_[cid] = car_state;

      // Attach an unknown object to this car
      float size_x = (idx % 2 == 0) ? 0.5f : 4.0f;
      float size_y = (idx % 2 == 0) ? 0.5f : 2.5f;
      addNewUnknown("unk_" + cid, car_state, size_x, size_y);

      car_id++;
      idx++;
    }
  }
}

void MergeTestBench::addNewUnknown(
  const std::string & uid, const ObjectState & car_state, float size_x, float size_y)
{
  ObjectState state;
  state.pose.position.x = car_state.pose.position.x + 1.0f;
  state.pose.position.y = car_state.pose.position.y + 1.0f;
  state.pose.position.z = 0.5f;
  state.twist.linear.x = 0.0f;
  state.twist.linear.y = 0.0f;
  state.shape.x = size_x;
  state.shape.y = size_y;

  unknown_states_[uid] = state;
  car_to_unknown_map_[uid] = uid.substr(4);  // map unknown -> car
}

autoware::multi_object_tracker::types::DynamicObjectList MergeTestBench::generateDetections(
  const rclcpp::Time & stamp)
{
  auto detections = TrackingTestBench::generateDetections(stamp);

  // Update unknowns to follow their associated car
  for (auto & [uid, state] : unknown_states_) {
    const auto car_id = car_to_unknown_map_[uid];
    if (car_states_.count(car_id)) {
      const auto & car = car_states_.at(car_id);
      state.pose.position.x = car.pose.position.x + 1.0f;
      state.pose.position.y = car.pose.position.y + 1.0f;
    }

    autoware::multi_object_tracker::types::DynamicObject obj;
    obj.uuid.uuid = stringToUUID(uid);
    obj.time = stamp;
    obj.classification.emplace_back();
    obj.classification[0].label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = state.shape.x;
    obj.shape.dimensions.y = state.shape.y;
    obj.shape.dimensions.z = 1.5;

    obj.pose = state.pose;
    obj.twist = state.twist;  // always 0
    obj.existence_probability = 0.9;
    obj.channel_index = 0;
    obj.area = state.shape.x * state.shape.y;
    detections.objects.push_back(obj);
  }

  return detections;
}
