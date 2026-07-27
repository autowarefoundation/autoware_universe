// Copyright 2024 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/fence.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{
bool doesPathCrossFence(
  const lanelet::BasicLineString2d & predicted_path, const lanelet::ConstLineString3d & fence_line)
{
  return boost::geometry::intersects(
    predicted_path, lanelet::utils::to2D(fence_line.basicLineString()));
}
}  // namespace

void FenceModule::buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  lanelet::LineStrings3d fences;
  for (const auto & linestring : lanelet_map_ptr->lineStringLayer) {
    if (const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, "none");
        type == "fence") {
      fences.emplace_back(std::const_pointer_cast<lanelet::LineStringData>(linestring.constData()));
    }
  }
  fence_layer_ = lanelet::utils::createMap(fences);
}

bool FenceModule::doesPathCrossAnyFenceBeforeCrosswalk(
  const PredictedPathWithArrivalIndex & predicted_path) const
{
  if (!fence_layer_) {
    return false;
  }
  if (predicted_path.path.empty()) return false;
  const size_t last_idx = std::min(predicted_path.arrival_index, predicted_path.path.size() - 1);
  lanelet::BasicLineString2d predicted_path_ls;
  for (auto i = 0UL; i <= last_idx; ++i) {
    const auto & pt = predicted_path.path[i];
    predicted_path_ls.emplace_back(pt.position.x, pt.position.y);
  }
  const auto candidates =
    fence_layer_->lineStringLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  for (const auto & candidate : candidates) {
    if (doesPathCrossFence(predicted_path_ls, candidate)) {
      return true;
    }
  }
  return false;
}

PredictedPath FenceModule::cutPathBeforeFences(const PredictedPath & predicted_path) const
{
  if (!fence_layer_) {
    return predicted_path;
  }
  const auto & path = predicted_path.path;
  if (path.size() < 2) {
    return predicted_path;
  }
  lanelet::BasicLineString2d predicted_path_ls;
  for (const auto & pt : path) {
    predicted_path_ls.emplace_back(pt.position.x, pt.position.y);
  }
  const auto candidates =
    fence_layer_->lineStringLayer.search(lanelet::geometry::boundingBox2d(predicted_path_ls));
  std::vector<lanelet::ConstLineString3d> crossed_fences{};
  for (const auto & candidate : candidates) {
    if (doesPathCrossFence(predicted_path_ls, candidate)) {
      crossed_fences.push_back(candidate);
      break;
    }
  }
  if (crossed_fences.empty()) {
    return predicted_path;
  }

  std::optional<size_t> closest_cross_index{};
  for (auto i = 0UL; i + 1 < predicted_path_ls.size() && !closest_cross_index.has_value(); ++i) {
    lanelet::BasicLineString2d path_segment(
      lanelet::BasicPoints2d{predicted_path_ls[i], predicted_path_ls[i + 1]});
    for (const auto & fence : crossed_fences) {
      if (boost::geometry::intersects(
            path_segment, lanelet::utils::to2D(fence).basicLineString())) {
        closest_cross_index = i;
      }
    }
  }

  if (!closest_cross_index) {
    return predicted_path;
  }
  auto trimmed_path = predicted_path;
  trimmed_path.path.clear();
  for (unsigned i = 0; i <= closest_cross_index.value(); ++i) {
    trimmed_path.path.push_back(path.at(i));
  }
  return trimmed_path;
}

}  // namespace autoware::map_based_prediction
