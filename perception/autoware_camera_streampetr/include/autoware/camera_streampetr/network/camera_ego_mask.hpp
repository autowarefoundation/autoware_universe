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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_EGO_MASK_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_EGO_MASK_HPP_

#include <cuda_runtime_api.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::camera_streampetr
{

struct EgoMaskPolygon
{
  std::vector<double> points;
  bool normalized{false};
};

struct EgoMaskRoiConfig
{
  std::vector<EgoMaskPolygon> polygons;
  std::array<std::uint8_t, 3> fill_bgr{0, 0, 0};
};

struct EgoMaskParams
{
  bool enabled{false};
  std::array<std::uint8_t, 3> fill_bgr{0, 0, 0};
  std::vector<std::string> roi_polygons_yaml;
};

std::vector<std::optional<EgoMaskRoiConfig>> loadEgoMaskRoiConfigs(
  const EgoMaskParams & params, std::size_t rois_number);

std::vector<EgoMaskPolygon> parsePolygonsYamlFile(const std::string & path);

std::vector<EgoMaskPolygon> parsePolygonsYamlText(const std::string & yaml_text);

/** Build UINT8 mask (255 inside polygons, 0 elsewhere) at full image resolution. */
std::vector<std::uint8_t> buildEgoMaskRaster(
  const std::vector<EgoMaskPolygon> & polygons, int width, int height);

cudaError_t applyEgoMask_launch(
  std::uint8_t * image_bgr, const std::uint8_t * mask, int height, int width,
  std::uint8_t fill_b, std::uint8_t fill_g, std::uint8_t fill_r, cudaStream_t stream);

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__CAMERA_EGO_MASK_HPP_
