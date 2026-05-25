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

#include "autoware/camera_streampetr/network/camera_ego_mask.hpp"

#include <opencv2/imgproc.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstring>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace autoware::camera_streampetr
{

namespace
{

std::string loadTextFile(const std::string & path)
{
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Could not open polygons YAML file: " + path);
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

bool trimEmpty(const std::string & s)
{
  return s.find_first_not_of(" \t\n\r\f\v") == std::string::npos;
}

std::array<std::uint8_t, 3> clampFillBgr(const std::array<std::uint8_t, 3> & fill)
{
  return fill;
}

}  // namespace

std::vector<EgoMaskPolygon> parsePolygonsYamlText(const std::string & yaml_text)
{
  if (yaml_text.empty() || trimEmpty(yaml_text)) {
    return {};
  }

  YAML::Node root = YAML::Load(yaml_text);
  if (!root.IsMap()) {
    throw std::runtime_error("polygons YAML: root must be a mapping (e.g. 'polygons: [...]').");
  }

  std::vector<bool> normalized_parallel;
  if (root["polygons_normalized"]) {
    const YAML::Node nn = root["polygons_normalized"];
    if (!nn.IsSequence()) {
      throw std::runtime_error(
        "polygons YAML: 'polygons_normalized' must be a sequence of booleans.");
    }
    for (const auto & item : nn) {
      normalized_parallel.push_back(item.as<bool>());
    }
  }

  const YAML::Node polys = root["polygons"];
  if (!polys.IsDefined() || polys.IsNull()) {
    return {};
  }
  if (!polys.IsSequence()) {
    throw std::runtime_error("polygons YAML: 'polygons' must be a sequence.");
  }

  if (!normalized_parallel.empty() && normalized_parallel.size() != polys.size()) {
    throw std::runtime_error(
      "polygons YAML: 'polygons_normalized' must be the same length as 'polygons' when provided.");
  }

  std::vector<EgoMaskPolygon> out;
  out.reserve(polys.size());
  for (std::size_t i = 0; i < polys.size(); ++i) {
    const YAML::Node & pn = polys[i];
    EgoMaskPolygon spec{};

    if (pn.IsSequence()) {
      for (const auto & v : pn) {
        spec.points.push_back(v.as<double>());
      }
      if (i < normalized_parallel.size()) {
        spec.normalized = normalized_parallel[i];
      }
    } else if (pn.IsMap()) {
      if (!pn["points"] || !pn["points"].IsSequence()) {
        throw std::runtime_error("polygons YAML: each map entry must have a 'points' sequence.");
      }
      for (const auto & v : pn["points"]) {
        spec.points.push_back(v.as<double>());
      }
      if (pn["normalized"]) {
        spec.normalized = pn["normalized"].as<bool>();
      } else if (i < normalized_parallel.size()) {
        spec.normalized = normalized_parallel[i];
      }
    } else {
      throw std::runtime_error(
        "polygons YAML: each polygon must be a number sequence or a map with 'points'.");
    }

    if (spec.points.size() < 6 || (spec.points.size() % 2) != 0) {
      throw std::runtime_error(
        "polygons YAML: each polygon must have an even length >= 6 (at least 3 (x,y) points).");
    }
    out.push_back(std::move(spec));
  }
  return out;
}

std::vector<EgoMaskPolygon> parsePolygonsYamlFile(const std::string & path)
{
  return parsePolygonsYamlText(loadTextFile(path));
}

std::vector<std::optional<EgoMaskRoiConfig>> loadEgoMaskRoiConfigs(
  const EgoMaskParams & params, const std::size_t rois_number)
{
  std::vector<std::optional<EgoMaskRoiConfig>> configs(rois_number, std::nullopt);

  if (!params.enabled) {
    return configs;
  }

  const auto fill = clampFillBgr(params.fill_bgr);

  for (std::size_t i = 0; i < rois_number; ++i) {
    if (i >= params.roi_polygons_yaml.size()) {
      continue;
    }
    const std::string & yaml_path = params.roi_polygons_yaml[i];
    if (yaml_path.empty() || trimEmpty(yaml_path)) {
      continue;
    }

    EgoMaskRoiConfig cfg;
    cfg.polygons = parsePolygonsYamlFile(yaml_path);
    cfg.fill_bgr = fill;
    if (!cfg.polygons.empty()) {
      configs[i] = std::move(cfg);
    }
  }

  return configs;
}

std::vector<std::uint8_t> buildEgoMaskRaster(
  const std::vector<EgoMaskPolygon> & polygons, const int width, const int height)
{
  if (polygons.empty() || width <= 0 || height <= 0) {
    return {};
  }

  std::vector<std::vector<cv::Point>> polys;
  polys.reserve(polygons.size());

  for (const auto & poly : polygons) {
    std::vector<cv::Point> pts;
    pts.reserve(poly.points.size() / 2);
    for (std::size_t j = 0; j < poly.points.size(); j += 2) {
      double x = poly.points[j];
      double y = poly.points[j + 1];
      if (poly.normalized) {
        x *= static_cast<double>(width);
        y *= static_cast<double>(height);
      }
      pts.emplace_back(static_cast<int>(x), static_cast<int>(y));
    }
    polys.push_back(std::move(pts));
  }

  cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
  cv::fillPoly(mask, polys, cv::Scalar(255), cv::LINE_AA);

  std::vector<std::uint8_t> raster(static_cast<std::size_t>(width * height));
  if (mask.isContinuous()) {
    std::memcpy(raster.data(), mask.data, raster.size());
  } else {
    for (int y = 0; y < height; ++y) {
      std::memcpy(
        raster.data() + static_cast<std::size_t>(y * width), mask.ptr(y),
        static_cast<std::size_t>(width));
    }
  }
  return raster;
}

}  // namespace autoware::camera_streampetr
