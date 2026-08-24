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
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

namespace
{

std::string read_config(const std::string & path)
{
  std::ifstream stream(path, std::ios::in | std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Could not open polygons YAML file: " + path);
  }
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

bool trim_empty(const std::string & s)
{
  return s.find_first_not_of(" \t\n\r\f\v") == std::string::npos;
}

YAML::Node parse_yaml_root(const std::string & yaml_text)
{
  YAML::Node root = YAML::Load(yaml_text);
  if (!root.IsMap()) {
    throw std::runtime_error("polygons YAML: root must be a mapping (e.g. 'polygons: [...]').");
  }
  return root;
}

YAML::Node get_polygons_node(const YAML::Node & root)
{
  const YAML::Node polygons = root["polygons"];
  if (!polygons.IsDefined() || polygons.IsNull()) {
    return {};
  }
  if (!polygons.IsSequence()) {
    throw std::runtime_error("polygons YAML: 'polygons' must be a sequence.");
  }
  return polygons;
}

std::vector<bool> parse_normalized_flags(const YAML::Node & root, const std::size_t polygons_count)
{
  std::vector<bool> normalized_parallel;
  const YAML::Node normalized_node = root["polygons_normalized"];
  if (!normalized_node) {
    return normalized_parallel;
  }

  if (!normalized_node.IsSequence()) {
    throw std::runtime_error(
      "polygons YAML: 'polygons_normalized' must be a sequence of booleans.");
  }
  if (normalized_node.size() != polygons_count) {
    throw std::runtime_error(
      "polygons YAML: 'polygons_normalized' must be the same length as 'polygons' when provided.");
  }

  normalized_parallel.reserve(polygons_count);
  for (const auto & item : normalized_node) {
    normalized_parallel.push_back(item.as<bool>());
  }
  return normalized_parallel;
}

std::vector<double> parse_polygon_points(const YAML::Node & points_node)
{
  const std::size_t points_count = points_node.size();
  if (points_count < 6 || (points_count % 2) != 0) {
    throw std::runtime_error(
      "polygons YAML: each polygon must have an even length >= 6 (at least 3 (x,y) points).");
  }

  std::vector<double> points;
  points.reserve(points_count);
  for (const auto & value : points_node) {
    points.push_back(value.as<double>());
  }
  return points;
}

EgoMaskPolygon parse_polygon_node(
  const YAML::Node & polygon_node, const std::vector<bool> & normalized_parallel,
  const std::size_t polygon_index)
{
  bool normalized = !normalized_parallel.empty() && normalized_parallel[polygon_index];
  YAML::Node points_node = polygon_node;

  if (polygon_node.IsSequence()) {
    points_node = polygon_node;
  } else if (polygon_node.IsMap()) {
    points_node = polygon_node["points"];
    if (!points_node || !points_node.IsSequence()) {
      throw std::runtime_error("polygons YAML: each map entry must have a 'points' sequence.");
    }
    if (polygon_node["normalized"]) {
      normalized = polygon_node["normalized"].as<bool>();
    }
  } else {
    throw std::runtime_error(
      "polygons YAML: each polygon must be a number sequence or a map with 'points'.");
  }

  EgoMaskPolygon spec;
  spec.points = parse_polygon_points(points_node);
  spec.normalized = normalized;
  return spec;
}

std::vector<cv::Point> to_cv_polygon(
  const EgoMaskPolygon & polygon, const int width, const int height)
{
  const double x_scale = polygon.normalized ? static_cast<double>(width) : 1.0;
  const double y_scale = polygon.normalized ? static_cast<double>(height) : 1.0;

  std::vector<cv::Point> points;
  points.reserve(polygon.points.size() / 2);
  for (std::size_t i = 0; i < polygon.points.size(); i += 2) {
    points.emplace_back(
      static_cast<int>(polygon.points[i] * x_scale),
      static_cast<int>(polygon.points[i + 1] * y_scale));
  }
  return points;
}

std::vector<std::vector<cv::Point>> to_cv_polygons(
  const std::vector<EgoMaskPolygon> & polygons, const int width, const int height)
{
  std::vector<std::vector<cv::Point>> cv_polygons;
  cv_polygons.reserve(polygons.size());
  for (const auto & polygon : polygons) {
    cv_polygons.push_back(to_cv_polygon(polygon, width, height));
  }
  return cv_polygons;
}

std::vector<std::uint8_t> copy_mask_to_raster(const cv::Mat & mask)
{
  const int width = mask.cols;
  const int height = mask.rows;
  std::vector<std::uint8_t> raster(static_cast<std::size_t>(width * height));

  if (mask.isContinuous()) {
    std::memcpy(raster.data(), mask.data, raster.size());
    return raster;
  }

  for (int y = 0; y < height; ++y) {
    std::memcpy(
      raster.data() + static_cast<std::size_t>(y * width), mask.ptr(y),
      static_cast<std::size_t>(width));
  }
  return raster;
}

}  // namespace

std::vector<EgoMaskPolygon> parse_polygons_yaml_text(const std::string & yaml_text)
{
  if (yaml_text.empty() || trim_empty(yaml_text)) {
    return {};
  }

  const YAML::Node root = parse_yaml_root(yaml_text);
  const YAML::Node polygons = get_polygons_node(root);
  if (!polygons.IsDefined()) {
    return {};
  }

  const auto normalized_parallel = parse_normalized_flags(root, polygons.size());
  std::vector<EgoMaskPolygon> out;
  out.reserve(polygons.size());
  for (std::size_t i = 0; i < polygons.size(); ++i) {
    out.push_back(parse_polygon_node(polygons[i], normalized_parallel, i));
  }
  return out;
}

std::vector<EgoMaskPolygon> parse_polygons_yaml_file(const std::string & path)
{
  return parse_polygons_yaml_text(read_config(path));
}

std::vector<std::optional<EgoMaskRoiConfig>> load_ego_mask_roi_configs(
  const EgoMaskParams & params, const std::size_t rois_number)
{
  std::vector<std::optional<EgoMaskRoiConfig>> configs(rois_number, std::nullopt);

  for (std::size_t i = 0; i < rois_number && i < params.roi_mask_configs.size(); ++i) {
    configs[i] = params.roi_mask_configs[i];
  }

  if (!params.enabled) {
    return configs;
  }

  const auto & fill = params.fill_rgb;

  for (std::size_t i = 0; i < rois_number; ++i) {
    if (i >= params.roi_polygons_yaml.size()) {
      continue;
    }
    const std::string & yaml_path = params.roi_polygons_yaml[i];
    if (yaml_path.empty() || trim_empty(yaml_path)) {
      continue;
    }

    EgoMaskRoiConfig cfg;
    cfg.polygons = parse_polygons_yaml_file(yaml_path);
    cfg.fill_rgb = fill;
    if (!cfg.polygons.empty()) {
      configs[i] = std::move(cfg);
    }
  }

  return configs;
}

std::vector<std::uint8_t> build_ego_mask_raster(
  const std::vector<EgoMaskPolygon> & polygons, const int width, const int height)
{
  if (polygons.empty() || width <= 0 || height <= 0) {
    return {};
  }

  cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
  cv::fillPoly(mask, to_cv_polygons(polygons, width, height), cv::Scalar(255), cv::LINE_AA);

  return copy_mask_to_raster(mask);
}

}  // namespace autoware::camera_streampetr
