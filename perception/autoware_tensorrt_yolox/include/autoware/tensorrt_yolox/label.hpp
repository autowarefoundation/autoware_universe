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

#ifndef AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_
#define AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_

#include <string>
#include <unordered_map>
#include <vector>

// cspell: ignore semseg

namespace autoware::tensorrt_yolox
{
typedef struct Colormap_
{
  int id;
  std::string name;
  std::vector<unsigned char> color;
} Colormap;

// label ID used for ROI classes that are not mapped to any output class
inline constexpr int unmapped_label_id = -1;

/**
 * @struct LabelMaps
 * @brief Label data loaded from disk and resolved into ready-to-use lookup tables. This is the
 * structured form of the label-related files, so that consumers (e.g. TrtYoloXDetector) can be
 * constructed from in-memory data instead of file paths, decoupling them from file I/O.
 */
struct LabelMaps
{
  // ROI class names indexed by the model's output class ID (from the label file)
  std::vector<std::string> roi_class_name_list;
  // Autoware interface class ID indexed by the model's output class ID. Always sized to
  // roi_class_name_list; entries are unmapped_label_id when no ROI remap file was specified.
  std::vector<int> roi_id_to_class_id_map;
  // semantic segmentation color map (from the color map file; empty when not specified)
  std::vector<autoware::tensorrt_yolox::Colormap> semseg_color_map;
  // semantic segmentation label ID indexed by the model's output class ID. Empty when no
  // ROI-to-segmentation remap file was specified.
  std::vector<int> roi_id_to_semseg_id_map;
};

// Load a list of image file paths from a text file, prefixing entries that cannot be found as-is.
std::vector<std::string> load_image_list(const std::string & filename, const std::string & prefix);

/**
 * @brief Load the label, remap and color-map files and resolve them into ready-to-use lookup
 * tables.
 *
 * The label file is mandatory. The remap and color-map files are optional: when a path is an empty
 * string the corresponding lookup table is left in its "not specified" form (see LabelMaps). The
 * remap files are applied here so that the returned LabelMaps can be used directly without further
 * processing.
 *
 * @param[in] label_path file path of the label file for ROI (mandatory)
 * @param[in] semseg_color_map_path file path of the color map file for segmentation (optional)
 * @param[in] roi_remap_path file path of the remap file for ROI (optional)
 * @param[in] roi_to_semseg_remap_path file path of the remap file for segmentation (optional)
 * @return resolved label lookup tables
 */
LabelMaps load_label_maps(
  const std::string & label_path, const std::string & semseg_color_map_path,
  const std::string & roi_remap_path, const std::string & roi_to_semseg_remap_path);

/**
 * @brief Build a mapping from the model's ROI class ID to a target label ID using a name-based
 * remap.
 *
 * When the remap is empty, every entry is set to @p unmapped_id (i.e. remapping is disabled). When
 * the remap is non-empty, every ROI class name must be present in it; otherwise a
 * std::runtime_error is thrown, since a missing entry usually indicates that a wrong model is being
 * used.
 *
 * @param[in] roi_class_name_list ROI class names indexed by the model's output class ID
 * @param[in] label_name_to_target_id remap from label name to target ID (empty disables remapping)
 * @param[in] unmapped_id value used for unmapped entries
 * @return target label ID indexed by the model's output class ID
 */
std::vector<int> build_roi_id_to_target_id_map(
  const std::vector<std::string> & roi_class_name_list,
  const std::unordered_map<std::string, int> & label_name_to_target_id, int unmapped_id);

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_
