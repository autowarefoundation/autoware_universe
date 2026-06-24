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
 * @struct RoiLabel
 * @brief Per-class label entry indexed by the model's output class ID.
 */
struct RoiLabel
{
  std::string name;
  // Autoware interface class ID; unmapped_label_id when no ROI remap file was specified
  int class_id = unmapped_label_id;
  // semantic segmentation label ID; unmapped_label_id when no ROI-to-segmentation remap was
  // specified
  int semseg_id = unmapped_label_id;
};

/**
 * @brief Load a list of image file paths from a text file, skipping blank lines.
 *
 * @param[in] filepath path to the text file containing one image path per line
 * @return list of image file paths read from the file
 */
std::vector<std::string> load_image_list(const std::string & filepath);

/**
 * @brief Load the label and remap files and resolve them into per-class RoiLabel entries.
 *
 * The label file is mandatory. The remap files are optional: when a path is an empty string the
 * corresponding ID field in each entry is left as unmapped_label_id.
 *
 * @param[in] label_path file path of the label file for ROI (mandatory)
 * @param[in] roi_remap_path file path of the remap file for ROI (optional)
 * @param[in] roi_to_semseg_remap_path file path of the remap file for segmentation (optional)
 * @return per-class label entries indexed by model output class ID
 */
std::vector<RoiLabel> load_label_maps(
  const std::string & label_path, const std::string & roi_remap_path,
  const std::string & roi_to_semseg_remap_path);

/**
 * @brief Load the segmentation color map file.
 *
 * @param[in] file_name file path of the color map file for segmentation (optional;
 * returns an empty vector when the path is an empty string)
 * @return color map entries indexed by segmentation label ID, or empty when path is empty
 */
std::vector<Colormap> load_segmentation_colormap(const std::string & file_name);

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__LABEL_HPP_
