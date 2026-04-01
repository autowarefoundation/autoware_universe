// Copyright 2025 TIER IV.
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

#include "utils/param_loader.hpp"
#include "utils/version_checker.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace autoware::tensorrt_vad
{
namespace
{
// Writes a minimal valid model parameter JSON file for testing
std::filesystem::path write_valid_param_json(const std::string & name_prefix)
{
  const auto tmp_dir = std::filesystem::temp_directory_path();
  const auto json_path = tmp_dir / (name_prefix + "_vad_param.json");

  const std::string json_content = R"JSON({
    "major_version": 0,
    "minor_version": 1,
    "model_name": "vad-carla-tiny",
    "image_normalization": {
      "mean": [0.485, 0.456, 0.406],
      "std": [0.229, 0.224, 0.225]
    },
    "input_specs": {
      "target_image_width": 640,
      "target_image_height": 384
    },
    "class_definitions": {
      "map_classes": ["lane", "stopline"],
      "object_classes": ["car", "pedestrian"]
    },
    "network_architecture": {
      "bev": {
        "height": 200,
        "width": 200,
        "feature_dim": 64,
        "downsample_factor": 4
      },
      "transformer": {
        "num_decoder_layers": 4
      },
      "prediction": {
        "num_queries": 64,
        "num_classes": 10,
        "bbox_pred_dim": 8,
        "trajectory_modes": 3,
        "timesteps": 6
      },
      "planning": {
        "ego_commands": 2,
        "timesteps": 6
      },
      "map": {
        "num_queries": 32,
        "num_classes": 5,
        "points_per_polyline": 8
      },
      "vehicle_state": {
        "can_bus_dim": 16
      }
    }
  })JSON";

  std::ofstream ofs(json_path);
  ofs << json_content;
  return json_path;
}
}  // namespace

TEST(ParamLoader, LoadsValidFile)
{
  const auto json_path = write_valid_param_json("valid");

  const auto params = utils::load_model_params(json_path.string());

  EXPECT_EQ(params.major_version, 0);
  EXPECT_EQ(params.minor_version, 1);
  EXPECT_EQ(params.model_name, "vad-carla-tiny");
  EXPECT_FLOAT_EQ(params.image_normalization_mean[0], 0.485F);
  EXPECT_EQ(params.map_classes.size(), 2U);
  EXPECT_EQ(params.object_classes.size(), 2U);
  EXPECT_EQ(params.bev_height, 200);
  EXPECT_EQ(params.bev_width, 200);
  EXPECT_EQ(params.prediction_num_queries, 64);
  EXPECT_EQ(params.map_points_per_polyline, 8);
  EXPECT_EQ(params.can_bus_dim, 16);
}

TEST(ParamLoader, ThrowsOnMissingFile)
{
  EXPECT_THROW(utils::load_model_params("/tmp/does_not_exist_vad_param.json"), std::runtime_error);
}

TEST(VersionChecker, AcceptsSupportedVersion)
{
  const auto json_path = write_valid_param_json("supported_version");
  EXPECT_NO_THROW(utils::check_model_version(json_path.string()));
}

TEST(VersionChecker, RejectsUnsupportedVersion)
{
  const auto tmp_dir = std::filesystem::temp_directory_path();
  const auto json_path = tmp_dir / "unsupported_vad_param.json";

  const std::string json_content = R"JSON({
    "major_version": 999,
    "minor_version": 0,
    "model_name": "vad-carla-tiny"
  })JSON";

  std::ofstream ofs(json_path);
  ofs << json_content;

  EXPECT_THROW(utils::check_model_version(json_path.string()), std::runtime_error);
}

TEST(VersionChecker, RejectsMissingMajorVersion)
{
  const auto tmp_dir = std::filesystem::temp_directory_path();
  const auto json_path = tmp_dir / "missing_major_vad_param.json";

  const std::string json_content = R"JSON({
    "minor_version": 0,
    "model_name": "vad-carla-tiny"
  })JSON";

  std::ofstream ofs(json_path);
  ofs << json_content;

  EXPECT_THROW(utils::check_model_version(json_path.string()), std::runtime_error);
}

}  // namespace autoware::tensorrt_vad

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
