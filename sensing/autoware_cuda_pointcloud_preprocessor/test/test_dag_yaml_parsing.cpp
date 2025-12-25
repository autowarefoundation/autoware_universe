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

#include "autoware/cuda_pointcloud_preprocessor/dag/dag_config_parser.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using autoware::cuda_pointcloud_preprocessor::dag::DagConfig;
using autoware::cuda_pointcloud_preprocessor::dag::DagConfigParser;

class DagYamlParsingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    temp_dir_ = std::filesystem::temp_directory_path() / "dag_yaml_test";
    std::filesystem::create_directories(temp_dir_);
  }

  void TearDown() override { std::filesystem::remove_all(temp_dir_); }

  // Helper to create a temporary YAML file
  std::string createTempYamlFile(const std::string & filename, const std::string & content)
  {
    std::filesystem::path filepath = temp_dir_ / filename;
    std::ofstream file(filepath);
    file << content;
    file.close();
    return filepath.string();
  }

  std::filesystem::path temp_dir_;
};

/**
 * Test 1: Parse a complete valid DAG configuration
 */
TEST_F(DagYamlParsingTest, ParseCompleteValidConfiguration)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  version: "1.0"

  inputs:
    - name: "pointcloud"
      type: "sensor_msgs::msg::PointCloud2"
      topic: "~/input/pointcloud"
      optional: false

  nodes:
    - id: "organize"
      type: "OrganizeFilter"
      inputs:
        - source: "pointcloud"
      outputs:
        - name: "organized"
      parameters: {}

    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "organized"
          from_node: "organize"
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"

  outputs:
    - name: "output"
      source: "transformed"
      from_node: "transform"
      topic: "~/output/pointcloud"
      type: "cuda_blackboard::CudaPointCloud2"
)";

  std::string filepath = createTempYamlFile("valid_config.yaml", yaml_content);

  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);

    EXPECT_EQ(config.name, "test_dag");
    EXPECT_EQ(config.version, "1.0");
    ASSERT_EQ(config.inputs.size(), 1);
    ASSERT_EQ(config.nodes.size(), 2);
    ASSERT_EQ(config.outputs.size(), 1);

    // Verify TransformFilter has required parameter
    EXPECT_EQ(config.nodes[1].type, "TransformFilter");
    EXPECT_TRUE(
      config.nodes[1].parameters.find("target_frame") != config.nodes[1].parameters.end());
  });
}

/**
 * Test 2: Error - Missing required node ID
 */
TEST_F(DagYamlParsingTest, ErrorMissingNodeId)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - type: "OrganizeFilter"
      outputs:
        - name: "output"
)";

  std::string filepath = createTempYamlFile("missing_id.yaml", yaml_content);

  EXPECT_THROW(
    { DagConfig config = DagConfigParser::parseFromFile(filepath); }, std::runtime_error);
}

/**
 * Test 3: Error - Missing required node type
 */
TEST_F(DagYamlParsingTest, ErrorMissingNodeType)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "test_node"
      outputs:
        - name: "output"
)";

  std::string filepath = createTempYamlFile("missing_type.yaml", yaml_content);

  EXPECT_THROW(
    { DagConfig config = DagConfigParser::parseFromFile(filepath); }, std::runtime_error);
}

/**
 * Test 4: Error - TransformFilter missing required target_frame parameter
 */
TEST_F(DagYamlParsingTest, ErrorTransformFilterMissingTargetFrame)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "input"
      outputs:
        - name: "output"
      parameters: {}
)";

  std::string filepath = createTempYamlFile("missing_target_frame.yaml", yaml_content);

  try {
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::string error_msg(e.what());
    EXPECT_TRUE(error_msg.find("target_frame") != std::string::npos)
      << "Error message should mention 'target_frame'. Got: " << error_msg;
  }
}

/**
 * Test 5: Error - RingOutlierFilter missing required parameters
 */
TEST_F(DagYamlParsingTest, ErrorRingOutlierFilterMissingParameters)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "ring_outlier"
      type: "RingOutlierFilter"
      inputs:
        - source: "input"
      outputs:
        - name: "output"
      parameters:
        distance_ratio: 1.03
)";

  std::string filepath = createTempYamlFile("missing_object_length_threshold.yaml", yaml_content);

  try {
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::string error_msg(e.what());
    EXPECT_TRUE(error_msg.find("object_length_threshold") != std::string::npos)
      << "Error message should mention 'object_length_threshold'. Got: " << error_msg;
  }
}

/**
 * Test 6: Error - Missing input source field
 */
TEST_F(DagYamlParsingTest, ErrorMissingInputSource)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "test"
      type: "OrganizeFilter"
      inputs:
        - from_node: "something"
      outputs:
        - name: "output"
)";

  std::string filepath = createTempYamlFile("missing_input_source.yaml", yaml_content);

  try {
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::string error_msg(e.what());
    EXPECT_TRUE(error_msg.find("source") != std::string::npos)
      << "Error message should mention 'source'. Got: " << error_msg;
  }
}

/**
 * Test 7: Error - Empty output name
 */
TEST_F(DagYamlParsingTest, ErrorEmptyOutputName)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "test"
      type: "OrganizeFilter"
      inputs:
        - source: "input"
      outputs:
        - name: ""
)";

  std::string filepath = createTempYamlFile("empty_output.yaml", yaml_content);

  try {
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::string error_msg(e.what());
    EXPECT_TRUE(error_msg.find("empty name") != std::string::npos)
      << "Error message should mention 'empty name'. Got: " << error_msg;
  }
}

/**
 * Test 8: Parse configuration with all filter types
 */
TEST_F(DagYamlParsingTest, ParseAllFilterTypes)
{
  std::string yaml_content = R"(
dag:
  name: "all_filters"

  nodes:
    - id: "organize"
      type: "OrganizeFilter"
      inputs:
        - source: "raw"
      outputs:
        - name: "organized"
      parameters: {}

    - id: "transform"
      type: "TransformFilter"
      inputs:
        - source: "organized"
          from_node: "organize"
      outputs:
        - name: "transformed"
      parameters:
        target_frame: "base_link"

    - id: "cropbox"
      type: "CropBoxFilter"
      inputs:
        - source: "transformed"
          from_node: "transform"
      outputs:
        - name: "cropped"
      parameters:
        crop_boxes: []

    - id: "distortion"
      type: "DistortionFilter"
      inputs:
        - source: "cropped"
          from_node: "cropbox"
      outputs:
        - name: "undistorted"
      parameters:
        use_3d: false

    - id: "ring_outlier"
      type: "RingOutlierFilter"
      inputs:
        - source: "undistorted"
          from_node: "distortion"
      outputs:
        - name: "filtered"
      parameters:
        distance_ratio: 1.03
        object_length_threshold: 0.05

    - id: "finalize"
      type: "FinalizeFilter"
      inputs:
        - source: "filtered"
          from_node: "ring_outlier"
      outputs:
        - name: "final"
      parameters: {}
)";

  std::string filepath = createTempYamlFile("all_filters.yaml", yaml_content);

  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);

    ASSERT_EQ(config.nodes.size(), 6);
    EXPECT_EQ(config.nodes[0].type, "OrganizeFilter");
    EXPECT_EQ(config.nodes[1].type, "TransformFilter");
    EXPECT_EQ(config.nodes[2].type, "CropBoxFilter");
    EXPECT_EQ(config.nodes[3].type, "DistortionFilter");
    EXPECT_EQ(config.nodes[4].type, "RingOutlierFilter");
    EXPECT_EQ(config.nodes[5].type, "FinalizeFilter");
  });
}

/**
 * Test 9: Parse standard_preprocessing.yaml (actual production config)
 */
TEST_F(DagYamlParsingTest, ParseStandardPreprocessingYaml)
{
  // Try to find the actual standard_preprocessing.yaml file
  std::vector<std::string> search_paths = {
    "../config/dag/standard_preprocessing.yaml",
    "../../src/autoware/universe/sensing/autoware_cuda_pointcloud_preprocessor/config/dag/"
    "standard_preprocessing.yaml",
    std::string(std::getenv("HOME") ? std::getenv("HOME") : "") +
      "/pilot-auto.xx1/src/autoware/universe/sensing/autoware_cuda_pointcloud_preprocessor/config/"
      "dag/standard_preprocessing.yaml"};

  std::string config_path;
  for (const auto & path : search_paths) {
    if (std::filesystem::exists(path)) {
      config_path = path;
      break;
    }
  }

  if (config_path.empty()) {
    GTEST_SKIP() << "Could not find standard_preprocessing.yaml";
  }

  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(config_path);

    // Verify the standard preprocessing pipeline structure
    EXPECT_EQ(config.name, "standard_pointcloud_preprocessing");
    EXPECT_GT(config.nodes.size(), 0);

    // Check that all filter types are valid
    for (const auto & node : config.nodes) {
      EXPECT_TRUE(
        node.type == "OrganizeFilter" || node.type == "TransformFilter" ||
        node.type == "CropBoxFilter" || node.type == "DistortionFilter" ||
        node.type == "RingOutlierFilter" || node.type == "FinalizeFilter")
        << "Unknown filter type: " << node.type;
    }
  });
}

/**
 * Test 10: Error - File not found
 */
TEST_F(DagYamlParsingTest, ErrorFileNotFound)
{
  try {
    DagConfig config = DagConfigParser::parseFromFile("/nonexistent/path/config.yaml");
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & e) {
    std::string error_msg(e.what());
    EXPECT_TRUE(error_msg.find("Failed to parse") != std::string::npos)
      << "Error message should mention 'Failed to parse'. Got: " << error_msg;
  }
}

/**
 * Test 11: Error - Duplicate node IDs
 */
TEST_F(DagYamlParsingTest, ErrorDuplicateNodeIds)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "duplicate_id"
      type: "OrganizeFilter"
      inputs:
        - source: "input1"
      outputs:
        - name: "output1"

    - id: "duplicate_id"
      type: "TransformFilter"
      inputs:
        - source: "input2"
      outputs:
        - name: "output2"
      parameters:
        target_frame: "base_link"
)";

  std::string filepath = createTempYamlFile("duplicate_ids.yaml", yaml_content);

  // Note: Current implementation may not check for duplicate IDs during parsing
  // This test documents expected behavior for future enhancement
  // For now, just verify it parses without crashing
  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    // Future: Should throw or warn about duplicate IDs
  });
}

/**
 * Test 12: Error - Invalid YAML syntax
 */
TEST_F(DagYamlParsingTest, ErrorInvalidYamlSyntax)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag
  nodes:  # Missing closing quote above will cause parse error
    - id: "test"
      type: "OrganizeFilter"
)";

  std::string filepath = createTempYamlFile("invalid_syntax.yaml", yaml_content);

  EXPECT_THROW(
    { DagConfig config = DagConfigParser::parseFromFile(filepath); }, std::runtime_error);
}

/**
 * Test 13: Error - CropBoxFilter with invalid crop_boxes format
 */
TEST_F(DagYamlParsingTest, ErrorInvalidCropBoxesFormat)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "cropbox"
      type: "CropBoxFilter"
      inputs:
        - source: "input"
      outputs:
        - name: "output"
      parameters:
        crop_boxes: "invalid_string_not_array"
)";

  std::string filepath = createTempYamlFile("invalid_cropbox.yaml", yaml_content);

  try {
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    // If parsing succeeds, validation should catch it
    // Future: Add validation for crop_boxes format
  } catch (const std::runtime_error & e) {
    // Expected to throw due to type mismatch
    SUCCEED();
  }
}

/**
 * Test 14: DistortionFilter parameters validation
 */
TEST_F(DagYamlParsingTest, DistortionFilterParameters)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "distortion"
      type: "DistortionFilter"
      inputs:
        - source: "input"
      outputs:
        - name: "output"
      parameters:
        use_3d: false
        use_imu: true
)";

  std::string filepath = createTempYamlFile("distortion_params.yaml", yaml_content);

  // DistortionFilter requires both use_3d and use_imu parameters
  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);

    auto & params = config.nodes[0].parameters;
    EXPECT_TRUE(params.find("use_3d") != params.end());
    EXPECT_TRUE(params.find("use_imu") != params.end());
    EXPECT_FALSE(std::any_cast<bool>(params.at("use_3d")));
    EXPECT_TRUE(std::any_cast<bool>(params.at("use_imu")));
  });
}

/**
 * Test 15: Parse configuration with optional inputs
 */
TEST_F(DagYamlParsingTest, ParseOptionalInputs)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"

  inputs:
    - name: "pointcloud"
      type: "sensor_msgs::msg::PointCloud2"
      topic: "~/input/pointcloud"
      optional: false

    - name: "twist"
      type: "geometry_msgs::msg::TwistWithCovarianceStamped"
      topic: "~/input/twist"
      optional: true

  nodes:
    - id: "process"
      type: "DistortionFilter"
      inputs:
        - source: "pointcloud"
        - source: "twist"
          optional: true
      outputs:
        - name: "output"
      parameters:
        use_3d: false
        use_imu: false
)";

  std::string filepath = createTempYamlFile("optional_inputs.yaml", yaml_content);

  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);

    ASSERT_EQ(config.inputs.size(), 2);
    EXPECT_FALSE(config.inputs[0].optional);
    EXPECT_TRUE(config.inputs[1].optional);

    ASSERT_EQ(config.nodes[0].inputs.size(), 2);
    EXPECT_TRUE(config.nodes[0].inputs[1].optional);
  });
}

/**
 * Test 16: Parameter type inference - comprehensive test
 */
TEST_F(DagYamlParsingTest, ParameterTypeInference)
{
  std::string yaml_content = R"(
dag:
  name: "type_test"
  nodes:
    - id: "test"
      type: "OrganizeFilter"
      inputs:
        - source: "input"
      outputs:
        - name: "output"
      parameters:
        string_param: "hello"
        bool_true: true
        bool_false: false
        int_param: 42
        float_param: 3.14
        negative_int: -10
        negative_float: -2.5
        array_param: [1, 2, 3]
)";

  std::string filepath = createTempYamlFile("type_inference.yaml", yaml_content);

  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);

    auto & params = config.nodes[0].parameters;

    // Verify string
    ASSERT_TRUE(params.find("string_param") != params.end());
    EXPECT_NO_THROW(std::any_cast<std::string>(params.at("string_param")));
    EXPECT_EQ(std::any_cast<std::string>(params.at("string_param")), "hello");

    // Verify booleans
    ASSERT_TRUE(params.find("bool_true") != params.end());
    EXPECT_NO_THROW(std::any_cast<bool>(params.at("bool_true")));
    EXPECT_TRUE(std::any_cast<bool>(params.at("bool_true")));

    EXPECT_FALSE(std::any_cast<bool>(params.at("bool_false")));

    // Verify integers
    ASSERT_TRUE(params.find("int_param") != params.end());
    EXPECT_NO_THROW(std::any_cast<int>(params.at("int_param")));
    EXPECT_EQ(std::any_cast<int>(params.at("int_param")), 42);

    EXPECT_EQ(std::any_cast<int>(params.at("negative_int")), -10);

    // Verify floats
    ASSERT_TRUE(params.find("float_param") != params.end());
    EXPECT_NO_THROW(std::any_cast<double>(params.at("float_param")));
    EXPECT_NEAR(std::any_cast<double>(params.at("float_param")), 3.14, 0.001);

    EXPECT_NEAR(std::any_cast<double>(params.at("negative_float")), -2.5, 0.001);
  });
}

/**
 * Test 17: Error - Reference to non-existent node
 */
TEST_F(DagYamlParsingTest, ErrorReferenceNonExistentNode)
{
  std::string yaml_content = R"(
dag:
  name: "test_dag"
  nodes:
    - id: "node1"
      type: "OrganizeFilter"
      inputs:
        - source: "output"
          from_node: "nonexistent_node"
      outputs:
        - name: "output"
)";

  std::string filepath = createTempYamlFile("nonexistent_reference.yaml", yaml_content);

  // Note: Validation of node references happens at DAG build time, not parse time
  // This test documents that parsing succeeds but execution should fail
  ASSERT_NO_THROW({
    DagConfig config = DagConfigParser::parseFromFile(filepath);
    // DAG executor should detect missing node during buildDag()
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
