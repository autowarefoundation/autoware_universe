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

#include <fstream>
#include <stdexcept>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

DagConfig DagConfigParser::parseFromFile(const std::string & yaml_file_path)
{
  try {
    YAML::Node yaml = YAML::LoadFile(yaml_file_path);
    return parseFromString(YAML::Dump(yaml));
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Failed to parse YAML file '" + yaml_file_path + "': " + e.what());
  }
}

DagConfig DagConfigParser::parseFromString(const std::string & yaml_content)
{
  try {
    YAML::Node root = YAML::Load(yaml_content);

    if (!root["dag"]) {
      throw std::runtime_error("YAML must contain 'dag' root element");
    }

    YAML::Node dag_node = root["dag"];
    DagConfig config;

    // Parse name and version
    config.name = dag_node["name"] ? dag_node["name"].as<std::string>() : "unnamed_dag";
    config.version = dag_node["version"] ? dag_node["version"].as<std::string>() : "1.0";

    // Parse inputs
    if (dag_node["inputs"]) {
      for (const auto & input_yaml : dag_node["inputs"]) {
        DagInputConfig input;
        input.name = input_yaml["name"].as<std::string>();
        input.type = input_yaml["type"].as<std::string>();
        input.topic = input_yaml["topic"].as<std::string>();
        input.optional = input_yaml["optional"] ? input_yaml["optional"].as<bool>() : false;
        config.inputs.push_back(input);
      }
    }

    // Parse nodes
    if (!dag_node["nodes"]) {
      throw std::runtime_error("DAG must contain at least one node");
    }

    for (const auto & node_yaml : dag_node["nodes"]) {
      config.nodes.push_back(parseNode(node_yaml));
    }

    // Parse outputs
    if (dag_node["outputs"]) {
      for (const auto & output_yaml : dag_node["outputs"]) {
        DagOutputConfig output;
        output.name = output_yaml["name"].as<std::string>();
        output.source = output_yaml["source"].as<std::string>();
        output.from_node = output_yaml["from_node"].as<std::string>();
        output.topic = output_yaml["topic"].as<std::string>();
        output.type = output_yaml["type"].as<std::string>();
        config.outputs.push_back(output);
      }
    }

    return config;

  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Failed to parse YAML: " + std::string(e.what()));
  }
}

DagNodeConfig DagConfigParser::parseNode(const YAML::Node & node_yaml)
{
  DagNodeConfig config;

  // Parse required fields with detailed error messages
  if (!node_yaml["id"]) {
    throw std::runtime_error(
      "DAG node is missing required 'id' field. Each node must have a unique identifier.");
  }
  config.id = node_yaml["id"].as<std::string>();

  if (!node_yaml["type"]) {
    throw std::runtime_error(
      "DAG node '" + config.id + "' is missing required 'type' field. " +
      "Valid types include: OrganizeFilter, TransformFilter, CropBoxFilter, " +
      "DistortionFilter, RingOutlierFilter, FinalizeFilter");
  }
  config.type = node_yaml["type"].as<std::string>();

  // Parse inputs with validation
  if (node_yaml["inputs"]) {
    int input_idx = 0;
    for (const auto & input_yaml : node_yaml["inputs"]) {
      if (!input_yaml["source"]) {
        throw std::runtime_error(
          "Input " + std::to_string(input_idx) + " of node '" + config.id +
          "' is missing required 'source' field.");
      }

      DagNodeInput input;
      input.source = input_yaml["source"].as<std::string>();
      input.from_node = input_yaml["from_node"] ? input_yaml["from_node"].as<std::string>() : "";
      input.name = input_yaml["name"] ? input_yaml["name"].as<std::string>()
                                      : "";  // Optional: defaults to source
      input.optional = input_yaml["optional"] ? input_yaml["optional"].as<bool>() : false;
      config.inputs.push_back(input);
      input_idx++;
    }
  }

  // Parse outputs with validation
  if (node_yaml["outputs"]) {
    int output_idx = 0;
    for (const auto & output_yaml : node_yaml["outputs"]) {
      std::string output_name;
      if (output_yaml["name"]) {
        output_name = output_yaml["name"].as<std::string>();
      } else if (output_yaml.IsScalar()) {
        output_name = output_yaml.as<std::string>();
      } else {
        throw std::runtime_error(
          "Output " + std::to_string(output_idx) + " of node '" + config.id +
          "' must be a string or have a 'name' field.");
      }

      if (output_name.empty()) {
        throw std::runtime_error(
          "Output " + std::to_string(output_idx) + " of node '" + config.id + "' has empty name.");
      }

      config.outputs.push_back(output_name);
      output_idx++;
    }
  }

  // Validate filter-specific required parameters
  if (node_yaml["parameters"]) {
    config.parameters = parseParameters(node_yaml["parameters"]);
  }

  // Filter-specific validation
  validateFilterParameters(config);

  return config;
}

void DagConfigParser::validateFilterParameters(const DagNodeConfig & config)
{
  // Validate filter-specific required parameters
  if (config.type == "TransformFilter") {
    if (config.parameters.find("target_frame") == config.parameters.end()) {
      throw std::runtime_error(
        "TransformFilter node '" + config.id + "' is missing required parameter 'target_frame'");
    }
  } else if (config.type == "DistortionFilter") {
    // Optional: use_3d, use_imu - no required params
  } else if (config.type == "RingOutlierFilter") {
    if (config.parameters.find("distance_ratio") == config.parameters.end()) {
      throw std::runtime_error(
        "RingOutlierFilter node '" + config.id +
        "' is missing required parameter 'distance_ratio'");
    }
    if (config.parameters.find("object_length_threshold") == config.parameters.end()) {
      throw std::runtime_error(
        "RingOutlierFilter node '" + config.id +
        "' is missing required parameter 'object_length_threshold'");
    }
  } else if (config.type == "CropBoxFilter") {
    // crop_boxes parameter is optional (can be empty)
  } else if (config.type == "DownsampleFilter") {
    if (config.parameters.find("voxel_size_x") == config.parameters.end()) {
      throw std::runtime_error(
        "DownsampleFilter node '" + config.id + "' is missing required parameter 'voxel_size_x'");
    }
    if (config.parameters.find("voxel_size_y") == config.parameters.end()) {
      throw std::runtime_error(
        "DownsampleFilter node '" + config.id + "' is missing required parameter 'voxel_size_y'");
    }
    if (config.parameters.find("voxel_size_z") == config.parameters.end()) {
      throw std::runtime_error(
        "DownsampleFilter node '" + config.id + "' is missing required parameter 'voxel_size_z'");
    }
  } else if (config.type == "OrganizeFilter" || config.type == "FinalizeFilter") {
    // No required parameters
  } else {
    // Unknown filter type - warn but don't fail
    // This allows for future extensibility
  }
}

std::map<std::string, std::any> DagConfigParser::parseParameters(const YAML::Node & params_yaml)
{
  std::map<std::string, std::any> params;

  if (!params_yaml.IsMap()) {
    return params;
  }

  for (const auto & param : params_yaml) {
    std::string key = param.first.as<std::string>();
    const YAML::Node & value = param.second;

    // Handle different YAML types
    if (value.IsScalar()) {
      // Try to parse as different types
      try {
        // Try bool
        if (value.as<std::string>() == "true" || value.as<std::string>() == "false") {
          params[key] = value.as<bool>();
        }
        // Try int
        else if (value.Tag() == "?" && value.as<std::string>().find('.') == std::string::npos) {
          params[key] = value.as<int>();
        }
        // Try double
        else {
          params[key] = value.as<double>();
        }
      } catch (...) {
        // Fallback to string
        params[key] = value.as<std::string>();
      }
    } else if (value.IsSequence()) {
      // Handle array of maps (e.g., crop_boxes)
      if (value.size() > 0 && value[0].IsMap()) {
        std::vector<std::map<std::string, double>> vec_of_maps;
        for (const auto & item : value) {
          std::map<std::string, double> map_item;
          for (const auto & kv : item) {
            map_item[kv.first.as<std::string>()] = kv.second.as<double>();
          }
          vec_of_maps.push_back(map_item);
        }
        params[key] = vec_of_maps;
      }
      // Handle simple arrays
      else {
        std::vector<double> vec;
        for (const auto & item : value) {
          vec.push_back(item.as<double>());
        }
        params[key] = vec;
      }
    } else if (value.IsMap()) {
      // Nested map - convert to map<string, double>
      std::map<std::string, double> nested_map;
      for (const auto & nested : value) {
        nested_map[nested.first.as<std::string>()] = nested.second.as<double>();
      }
      params[key] = nested_map;
    }
  }

  return params;
}

DagConfig DagConfigParser::parseFromParameters(rclcpp::Node * node)
{
  // Get the DAG configuration file path from ROS parameters
  std::string yaml_file = node->declare_parameter<std::string>("dag_config_file", "");

  if (yaml_file.empty()) {
    throw std::runtime_error(
      "Required parameter 'dag_config_file' is not set. "
      "This parameter must contain the path to a DAG configuration YAML file. "
      "Example: dag_config_file: \"$(find-pkg-share "
      "autoware_cuda_pointcloud_preprocessor)/config/dag/standard_preprocessing.yaml\"");
  }

  // Parse the YAML file
  try {
    return parseFromFile(yaml_file);
  } catch (const std::exception & e) {
    throw std::runtime_error(
      "Failed to parse DAG configuration from file '" + yaml_file + "': " + e.what());
  }
}

}  // namespace autoware::cuda_pointcloud_preprocessor::dag
