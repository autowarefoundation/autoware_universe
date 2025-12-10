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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_CONFIG_PARSER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_CONFIG_PARSER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/dag/dag_executor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <any>
#include <map>
#include <string>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Input definition from YAML
 */
struct DagInputConfig
{
  std::string name;
  std::string type;
  std::string topic;
  bool optional{false};
};

/**
 * @brief Output definition from YAML
 */
struct DagOutputConfig
{
  std::string name;
  std::string source;     // Output name from node
  std::string from_node;  // Node ID
  std::string topic;
  std::string type;
};

/**
 * @brief Complete DAG configuration from YAML
 */
struct DagConfig
{
  std::string name;
  std::string version;
  std::vector<DagInputConfig> inputs;
  std::vector<DagNodeConfig> nodes;
  std::vector<DagOutputConfig> outputs;
};

/**
 * @brief Parser for DAG configuration from YAML file
 */
class DagConfigParser
{
public:
  /**
   * @brief Parse DAG configuration from YAML file path
   * @param yaml_file_path Path to YAML configuration file
   * @return Parsed DAG configuration
   */
  static DagConfig parseFromFile(const std::string & yaml_file_path);

  /**
   * @brief Parse DAG configuration from YAML string
   * @param yaml_content YAML content as string
   * @return Parsed DAG configuration
   */
  static DagConfig parseFromString(const std::string & yaml_content);

  /**
   * @brief Parse DAG configuration from ROS2 parameters
   * @param node ROS2 node to read parameters from
   * @return Parsed DAG configuration
   */
  static DagConfig parseFromParameters(rclcpp::Node * node);

private:
  /**
   * @brief Parse a single node configuration from YAML node
   */
  static DagNodeConfig parseNode(const YAML::Node & node_yaml);

  /**
   * @brief Parse parameters map from YAML node
   */
  static std::map<std::string, std::any> parseParameters(const YAML::Node & params_yaml);

  /**
   * @brief Validate filter-specific required parameters
   * @throws std::runtime_error if required parameters are missing
   */
  static void validateFilterParameters(const DagNodeConfig & config);
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__DAG_CONFIG_PARSER_HPP_
