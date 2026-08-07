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

#ifndef AUTOWARE__TENSORRT_YOLOX__PARAMETER_LOADER_HPP_
#define AUTOWARE__TENSORRT_YOLOX__PARAMETER_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <string>

namespace autoware::tensorrt_yolox
{

// Filename of the manifest expected inside the config directory. The manifest
// is a YAML file with a `config_files:` sequence listing which config files to
// load. Paths inside the manifest are resolved relative to the config directory.
inline constexpr const char * kParameterManifestFilename = "parameters.manifest.yaml";

void load_parameters_from_manifest(rclcpp::Node & node, const std::filesystem::path & dir);

// Returns the value of parameter `name` if it has been declared on `node`,
// otherwise `default_value`.
template <typename T>
T get_parameter_or(const rclcpp::Node & node, const std::string & name, const T & default_value)
{
  return node.has_parameter(name) ? node.get_parameter(name).get_value<T>() : default_value;
}

// Facade over parameter reads for nodes that support both the classic layout
// (each param declared here with a launcher-side `<param>` override) and the
// manifest layout (params pre-loaded from yaml files). Callers set
// `use_manifest` once and then call `required` / `optional` uniformly — the
// call site no longer branches on layout, which keeps the ctor readable
// during the migration window.
//
// When every caller has switched to manifest mode this class can be dropped
// in favor of plain `get_parameter`.
class ParameterReader
{
public:
  ParameterReader(rclcpp::Node & node, bool use_manifest) : node_(node), use_manifest_(use_manifest)
  {
  }

  rclcpp::Parameter required(const std::string & name);

  rclcpp::Parameter optional(
    const std::string & name, const rclcpp::ParameterValue & default_value);

private:
  rclcpp::Node & node_;
  bool use_manifest_;
};

std::string join_relative(const std::filesystem::path & root, const std::string & relative);

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__PARAMETER_LOADER_HPP_
