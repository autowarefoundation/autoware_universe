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

#include "autoware/tensorrt_yolox/parameter_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_yolox
{

namespace
{

// Best-effort conversion of a YAML scalar to a typed rclcpp::ParameterValue.
// Order matters: bool before int (`true`/`false` would otherwise fail as int),
// int before double (so `0` stays an int rather than becoming 0.0), and
// string as the final fallback. Keeping the type ladder here rather than in
// each caller means the yaml stays free of type hints.
//
// Quoted scalars (single or double) are treated as strings even if their
// content looks numeric — this matches YAML 1.2 (`"42"` is a string) and
// matches how `rcl_yaml_param_parser` behaves, so migrating between the two
// loaders doesn't silently change types.
rclcpp::ParameterValue scalar_to_value(const YAML::Node & node)
{
  const auto & tag = node.Tag();
  // yaml-cpp uses "!" for scalars whose type came from being quoted (i.e. no
  // implicit resolution needed) and "?" for plain scalars that still need
  // resolution. explicit `!!str` tags come through as their full URI form
  const bool is_forced_string = tag == "!" || tag == "tag:yaml.org,2002:str" || tag == "!!str";
  if (is_forced_string) {
    return rclcpp::ParameterValue(node.Scalar());
  }

  bool b{};
  if (YAML::convert<bool>::decode(node, b)) {
    // yaml-cpp treats numeric literals as decodable booleans (`1` -> true)
    // guard against that by re-checking the raw scalar text
    const auto & s = node.Scalar();
    if (s == "true" || s == "false" || s == "True" || s == "False") {
      return rclcpp::ParameterValue(b);
    }
  }

  int64_t i{};
  if (YAML::convert<int64_t>::decode(node, i)) {
    return rclcpp::ParameterValue(i);
  }

  double d{};
  if (YAML::convert<double>::decode(node, d)) {
    return rclcpp::ParameterValue(d);
  }

  return rclcpp::ParameterValue(node.Scalar());
}

// Convert a homogeneous YAML sequence into a typed array. Falls back to
// string array if elements are mixed / non-scalar. Matches how ROS 2 handles
// yaml sequences so `ros2 param get` output stays sensible.
//
// Type selection walks the whole sequence first (rather than trusting the
// first element) so that a mix like `[1, 2.5, 3]` widens to a double array
// even when the leading element looks like an int.
rclcpp::ParameterValue sequence_to_value(const YAML::Node & node)
{
  if (node.size() == 0) {
    return rclcpp::ParameterValue(std::vector<std::string>{});
  }

  std::vector<rclcpp::ParameterValue> values;
  values.reserve(node.size());
  for (const auto & elem : node) {
    values.push_back(scalar_to_value(elem));
  }

  bool all_bool = true;
  bool all_int = true;
  bool all_numeric = true;
  for (const auto & v : values) {
    const auto t = v.get_type();
    if (t != rclcpp::ParameterType::PARAMETER_BOOL) all_bool = false;
    if (t != rclcpp::ParameterType::PARAMETER_INTEGER) all_int = false;
    if (
      t != rclcpp::ParameterType::PARAMETER_INTEGER &&
      t != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      all_numeric = false;
    }
  }

  if (all_bool) {
    std::vector<bool> out;
    out.reserve(values.size());
    for (const auto & v : values) out.push_back(v.get<bool>());

    return rclcpp::ParameterValue(out);
  }

  if (all_int) {
    std::vector<int64_t> out;
    out.reserve(values.size());
    for (const auto & v : values) out.push_back(v.get<int64_t>());

    return rclcpp::ParameterValue(out);
  }

  if (all_numeric) {
    std::vector<double> out;
    out.reserve(values.size());
    for (const auto & v : values) {
      out.push_back(
        v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
          ? static_cast<double>(v.get<int64_t>())
          : v.get<double>());
    }

    return rclcpp::ParameterValue(out);
  }

  // fallback: stringify every element
  // uses the raw yaml scalar so quoted numerics keep their original spelling
  std::vector<std::string> out;
  out.reserve(node.size());
  for (const auto & elem : node) {
    out.push_back(elem.Scalar());
  }

  return rclcpp::ParameterValue(out);
}

// Walks `node` and appends every leaf to `out` under `prefix.child.leaf`. This
// lets a yaml author write nested maps (grouping related settings) while the
// node still sees flat dotted names like `image_pub.qos.depth`.
//
// Also auto-detects ROS 2's `--params-file` format at the top level: keys
// that look like node paths (`/name` or `/**`) with a `ros__parameters:`
// child have both layers stripped. The parameter names inside then match
// what the flat form would produce, so the same yaml file can be dropped in
// with either style without touching the node code.
void flatten(
  const YAML::Node & node, const std::string & prefix,
  std::vector<std::pair<std::string, rclcpp::ParameterValue>> & out)
{
  if (node.IsMap()) {
    for (const auto & kv : node) {
      const auto key = kv.first.as<std::string>();
      // ROS 2 params-file wrapper: only recognized at the top level so that
      // stray `/`-prefixed keys deeper in a yaml (unusual, but not our call)
      // don't get silently rewritten.
      if (
        prefix.empty() && !key.empty() && key.front() == '/' && kv.second.IsMap() &&
        kv.second["ros__parameters"] && kv.second["ros__parameters"].IsMap()) {
        flatten(kv.second["ros__parameters"], "", out);
        continue;
      }
      const auto child_prefix = prefix.empty() ? key : prefix + "." + key;
      flatten(kv.second, child_prefix, out);
    }
  } else if (node.IsSequence()) {
    out.emplace_back(prefix, sequence_to_value(node));
  } else if (node.IsScalar()) {
    out.emplace_back(prefix, scalar_to_value(node));
  }
  // null nodes are ignored: they typically come from a bare `key:` line and
  // shouldn't be declared as a parameter with no type.
}

std::vector<std::pair<std::string, rclcpp::ParameterValue>> parse_config_file(
  const std::filesystem::path & file)
{
  const YAML::Node root = YAML::LoadFile(file.string());
  if (!root.IsMap()) {
    throw std::runtime_error(
      "config file '" + file.string() + "' must be a YAML mapping at the top level");
  }

  std::vector<std::pair<std::string, rclcpp::ParameterValue>> out;
  flatten(root, "", out);

  return out;
}

std::vector<std::string> read_manifest(const std::filesystem::path & manifest_path)
{
  const YAML::Node root = YAML::LoadFile(manifest_path.string());
  const YAML::Node files_node = root["config_files"];
  if (!files_node || !files_node.IsSequence()) {
    throw std::runtime_error(
      "params manifest '" + manifest_path.string() + "' must contain a `config_files:` sequence");
  }

  std::vector<std::string> files;
  files.reserve(files_node.size());
  for (const auto & entry : files_node) {
    files.push_back(entry.as<std::string>());
  }

  return files;
}

}  // namespace

// Loads every config file named in `<dir>/parameters.manifest.yaml` into `node`.
// Files are plain YAML mappings — no `/**: ros__parameters:` wrapper. Nested
// maps are flattened into dotted keys (`a.b.c`), and scalar types are inferred
// (bool -> int -> double -> string, in that order).
//
// Files are loaded in the order listed in the manifest; if the same key
// appears in multiple files, later files win.
void load_parameters_from_manifest(rclcpp::Node & node, const std::filesystem::path & dir)
{
  namespace fs = std::filesystem;
  const auto manifest_path = dir / kParameterManifestFilename;
  if (!fs::is_regular_file(manifest_path)) {
    throw std::runtime_error("params manifest not found: " + manifest_path.string());
  }

  const auto files = read_manifest(manifest_path);

  for (const auto & relative_name : files) {
    const fs::path file = dir / relative_name;
    if (!fs::is_regular_file(file)) {
      throw std::runtime_error(
        "config file '" + relative_name + "' listed in " + manifest_path.string() +
        " does not exist at " + file.string());
    }
    RCLCPP_INFO(node.get_logger(), "Loading config from %s", file.c_str());

    for (auto & [name, value] : parse_config_file(file)) {
      if (node.has_parameter(name)) {
        node.set_parameter(rclcpp::Parameter(name, value));
      } else {
        node.declare_parameter(name, value);
      }
    }
  }
}

// Joins `relative` onto `root` and returns the normalized absolute path.
// Throws if `relative` is empty or absolute.
std::string join_relative(const std::filesystem::path & root, const std::string & relative)
{
  if (relative.empty()) {
    throw std::runtime_error("expected a relative path, got an empty string");
  }
  const std::filesystem::path rel(relative);
  if (rel.is_absolute()) {
    throw std::runtime_error("expected a relative path, got absolute: " + relative);
  }

  // Note: we do not reject `..`
  return (root / rel).lexically_normal().string();
}

// Fetches a parameter that must exist.
rclcpp::Parameter ParameterReader::required(const std::string & name)
{
  if (use_manifest_) {
    if (!node_.has_parameter(name)) {
      throw rclcpp::exceptions::ParameterNotDeclaredException(
        "required parameter '" + name +
        "' is not declared; "
        "check that a config file in the parameters manifest sets it");
    }
    return node_.get_parameter(name);
  }

  // declare with no default and dynamic typing, so the launcher
  // must supply the value. Without `dynamic_typing`, rclcpp rejects
  // `PARAMETER_NOT_SET` outright
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  return rclcpp::Parameter(name, node_.declare_parameter(name, rclcpp::ParameterValue{}, desc));
}

// Fetches a parameter that may be missing, falling back to `default_value`.
rclcpp::Parameter ParameterReader::optional(
  const std::string & name, const rclcpp::ParameterValue & default_value)
{
  if (use_manifest_) {
    return node_.has_parameter(name) ? node_.get_parameter(name)
                                     : rclcpp::Parameter(name, default_value);
  }
  return rclcpp::Parameter(name, node_.declare_parameter(name, default_value));
}

}  // namespace autoware::tensorrt_yolox
