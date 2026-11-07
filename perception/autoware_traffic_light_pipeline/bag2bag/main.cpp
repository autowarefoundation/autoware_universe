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

#include "config_builder.hpp"
#include "pipeline_runner.hpp"

#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light::CommandLineArgs;

// Parses `--camera 4,5` (or `--camera 4 --camera 5`) into {4, 5}: the numeric part of each
// camera's namespace, which is all the topic names need (see build_camera_config()).
std::vector<int> parse_camera_indices(const std::string & value)
{
  std::vector<int> camera_indices;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (token.empty() || token.find_first_not_of("0123456789") != std::string::npos) {
      throw std::runtime_error(
        "--camera takes comma-separated camera numbers (e.g. 4,5), but got '" + token + "'");
    }
    camera_indices.push_back(std::stoi(token));
  }
  return camera_indices;
}

CommandLineArgs parse_args(int argc, char ** argv)
{
  // `--flag value` and `--flag=value` are both accepted, so neither habit trips over the other.
  const auto split_flag = [](const std::string & arg) {
    const auto separator = arg.find('=');
    return separator == std::string::npos ? std::pair<std::string, std::string>{arg, {}}
                                          : std::pair<std::string, std::string>{
                                              arg.substr(0, separator), arg.substr(separator + 1)};
  };

  CommandLineArgs args;
  for (int i = 1; i < argc; ++i) {
    auto [flag, value] = split_flag(argv[i]);
    if (value.empty()) {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + flag);
      }
      value = argv[++i];
    }

    if (flag == "--input-bag") {
      args.input_bag_path = value;
    } else if (flag == "--map") {
      args.map_path = value;
    } else if (flag == "--output-bag") {
      args.output_bag_path = value;
    } else if (flag == "--camera") {
      const auto camera_indices = parse_camera_indices(value);
      args.camera_indices.insert(
        args.camera_indices.end(), camera_indices.begin(), camera_indices.end());
    } else if (flag == "--ml-model-path") {
      args.ml_model_path = value;
    } else if (flag == "--config") {
      args.config_path = value;
    } else {
      throw std::runtime_error("unknown argument " + flag);
    }
  }
  if (
    args.input_bag_path.empty() || args.map_path.empty() || args.output_bag_path.empty() ||
    args.camera_indices.empty()) {
    throw std::runtime_error(
      "usage: traffic_light_pipeline_bag2bag_runner --input-bag <path> --map <dir> "
      "--output-bag <path> --camera <numbers, e.g. 4,5> [--ml-model-path <dir>] "
      "[--config <param.yaml>]");
  }
  return args;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto config = autoware::traffic_light::build_bag2bag_config(parse_args(argc, argv));
    autoware::traffic_light::run_bag2bag(config);
  } catch (const std::exception & e) {
    std::cerr << "traffic_light_pipeline_bag2bag_runner failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
