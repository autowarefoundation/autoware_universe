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

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/multi_step_inference.hpp"
#include "autoware/diffusion_planner/inference/onnxruntime_inference.hpp"
#include "autoware/diffusion_planner/inference/single_step_inference.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"

#include <CLI/CLI.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{

struct Options
{
  std::filesystem::path model_dir;
  std::filesystem::path plugins_path;
  std::filesystem::path input_npz;
  std::string precision{"fp32"};
  int batch_size{1};
};

struct CompareStats
{
  size_t size{0};
  size_t max_error_index{0};
  float max_abs_error{0.0f};
  float max_rel_error{0.0f};
  float avg_abs_error{0.0f};
  float rmse{0.0f};
};

struct ReportItem
{
  std::string name;
  size_t size{0};
  size_t max_error_index{0};
  float max_abs_error{0.0f};
  float max_rel_error{0.0f};
  float avg_abs_error{0.0f};
  float rmse{0.0f};
  float lhs_at_max_error{0.0f};
  float rhs_at_max_error{0.0f};
};

struct InferenceOutputs
{
  std::vector<float> prediction;
  std::vector<float> turn_indicator_logit;
};

struct InferenceReports
{
  InferenceOutputs native_single_step;
  InferenceOutputs native_multi_step;
  InferenceOutputs ort_cpu_single_step;
  InferenceOutputs ort_cpu_multi_step;
};

struct ModelPaths
{
  std::filesystem::path single_step_model_path;
  std::filesystem::path encoder_model_path;
  std::filesystem::path decoder_model_path;
  std::filesystem::path turn_indicator_model_path;
  std::filesystem::path args_path;
};

template <class Shape>
size_t shape_size(const Shape & shape)
{
  size_t size = 1;
  for (const auto dim : shape) {
    size *= static_cast<size_t>(dim);
  }
  return size;
}

std::optional<std::filesystem::path> existing_file(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    return std::nullopt;
  }
  return path;
}

std::filesystem::path default_plugins_path()
{
  const auto share_dir = ament_index_cpp::get_package_share_directory("autoware_tensorrt_plugins");
  return std::filesystem::path{share_dir} / "plugins" / "libautoware_tensorrt_plugins.so";
}

Options parse_options(const int argc, char ** argv)
{
  Options options;
  CLI::App app{"Report diffusion planner inference mode differences"};
  app
    .add_option(
      "--model-dir", options.model_dir, "Directory containing diffusion planner ONNX models")
    ->required()
    ->check(CLI::ExistingDirectory);
  app
    .add_option(
      "--plugins-path", options.plugins_path,
      "Path to libautoware_tensorrt_plugins.so. Defaults to the installed "
      "autoware_tensorrt_plugins path.")
    ->check(CLI::ExistingFile);
  app
    .add_option(
      "--input-npz", options.input_npz,
      "Optional NPZ input file. If omitted, deterministic random input is used.")
    ->check(CLI::ExistingFile);
  app.add_option("--precision", options.precision, "Native TensorRT precision")
    ->check(CLI::IsMember({"fp32", "fp16"}));
  app.add_option("--batch-size", options.batch_size, "Batch size")->check(CLI::Range(1, 1));

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError & e) {
    std::exit(app.exit(e));
  }

  if (options.plugins_path.empty()) {
    options.plugins_path = default_plugins_path();
  }
  return options;
}

ModelPaths resolve_model_paths(const std::filesystem::path & model_dir)
{
  const ModelPaths paths{
    model_dir / "diffusion_planner.onnx", model_dir / "diffusion_planner_encoder.onnx",
    model_dir / "diffusion_planner_decoder.onnx",
    model_dir / "diffusion_planner_turn_indicator.onnx", model_dir / "args.json"};

  const std::vector<std::filesystem::path> required_paths{
    paths.single_step_model_path, paths.encoder_model_path, paths.decoder_model_path,
    paths.turn_indicator_model_path, paths.args_path};
  for (const auto & path : required_paths) {
    if (!existing_file(path)) {
      throw std::runtime_error("Required file does not exist: " + path.string());
    }
  }
  return paths;
}

std::vector<float> random_vector(const size_t size, std::mt19937 & rng)
{
  std::uniform_real_distribution<float> distribution{-1.0f, 1.0f};
  std::vector<float> data(size);
  std::generate(data.begin(), data.end(), [&]() { return distribution(rng); });
  return data;
}

preprocess::InputDataMap create_random_input_data()
{
  std::mt19937 rng{0};

  preprocess::InputDataMap input_data_map;
  input_data_map["sampled_trajectories"] =
    random_vector(shape_size(SAMPLED_TRAJECTORIES_SHAPE), rng);
  input_data_map["ego_agent_past"] = random_vector(shape_size(EGO_HISTORY_SHAPE), rng);
  input_data_map["ego_current_state"] = random_vector(shape_size(EGO_CURRENT_STATE_SHAPE), rng);
  input_data_map["neighbor_agents_past"] = random_vector(shape_size(NEIGHBOR_SHAPE), rng);
  input_data_map["static_objects"] = random_vector(shape_size(STATIC_OBJECTS_SHAPE), rng);
  input_data_map["lanes"] = random_vector(shape_size(LANES_SHAPE), rng);
  input_data_map["lanes_speed_limit"] = random_vector(shape_size(LANES_SPEED_LIMIT_SHAPE), rng);
  input_data_map["route_lanes"] = random_vector(shape_size(ROUTE_LANES_SHAPE), rng);
  input_data_map["route_lanes_speed_limit"] =
    random_vector(shape_size(ROUTE_LANES_SPEED_LIMIT_SHAPE), rng);
  input_data_map["polygons"] = random_vector(shape_size(POLYGONS_SHAPE), rng);
  input_data_map["line_strings"] = random_vector(shape_size(LINE_STRINGS_SHAPE), rng);
  input_data_map["goal_pose"] = random_vector(shape_size(GOAL_POSE_SHAPE), rng);
  input_data_map["ego_shape"] = random_vector(shape_size(EGO_SHAPE_SHAPE), rng);
  input_data_map["turn_indicators"] = random_vector(shape_size(TURN_INDICATORS_SHAPE), rng);
  input_data_map["delay"] = std::vector<float>(shape_size(DELAY_SHAPE), 0.0f);
  return input_data_map;
}

template <class Shape>
std::vector<size_t> shape_without_batch(const Shape & shape)
{
  std::vector<size_t> expected;
  expected.reserve(shape.size() - 1);
  for (size_t i = 1; i < shape.size(); ++i) {
    expected.push_back(static_cast<size_t>(shape[i]));
  }
  return expected;
}

void validate_shape(
  const std::string & name, const cnpy::NpyArray & array, const std::vector<size_t> & expected)
{
  if (array.shape != expected) {
    std::string message = "Unexpected shape for " + name + ": actual=[";
    for (size_t i = 0; i < array.shape.size(); ++i) {
      message += (i == 0 ? "" : ",") + std::to_string(array.shape[i]);
    }
    message += "] expected=[";
    for (size_t i = 0; i < expected.size(); ++i) {
      message += (i == 0 ? "" : ",") + std::to_string(expected[i]);
    }
    message += "]";
    throw std::runtime_error(message);
  }
}

std::vector<float> load_float_array(
  cnpy::npz_t & npz, const std::string & name, const std::vector<size_t> & expected_shape)
{
  const auto it = npz.find(name);
  if (it == npz.end()) {
    throw std::runtime_error("NPZ does not contain key: " + name);
  }
  validate_shape(name, it->second, expected_shape);
  if (it->second.word_size != sizeof(float)) {
    throw std::runtime_error("Expected float32 array for key: " + name);
  }
  const auto * data = it->second.data<float>();
  return std::vector<float>(data, data + it->second.num_vals);
}

std::vector<float> load_int32_array_as_float(
  cnpy::npz_t & npz, const std::string & name, const std::vector<size_t> & expected_shape)
{
  const auto it = npz.find(name);
  if (it == npz.end()) {
    throw std::runtime_error("NPZ does not contain key: " + name);
  }
  validate_shape(name, it->second, expected_shape);
  if (it->second.word_size != sizeof(int32_t)) {
    throw std::runtime_error("Expected int32 array for key: " + name);
  }
  const auto * data = it->second.data<int32_t>();
  std::vector<float> output(it->second.num_vals);
  std::transform(data, data + it->second.num_vals, output.begin(), [](const int32_t value) {
    return static_cast<float>(value);
  });
  return output;
}

std::vector<float> pose3_to_pose4(const std::vector<float> & pose3)
{
  if (pose3.size() % 3 != 0) {
    throw std::runtime_error("pose3_to_pose4 input size must be a multiple of 3");
  }
  std::vector<float> pose4(pose3.size() / 3 * 4);
  for (size_t i = 0; i < pose3.size() / 3; ++i) {
    pose4[i * 4 + 0] = pose3[i * 3 + 0];
    pose4[i * 4 + 1] = pose3[i * 3 + 1];
    pose4[i * 4 + 2] = std::cos(pose3[i * 3 + 2]);
    pose4[i * 4 + 3] = std::sin(pose3[i * 3 + 2]);
  }
  return pose4;
}

std::vector<float> with_batch(const std::vector<float> & data)
{
  return data;
}

preprocess::InputDataMap load_npz_input_data(const std::filesystem::path & npz_path)
{
  auto npz = cnpy::npz_load(npz_path.string());

  preprocess::InputDataMap input_data_map;
  input_data_map["ego_agent_past"] =
    pose3_to_pose4(load_float_array(npz, "ego_agent_past", {INPUT_T + 1, 3}));
  input_data_map["ego_current_state"] =
    load_float_array(npz, "ego_current_state", shape_without_batch(EGO_CURRENT_STATE_SHAPE));
  input_data_map["neighbor_agents_past"] =
    with_batch(load_float_array(npz, "neighbor_agents_past", shape_without_batch(NEIGHBOR_SHAPE)));
  input_data_map["static_objects"] =
    with_batch(load_float_array(npz, "static_objects", shape_without_batch(STATIC_OBJECTS_SHAPE)));
  input_data_map["lanes"] =
    with_batch(load_float_array(npz, "lanes", shape_without_batch(LANES_SHAPE)));
  input_data_map["lanes_speed_limit"] = with_batch(
    load_float_array(npz, "lanes_speed_limit", shape_without_batch(LANES_SPEED_LIMIT_SHAPE)));
  input_data_map["route_lanes"] =
    with_batch(load_float_array(npz, "route_lanes", shape_without_batch(ROUTE_LANES_SHAPE)));
  input_data_map["route_lanes_speed_limit"] = with_batch(load_float_array(
    npz, "route_lanes_speed_limit", shape_without_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE)));
  input_data_map["polygons"] =
    with_batch(load_float_array(npz, "polygons", shape_without_batch(POLYGONS_SHAPE)));
  input_data_map["line_strings"] =
    with_batch(load_float_array(npz, "line_strings", shape_without_batch(LINE_STRINGS_SHAPE)));
  input_data_map["goal_pose"] = pose3_to_pose4(load_float_array(npz, "goal_pose", {3}));
  input_data_map["ego_shape"] =
    load_float_array(npz, "ego_shape", shape_without_batch(EGO_SHAPE_SHAPE));
  input_data_map["turn_indicators"] =
    load_int32_array_as_float(npz, "turn_indicators", shape_without_batch(TURN_INDICATORS_SHAPE));
  input_data_map["delay"] = std::vector<float>(shape_size(DELAY_SHAPE), 0.0f);
  input_data_map["sampled_trajectories"] = preprocess::create_sampled_trajectories(0.0);
  return input_data_map;
}

CompareStats compare_vectors(const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  if (lhs.size() != rhs.size()) {
    throw std::runtime_error(
      "Vector size mismatch: lhs=" + std::to_string(lhs.size()) +
      " rhs=" + std::to_string(rhs.size()));
  }

  CompareStats stats;
  stats.size = lhs.size();
  float sum_abs_error = 0.0f;
  float sum_squared_error = 0.0f;
  for (size_t i = 0; i < lhs.size(); ++i) {
    const float abs_error = std::abs(lhs[i] - rhs[i]);
    const float rel_error = abs_error / std::max({1.0f, std::abs(lhs[i]), std::abs(rhs[i])});
    sum_abs_error += abs_error;
    sum_squared_error += abs_error * abs_error;
    if (abs_error > stats.max_abs_error) {
      stats.max_abs_error = abs_error;
      stats.max_rel_error = rel_error;
      stats.max_error_index = i;
    }
  }

  stats.avg_abs_error = sum_abs_error / static_cast<float>(lhs.size());
  stats.rmse = std::sqrt(sum_squared_error / static_cast<float>(lhs.size()));
  return stats;
}

std::string format_float(const float value)
{
  std::ostringstream stream;
  stream << std::setprecision(3) << value;
  return stream.str();
}

ReportItem make_report_item(
  const std::string & name, const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  const auto stats = compare_vectors(lhs, rhs);
  return ReportItem{
    name,
    stats.size,
    stats.max_error_index,
    stats.max_abs_error,
    stats.max_rel_error,
    stats.avg_abs_error,
    stats.rmse,
    lhs[stats.max_error_index],
    rhs[stats.max_error_index]};
}

template <class ValueGetter>
void print_report_line(
  const std::string & metric_name, const std::vector<ReportItem> & items, ValueGetter value_getter)
{
  std::cout << "  " << metric_name << ": ";
  for (size_t i = 0; i < items.size(); ++i) {
    std::cout << (i == 0 ? "" : ", ") << '(' << items[i].name << ": " << value_getter(items[i])
              << ')';
  }
  std::cout << '\n';
}

void print_report(const std::string & label, const std::vector<ReportItem> & items)
{
  std::cout << "[REPORT] " << label << '\n';
  print_report_line(
    "size", items, [](const ReportItem & item) { return std::to_string(item.size); });
  print_report_line("max_abs_error", items, [](const ReportItem & item) {
    return format_float(item.max_abs_error);
  });
  print_report_line("max_rel_error", items, [](const ReportItem & item) {
    return format_float(item.max_rel_error);
  });
  print_report_line("avg_abs_error", items, [](const ReportItem & item) {
    return format_float(item.avg_abs_error);
  });
  print_report_line("rmse", items, [](const ReportItem & item) { return format_float(item.rmse); });
  print_report_line("max_error_index", items, [](const ReportItem & item) {
    return std::to_string(item.max_error_index);
  });
  print_report_line(
    "lhs", items, [](const ReportItem & item) { return format_float(item.lhs_at_max_error); });
  print_report_line(
    "rhs", items, [](const ReportItem & item) { return format_float(item.rhs_at_max_error); });
}

void print_comparison(
  const std::string & label, const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  print_report(label, {make_report_item("all", lhs, rhs)});
}

std::vector<float> extract_ego_trajectory(const std::vector<float> & prediction)
{
  constexpr size_t trajectory_size = static_cast<size_t>(OUTPUT_T) * POSE_DIM;
  if (prediction.size() % trajectory_size != 0) {
    throw std::runtime_error("Prediction size is not compatible with ego trajectory extraction");
  }

  std::vector<float> ego_trajectory(trajectory_size);
  std::copy_n(prediction.begin(), trajectory_size, ego_trajectory.begin());
  return ego_trajectory;
}

std::vector<float> extract_ego_trajectory_dim(
  const std::vector<float> & prediction, const size_t dim)
{
  if (dim >= static_cast<size_t>(POSE_DIM)) {
    throw std::runtime_error("Invalid ego trajectory dimension");
  }
  const auto ego_trajectory = extract_ego_trajectory(prediction);
  std::vector<float> output(static_cast<size_t>(OUTPUT_T));
  for (size_t t = 0; t < static_cast<size_t>(OUTPUT_T); ++t) {
    output[t] = ego_trajectory[t * POSE_DIM + dim];
  }
  return output;
}

void print_ego_trajectory_comparison(
  const std::string & label, const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  const std::vector<ReportItem> items{
    make_report_item("all", lhs, rhs),
    make_report_item(
      "ego_x", extract_ego_trajectory_dim(lhs, 0), extract_ego_trajectory_dim(rhs, 0)),
    make_report_item(
      "ego_y", extract_ego_trajectory_dim(lhs, 1), extract_ego_trajectory_dim(rhs, 1)),
    make_report_item(
      "cos_yaw", extract_ego_trajectory_dim(lhs, 2), extract_ego_trajectory_dim(rhs, 2)),
    make_report_item(
      "sin_yaw", extract_ego_trajectory_dim(lhs, 3), extract_ego_trajectory_dim(rhs, 3))};

  print_report(label, items);
}

InferenceOutput require_result(InferenceResult result, const std::string & label)
{
  if (!result.has_value()) {
    throw std::runtime_error(label + " failed: " + result.error());
  }
  return std::move(result.value());
}

InferenceReports run_inference(
  const ModelPaths & paths, const Options & options,
  const preprocess::InputDataMap & input_data_map,
  const utils::StateNormalization & state_normalization)
{
  SingleStepInference single_step_inference{
    paths.single_step_model_path.string(), options.plugins_path.string(), options.batch_size,
    options.precision, false};
  MultiStepInference multi_step_inference{
    paths.encoder_model_path.string(),
    paths.decoder_model_path.string(),
    paths.turn_indicator_model_path.string(),
    options.plugins_path.string(),
    options.batch_size,
    options.precision,
    false};
  OnnxruntimeSingleStepInference cpu_single_step_inference{
    paths.single_step_model_path.string(), "cpu", "", options.batch_size};
  OnnxruntimeMultiStepInference cpu_multi_step_inference{
    paths.encoder_model_path.string(),
    paths.decoder_model_path.string(),
    paths.turn_indicator_model_path.string(),
    "cpu",
    "",
    options.batch_size};

  const auto native_single_step =
    require_result(single_step_inference.infer(input_data_map), "native TensorRT single-step");
  const auto native_multi_step =
    require_result(multi_step_inference.infer(input_data_map), "native TensorRT multi-step");
  const auto ort_cpu_single_step =
    require_result(cpu_single_step_inference.infer(input_data_map), "ONNX Runtime CPU single-step");
  const auto ort_cpu_multi_step =
    require_result(cpu_multi_step_inference.infer(input_data_map), "ONNX Runtime CPU multi-step");

  if (!native_single_step.is_denormalized || !ort_cpu_single_step.is_denormalized) {
    throw std::runtime_error("Single-step outputs must be denormalized");
  }
  if (native_multi_step.is_denormalized || ort_cpu_multi_step.is_denormalized) {
    throw std::runtime_error("Multi-step outputs must be normalized");
  }

  return InferenceReports{
    {native_single_step.outputs.first, native_single_step.outputs.second},
    {postprocess::denormalize_prediction(native_multi_step.outputs.first, state_normalization),
     native_multi_step.outputs.second},
    {ort_cpu_single_step.outputs.first, ort_cpu_single_step.outputs.second},
    {postprocess::denormalize_prediction(ort_cpu_multi_step.outputs.first, state_normalization),
     ort_cpu_multi_step.outputs.second}};
}

void print_prediction_comparison(
  const std::string & label, const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  print_ego_trajectory_comparison(label, lhs, rhs);
}

void print_reports(const InferenceReports & reports)
{
  print_prediction_comparison(
    "native TensorRT single-step prediction vs native TensorRT multi-step prediction",
    reports.native_single_step.prediction, reports.native_multi_step.prediction);
  print_prediction_comparison(
    "native TensorRT single-step prediction vs ONNX Runtime CPU single-step prediction",
    reports.native_single_step.prediction, reports.ort_cpu_single_step.prediction);
  print_prediction_comparison(
    "ONNX Runtime CPU single-step prediction vs ONNX Runtime CPU multi-step prediction",
    reports.ort_cpu_single_step.prediction, reports.ort_cpu_multi_step.prediction);
  print_prediction_comparison(
    "native TensorRT multi-step prediction vs ONNX Runtime CPU multi-step prediction",
    reports.native_multi_step.prediction, reports.ort_cpu_multi_step.prediction);
  print_prediction_comparison(
    "native TensorRT multi-step prediction vs ONNX Runtime CPU single-step prediction",
    reports.native_multi_step.prediction, reports.ort_cpu_single_step.prediction);

  print_comparison(
    "native TensorRT single-step turn-indicator logit vs native TensorRT multi-step "
    "turn-indicator logit",
    reports.native_single_step.turn_indicator_logit,
    reports.native_multi_step.turn_indicator_logit);
  print_comparison(
    "native TensorRT single-step turn-indicator logit vs ONNX Runtime CPU single-step "
    "turn-indicator logit",
    reports.native_single_step.turn_indicator_logit,
    reports.ort_cpu_single_step.turn_indicator_logit);
  print_comparison(
    "native TensorRT multi-step turn-indicator logit vs ONNX Runtime CPU multi-step "
    "turn-indicator logit",
    reports.native_multi_step.turn_indicator_logit,
    reports.ort_cpu_multi_step.turn_indicator_logit);
}

int run(const Options & options)
{
  if (!existing_file(options.plugins_path)) {
    throw std::runtime_error(
      "TensorRT plugin file does not exist: " + options.plugins_path.string());
  }

  const auto paths = resolve_model_paths(options.model_dir);
  const auto input_data_map =
    options.input_npz.empty() ? create_random_input_data() : load_npz_input_data(options.input_npz);
  const auto state_normalization = utils::load_state_normalization(paths.args_path.string());

  std::cout << "model_dir: " << options.model_dir << '\n';
  std::cout << "plugins_path: " << options.plugins_path << '\n';
  if (!options.input_npz.empty()) {
    std::cout << "input_npz: " << options.input_npz << '\n';
  }
  std::cout << "batch_size: " << options.batch_size << '\n';
  std::cout << "precision: " << options.precision << '\n';

  const auto reports = run_inference(paths, options, input_data_map, state_normalization);
  print_reports(reports);

  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace autoware::diffusion_planner

int main(int argc, char ** argv)
{
  try {
    const auto options = autoware::diffusion_planner::parse_options(argc, argv);
    return autoware::diffusion_planner::run(options);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
