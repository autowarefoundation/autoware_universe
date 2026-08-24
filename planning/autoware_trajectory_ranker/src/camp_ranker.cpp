// Copyright 2026 Xinchen Lin
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

#include "autoware/trajectory_ranker/camp_ranker.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{
namespace
{

using nlohmann::json;

CampAtomVector parse_atom_vector(const json & values, const std::string & field)
{
  if (!values.is_array() || values.size() != kCampAtomCount) {
    throw std::invalid_argument(field + " must contain exactly 16 values");
  }

  CampAtomVector output{};
  for (std::size_t i = 0; i < kCampAtomCount; ++i) {
    output.at(i) = values.at(i).get<double>();
  }
  return output;
}

std::array<double, 3> parse_transition_scales(const json & values)
{
  const auto & source = values.at("transition_component_positive_q95");
  return {
    source.at("position_m").get<double>(), source.at("yaw_rad").get<double>(),
    source.at("longitudinal_velocity_mps").get<double>()};
}

CampStatusPattern parse_status_pattern(const json & values)
{
  if (!values.is_array() || values.size() != kCampAtomCount) {
    throw std::invalid_argument("pattern status must contain exactly 16 values");
  }

  CampStatusPattern output{};
  for (std::size_t i = 0; i < kCampAtomCount; ++i) {
    output.at(i) = camp_atom_status_from_string(values.at(i).get<std::string>());
  }
  return output;
}

void validate_model(const CampFixedWeightModel & model)
{
  if (model.candidate_pool_k == 0) {
    throw std::invalid_argument("candidate_pool_k must be positive");
  }
  if (model.atom_names.size() != kCampAtomCount) {
    throw std::invalid_argument("atom_names must contain exactly 16 values");
  }
  for (std::size_t i = 0; i < kCampAtomCount; ++i) {
    if (std::string_view(model.atom_names.at(i)) != kCampAtomNames.at(i)) {
      throw std::invalid_argument("CAMP atom names or order do not match the deployment contract");
    }
  }
  if (model.patterns.empty()) {
    throw std::invalid_argument("at least one CAMP status pattern is required");
  }

  for (const double scale : model.scales) {
    if (!std::isfinite(scale) || scale <= 0.0) {
      throw std::invalid_argument("CAMP atom scales must be finite and positive");
    }
  }
  for (const double scale : model.transition_component_scales) {
    if (!std::isfinite(scale) || scale <= 0.0) {
      throw std::invalid_argument("CAMP transition component scales must be finite and positive");
    }
  }

  for (const auto & pattern : model.patterns) {
    double weight_sum = 0.0;
    for (std::size_t i = 0; i < kCampAtomCount; ++i) {
      const double weight = pattern.weights.at(i);
      if (!std::isfinite(weight) || weight < 0.0) {
        throw std::invalid_argument("CAMP weights must be finite and nonnegative");
      }
      if (pattern.status.at(i) != CampAtomStatus::Observed && std::abs(weight) > 1.0e-12) {
        throw std::invalid_argument("non-observed CAMP atoms must have zero weight");
      }
      weight_sum += weight;
    }
    if (std::abs(weight_sum - 1.0) > 1.0e-6) {
      throw std::invalid_argument("CAMP weights must sum to one for every status pattern");
    }
  }
}

}  // namespace

CampAtomStatus camp_atom_status_from_string(const std::string_view status)
{
  if (status == "observed") return CampAtomStatus::Observed;
  if (status == "not_applicable") return CampAtomStatus::NotApplicable;
  if (status == "typed_missing") return CampAtomStatus::TypedMissing;
  throw std::invalid_argument("unknown CAMP atom status: " + std::string(status));
}

CampFixedWeightModel load_camp_fixed_weight_model(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open CAMP model: " + path.string());
  }

  const json root = json::parse(stream);
  if (root.at("format_version").get<int>() != 1) {
    throw std::invalid_argument("unsupported CAMP model format_version");
  }

  CampFixedWeightModel model;
  model.candidate_pool_k = root.at("candidate_pool_k").get<std::size_t>();
  model.atom_names = root.at("atom_names").get<std::vector<std::string>>();
  model.scales = parse_atom_vector(root.at("scales"), "scales");
  model.transition_component_scales = parse_transition_scales(root.at("transition_scales"));

  for (const auto & pattern_json : root.at("patterns")) {
    model.patterns.push_back(CampPatternWeights{
      parse_status_pattern(pattern_json.at("status")),
      parse_atom_vector(pattern_json.at("weights"), "pattern weights")});
  }

  validate_model(model);
  return model;
}

CampRankingResult rank_camp_candidates(
  const CampFixedWeightModel & model, const CampStatusPattern & status,
  const std::vector<CampAtomVector> & raw_atoms)
{
  validate_model(model);
  if (raw_atoms.size() != model.candidate_pool_k) {
    throw std::invalid_argument("candidate count does not match the CAMP model pool size");
  }

  const auto pattern = std::find_if(
    model.patterns.begin(), model.patterns.end(),
    [&status](const CampPatternWeights & candidate) { return candidate.status == status; });
  if (pattern == model.patterns.end()) {
    throw std::invalid_argument("CAMP model does not contain the requested status pattern");
  }

  CampRankingResult result;
  result.selected_index = 0;
  result.costs.reserve(raw_atoms.size());

  for (const auto & candidate : raw_atoms) {
    double cost = 0.0;
    for (std::size_t i = 0; i < kCampAtomCount; ++i) {
      if (status.at(i) != CampAtomStatus::Observed) continue;
      const double raw = candidate.at(i);
      if (!std::isfinite(raw)) {
        throw std::invalid_argument("observed CAMP atoms must be finite");
      }
      const double normalized = std::clamp(raw / model.scales.at(i), 0.0, 10.0);
      cost += normalized * pattern->weights.at(i);
    }
    result.costs.push_back(cost);
  }

  for (std::size_t i = 1; i < result.costs.size(); ++i) {
    if (result.costs.at(i) < result.costs.at(result.selected_index)) {
      result.selected_index = i;
    }
  }
  return result;
}

}  // namespace autoware::trajectory_ranker
