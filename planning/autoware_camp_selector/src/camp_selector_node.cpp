// Copyright 2026 Xinchen Lin
// Licensed under the Apache License, Version 2.0.

#include "autoware/camp_selector/camp_ranker.hpp"

#include <autoware_camp_selector/msg/camp_candidate_pool.hpp>
#include <autoware_camp_selector/msg/camp_selection.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::camp_selector
{
class CampSelectorNode : public rclcpp::Node
{
public:
  explicit CampSelectorNode(const rclcpp::NodeOptions & options)
  : Node("camp_selector", options), model_([this]() {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      return load_camp_fixed_weight_model(
        declare_parameter<std::string>("fixed_weight_model_path", "", descriptor));
    }())
  {
    selection_pub_ = create_publisher<Selection>("~/output/selection", 1);
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
    turn_pub_ = create_publisher<TurnIndicators>("~/output/turn_indicators", 1);
    subscription_ = create_subscription<Pool>(
      "~/input/candidate_pool", 1, [this](const Pool::ConstSharedPtr pool) { select(*pool); });
  }

private:
  using Pool = autoware_camp_selector::msg::CampCandidatePool;
  using Selection = autoware_camp_selector::msg::CampSelection;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using TurnIndicators = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

  void select(const Pool & pool)
  {
    try {
      const auto & candidates = pool.candidates.candidate_trajectories;
      if (
        candidates.size() != model_.candidate_pool_k ||
        pool.raw_atoms.size() != candidates.size() * kCampAtomCount) {
        throw std::invalid_argument("candidate/atom count differs from frozen CAMP K");
      }
      CampStatusPattern status;
      for (std::size_t atom = 0; atom < kCampAtomCount; ++atom) {
        switch (pool.atom_status.at(atom)) {
          case Pool::OBSERVED:
            status.at(atom) = CampAtomStatus::Observed;
            break;
          case Pool::NOT_APPLICABLE:
            status.at(atom) = CampAtomStatus::NotApplicable;
            break;
          case Pool::TYPED_MISSING:
            status.at(atom) = CampAtomStatus::TypedMissing;
            break;
          default:
            throw std::invalid_argument("unknown endpoint state");
        }
      }
      std::vector<CampAtomVector> atoms(candidates.size());
      for (std::size_t row = 0; row < atoms.size(); ++row) {
        const auto & candidate = candidates.at(row);
        if (candidate.points.empty() || candidate.header != pool.header) {
          throw std::invalid_argument("candidate must have points and the pool's same-tick header");
        }
        std::copy_n(
          pool.raw_atoms.begin() + row * kCampAtomCount, kCampAtomCount, atoms.at(row).begin());
      }
      const auto decision = rank_camp_candidates(model_, status, atoms);
      Selection selection;
      selection.header = pool.header;
      selection.pool_id = pool.pool_id;
      selection.selected_index = static_cast<std::uint32_t>(decision.selected_index);
      selection.costs = decision.costs;
      selection.candidate = candidates.at(decision.selected_index);
      Trajectory trajectory;
      trajectory.header = selection.candidate.header;
      trajectory.points = selection.candidate.points;
      trajectory_pub_->publish(trajectory);
      turn_pub_->publish(selection.candidate.turn_indicators_command);
      selection_pub_->publish(selection);
      RCLCPP_DEBUG(get_logger(), "CAMP selected original candidate %zu", decision.selected_index);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAMP rejected pool: %s", error.what());
    }
  }

  const CampFixedWeightModel model_;
  rclcpp::Publisher<Selection>::SharedPtr selection_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<TurnIndicators>::SharedPtr turn_pub_;
  rclcpp::Subscription<Pool>::SharedPtr subscription_;
};
}  // namespace autoware::camp_selector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::camp_selector::CampSelectorNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
