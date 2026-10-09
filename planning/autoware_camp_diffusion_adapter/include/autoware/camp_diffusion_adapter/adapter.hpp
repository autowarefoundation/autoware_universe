// Copyright 2026 Xinchen Lin. Licensed under the Apache License, Version 2.0.
#ifndef AUTOWARE__CAMP_DIFFUSION_ADAPTER__ADAPTER_HPP_
#define AUTOWARE__CAMP_DIFFUSION_ADAPTER__ADAPTER_HPP_

#include <autoware/camp_selector/adapter_contracts.hpp>
#include <autoware/camp_selector/camp_atom_materializer.hpp>
#include <autoware/diffusion_planner/diffusion_planner_core.hpp>
#include <autoware_camp_selector/msg/camp_candidate_pool.hpp>
#include <autoware_camp_selector/msg/camp_selection.hpp>

#include <std_msgs/msg/header.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::camp_diffusion_adapter
{
namespace dp = autoware::diffusion_planner;
namespace camp = autoware::camp_selector;

struct PreparedTick
{
  autoware_camp_selector::msg::CampCandidatePool pool;
};
struct SelectedTick
{
  dp::Trajectory trajectory;
  dp::TurnIndicatorsCommand turn_indicators_command;
  dp::PredictedObjects predicted_objects;
  autoware_camp_selector::msg::CampSelection selection;
};

// Uses only installed upstream public interfaces; does not compile/copy DP sources.
// All methods run in the single-threaded ROS executor owned by the host.
class CampDiffusionAdapter
{
public:
  CampDiffusionAdapter(
    dp::DiffusionPlannerParams params, dp::VehicleInfo vehicle,
    const std::string & fixed_weight_model_path);
  dp::DiffusionPlannerCore & core() { return *core_; }
  bool ready() const { return core_->is_model_loaded() && core_->is_map_loaded(); }
  void set_map(const lanelet::LaneletMapConstPtr & map);
  void reset_episode();
  void set_start_guidance_enabled(bool enabled);
  void set_stop_guidance_enabled(bool enabled);
  void set_centerline_guidance_enabled(bool enabled);
  PreparedTick prepare(
    const dp::FrameContext & frame, const rclcpp::Time & timestamp,
    const dp::UUID & generator_uuid);
  std::optional<SelectedTick> select(const autoware_camp_selector::msg::CampSelection & selection);
  bool confirm_published_selection(const std_msgs::msg::Header & header, std::uint32_t row);

private:
  struct Pending
  {
    PreparedTick tick;
    dp::FrameContext frame;
    dp::AgentPoses poses;
    camp::CampRankingResult expected;
  };
  dp::DiffusionPlannerParams params_;
  dp::VehicleInfo vehicle_;
  const camp::CampFixedWeightModel model_;
  dp::StateNormalization state_normalization_;
  std::unique_ptr<dp::DiffusionPlannerCore> core_;
  lanelet::LaneletMapConstPtr map_;
  std::vector<camp::CampLaneBoundary> boundaries_;
  camp::PublishedPlanLedger<camp::CampWorldPlan> ledger_;
  std::optional<Pending> pending_;
  std::optional<std::uint32_t> selected_ready_;
  bool start_guidance_enabled_{false}, stop_guidance_enabled_{false},
    centerline_guidance_enabled_{false};
};
}  // namespace autoware::camp_diffusion_adapter
#endif
