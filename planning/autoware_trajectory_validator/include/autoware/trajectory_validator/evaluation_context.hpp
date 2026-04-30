#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__EVALUATION_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__EVALUATION_CONTEXT_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::trajectory_validator
{

/**
 * @brief A read-only snapshot of the world state for a single pipeline execution.
 * Callees receive it by `const &` and must not mutate fields.
 */
struct EvaluationContext
{
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects;
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_light_signals;
};

// Alias to maintain backward compatibility with existing validator plugins
using FilterContext = EvaluationContext;

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__EVALUATION_CONTEXT_HPP_
