// Tensor layout of the preserved v5.0 DP materializer. Other planners may
// call rank_camp_candidates directly with equivalent 16 atoms instead.
#ifndef AUTOWARE__CAMP_SELECTOR__TENSOR_DIMENSIONS_HPP_
#define AUTOWARE__CAMP_SELECTOR__TENSOR_DIMENSIONS_HPP_
#include <cstdint>
namespace autoware::camp_selector
{
inline constexpr std::int64_t NUM_SEGMENTS_IN_LANE = 140;
inline constexpr std::int64_t NUM_SEGMENTS_IN_ROUTE = 25;
inline constexpr std::int64_t MAX_NUM_AGENTS = 321;
inline constexpr std::int64_t POINTS_PER_SEGMENT = 20;
inline constexpr std::int64_t SEGMENT_POINT_DIM = 33;
inline constexpr std::int64_t OUTPUT_T = 80;
inline constexpr std::int64_t POSE_DIM = 4;
inline constexpr std::int64_t X = 0;
inline constexpr std::int64_t Y = 1;
inline constexpr std::int64_t dX = 2;
inline constexpr std::int64_t dY = 3;
inline constexpr std::int64_t LB_X = 4;
inline constexpr std::int64_t LB_Y = 5;
inline constexpr std::int64_t RB_X = 6;
inline constexpr std::int64_t RB_Y = 7;
inline constexpr std::int64_t TRAFFIC_LIGHT_RED = 10;
inline constexpr std::int64_t TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT = 12;
}  // namespace autoware::camp_selector
#endif
