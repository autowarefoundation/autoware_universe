// Copyright 2026 The Autoware Contributors
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
#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>

#include <boost/geometry.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace autoware::obstacle_grid_utils
{
namespace
{
constexpr double grid_resolution = 0.2;  // metres per cell
constexpr double grid_length = 10.0;     // metres per side, centred on the origin
constexpr int expected_grid_columns = 50;

/// What the extractor is assumed to have written into one cell. Only point_count and max_height
/// are ever read by the helpers under test; min_height is populated for realism, never queried.
struct CellContent
{
  float point_count;
  float max_height;
};

constexpr CellContent occupied{40.0f, 1.5f};
constexpr float realistic_min_height = 0.5f;

/// An all-NaN (heartbeat) grid: no cell qualifies until a test marks one.
grid_map::GridMap make_empty_grid()
{
  grid_map::GridMap grid({"max_height", "min_height", "point_count"});
  grid.setFrameId("base_link");
  grid.setGeometry(
    grid_map::Length(grid_length, grid_length), grid_resolution, grid_map::Position(0.0, 0.0));
  grid["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid["min_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  return grid;
}

void mark_cell(grid_map::GridMap & grid, const grid_map::Index & index, const CellContent & content)
{
  grid.at("point_count", index) = content.point_count;
  grid.at("max_height", index) = content.max_height;
  grid.at("min_height", index) = realistic_min_height;
}

grid_map::Index index_at(const grid_map::GridMap & grid, const grid_map::Position & position)
{
  grid_map::Index index;
  grid.getIndex(position, index);
  return index;
}

/// A grid whose only marked cell is the one containing `center`.
///
/// NOTE: with the grid centred at (0,0) and a 0.2 m resolution, cell centres fall on odd
/// multiples of 0.1 (..., 1.9, 2.1, ...), so the tests pass true cell-centre coordinates
/// (e.g. 2.1, 1.1) to keep the hand-computed oracles exact.
grid_map::GridMap make_grid_with_one_cell(
  const grid_map::Position & center, const CellContent & content)
{
  auto grid = make_empty_grid();
  mark_cell(grid, index_at(grid, center), content);
  return grid;
}

/// Degenerate single-point ego polygon at the origin, so the measured distance is purely the
/// distance to the obstacle cell.
Polygon2d ego_polygon_at_origin()
{
  Polygon2d polygon;
  boost::geometry::append(polygon.outer(), Point2d(0.0, 0.0));
  boost::geometry::correct(polygon);
  return polygon;
}

bool cell_qualifies_at(
  const grid_map::GridMap & grid, const grid_map::Position & position, const Gate & gate)
{
  return cell_qualifies(grid, index_at(grid, position), gate);
}
}  // namespace

// --- nearest_distance ---------------------------------------------------------------------------

TEST(ObstacleGridUtils, NearestDistanceMeasuresToTheCellEdgeNotItsCenter)
{
  // Arrange: one occupied cell centred at (2.1, 1.1); its 0.2 m footprint box is
  // [2.0, 2.2] x [1.0, 1.2], so its near corner (2.0, 1.0) is closer to ego than its centre.
  const auto grid = make_grid_with_one_cell(grid_map::Position(2.1, 1.1), occupied);
  const Gate accept_anything{1u, 0.0};

  // Act
  const double distance_to_nearest =
    nearest_distance(grid, ego_polygon_at_origin(), accept_anything);

  // Assert: the hand-computed distance to the near footprint corner, strictly closer than the
  // distance to the cell centre would be.
  EXPECT_NEAR(distance_to_nearest, std::hypot(2.0, 1.0), 1e-6);
  EXPECT_LT(distance_to_nearest, std::hypot(2.1, 1.1));
}

TEST(ObstacleGridUtils, NearestDistanceIsInfiniteWhenNothingQualifies)
{
  // Arrange: the only marked cell holds 5 points, below the gate's threshold of 10.
  const auto grid = make_grid_with_one_cell(grid_map::Position(2.1, 1.1), CellContent{5.0f, 1.5f});
  const Gate needs_ten_points{10u, 0.0};

  // Act
  const double distance_to_nearest =
    nearest_distance(grid, ego_polygon_at_origin(), needs_ten_points);

  // Assert
  EXPECT_TRUE(std::isinf(distance_to_nearest));
}

// --- cell_qualifies -----------------------------------------------------------------------------

TEST(ObstacleGridUtils, CellQualifiesWhenPointCountReachesTheThreshold)
{
  // Arrange
  const grid_map::Position center(2.1, 1.1);
  const auto grid = make_grid_with_one_cell(center, CellContent{10.0f, 1.5f});

  // Act & Assert
  EXPECT_TRUE(cell_qualifies_at(grid, center, Gate{10u, 0.0}));
}

TEST(ObstacleGridUtils, CellDoesNotQualifyWhenPointCountIsBelowTheThreshold)
{
  // Arrange
  const grid_map::Position center(2.1, 1.1);
  const auto grid = make_grid_with_one_cell(center, CellContent{9.0f, 1.5f});

  // Act & Assert
  EXPECT_FALSE(cell_qualifies_at(grid, center, Gate{10u, 0.0}));
}

TEST(ObstacleGridUtils, CellQualifiesWhenMaxHeightReachesTheThreshold)
{
  // Arrange: a cell tall enough to clear a 0.3 m height gate.
  const grid_map::Position center(2.1, 1.1);
  const auto grid = make_grid_with_one_cell(center, CellContent{50.0f, 1.2f});

  // Act & Assert
  EXPECT_TRUE(cell_qualifies_at(grid, center, Gate{10u, 0.3}));
}

TEST(ObstacleGridUtils, CellDoesNotQualifyWhenMaxHeightIsBelowTheThreshold)
{
  // Arrange: a flat ground-height sliver that must not trip a 0.3 m height gate.
  const grid_map::Position center(2.1, 1.1);
  const auto grid = make_grid_with_one_cell(center, CellContent{50.0f, 0.1f});

  // Act & Assert
  EXPECT_FALSE(cell_qualifies_at(grid, center, Gate{10u, 0.3}));
}

TEST(ObstacleGridUtils, CellDoesNotQualifyWhenMaxHeightIsNanEvenIfPointCountPasses)
{
  // Arrange: a heartbeat cell that got a point count but was never populated with a height. The
  // count clears the gate, so only the NaN guard on max_height can reject it.
  const grid_map::Position center(2.1, 1.1);
  const auto nan_height = std::numeric_limits<float>::quiet_NaN();
  const auto grid = make_grid_with_one_cell(center, CellContent{50.0f, nan_height});

  // Act & Assert
  EXPECT_FALSE(cell_qualifies_at(grid, center, Gate{10u, 0.0}));
}

// --- cell_corners -------------------------------------------------------------------------------

TEST(ObstacleGridUtils, CellCornersAreTheLiteralFootprintCorners)
{
  // Arrange: centre (2.1, 1.1) at 0.2 m resolution -> box [2.0, 2.2] x [1.0, 1.2], wound
  // min-min, min-max, max-max, max-min.
  const grid_map::Position center(2.1, 1.1);
  const std::array<Point2d, 4> expected_corners{
    Point2d(2.0, 1.0), Point2d(2.0, 1.2), Point2d(2.2, 1.2), Point2d(2.2, 1.0)};

  // Act
  const std::array<Point2d, 4> corners = cell_corners(center, grid_resolution);

  // Assert
  for (std::size_t i = 0; i < expected_corners.size(); ++i) {
    SCOPED_TRACE("corner " + std::to_string(i));
    EXPECT_NEAR(corners[i].x(), expected_corners[i].x(), 1e-9);
    EXPECT_NEAR(corners[i].y(), expected_corners[i].y(), 1e-9);
  }
}

// --- nearest_cell -------------------------------------------------------------------------------

TEST(ObstacleGridUtils, NearestCellReportsTheSameDistanceAsNearestDistance)
{
  // Arrange
  const auto grid = make_grid_with_one_cell(grid_map::Position(2.1, 1.1), occupied);
  const auto ego = ego_polygon_at_origin();
  const Gate accept_anything{1u, 0.0};
  const double expected_distance = nearest_distance(grid, ego, accept_anything);

  // Act
  const auto nearest = nearest_cell(grid, ego, accept_anything);

  // Assert
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(nearest->distance, expected_distance, 1e-12);
}

TEST(ObstacleGridUtils, NearestCellReportsWhereTheCellIs)
{
  // Arrange
  const grid_map::Position center(2.1, 1.1);
  const auto grid = make_grid_with_one_cell(center, occupied);
  const grid_map::Index expected_index = index_at(grid, center);

  // Act
  const auto nearest = nearest_cell(grid, ego_polygon_at_origin(), Gate{1u, 0.0});

  // Assert: the cell centre and the grid index, for debug markers / SafetyFactor.points.
  ASSERT_TRUE(nearest.has_value());
  EXPECT_NEAR(nearest->position.x(), center.x(), 1e-6);
  EXPECT_NEAR(nearest->position.y(), center.y(), 1e-6);
  EXPECT_TRUE((nearest->index == expected_index).all());
}

TEST(ObstacleGridUtils, NearestCellIsNulloptWhenNothingQualifies)
{
  // Arrange: 5 points is below the gate's threshold of 10, so nothing qualifies.
  const auto grid = make_grid_with_one_cell(grid_map::Position(2.1, 1.1), CellContent{5.0f, 1.5f});

  // Act
  const auto nearest = nearest_cell(grid, ego_polygon_at_origin(), Gate{10u, 0.0});

  // Assert: the std::nullopt <-> +inf analog of nearest_distance.
  EXPECT_FALSE(nearest.has_value());
}

TEST(ObstacleGridUtils, NearestCellPicksTheCloserOfTwoOccupiedCells)
{
  // Arrange
  auto grid = make_empty_grid();
  const grid_map::Position near_center(2.1, 1.1);
  const grid_map::Index near_index = index_at(grid, near_center);
  mark_cell(grid, near_index, occupied);
  mark_cell(grid, index_at(grid, grid_map::Position(4.1, 3.1)), occupied);

  // Act
  const auto nearest = nearest_cell(grid, ego_polygon_at_origin(), Gate{1u, 0.0});

  // Assert
  ASSERT_TRUE(nearest.has_value());
  EXPECT_TRUE((nearest->index == near_index).all());
}

// --- connected_components -----------------------------------------------------------------------
// connected_components does not gate, so these tests supply the qualifying index list directly
// and assert the resulting component structure.

TEST(ObstacleGridUtils, ConnectedComponentsOfNothingIsNoComponents)
{
  // Arrange
  const auto grid = make_empty_grid();

  // Act
  const auto components = connected_components(grid, {});

  // Assert
  EXPECT_TRUE(components.empty());
}

TEST(ObstacleGridUtils, ConnectedComponentsOfOneCellIsOneComponent)
{
  // Arrange
  auto grid = make_empty_grid();
  const grid_map::Index only_cell(10, 10);
  mark_cell(grid, only_cell, occupied);

  // Act
  const auto components = connected_components(grid, {only_cell});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 1u);
}

TEST(ObstacleGridUtils, ConnectedComponentsSumsPointCountAcrossTheComponent)
{
  // Arrange: three adjacent cells holding 10, 20 and 30 points.
  auto grid = make_empty_grid();
  const grid_map::Index left(10, 10);
  const grid_map::Index middle(10, 11);
  const grid_map::Index right(10, 12);
  mark_cell(grid, left, CellContent{10.0f, 1.5f});
  mark_cell(grid, middle, CellContent{20.0f, 1.5f});
  mark_cell(grid, right, CellContent{30.0f, 1.5f});

  // Act
  const auto components = connected_components(grid, {left, middle, right});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_NEAR(components[0].point_sum, 60.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsJoinAStraightLineOfCells)
{
  // Arrange
  auto grid = make_empty_grid();
  const grid_map::Index left(10, 10);
  const grid_map::Index middle(10, 11);
  const grid_map::Index right(10, 12);
  mark_cell(grid, left, occupied);
  mark_cell(grid, middle, occupied);
  mark_cell(grid, right, occupied);

  // Act
  const auto components = connected_components(grid, {left, middle, right});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 3u);
}

TEST(ObstacleGridUtils, ConnectedComponentsJoinAnLShapeOfCells)
{
  // Arrange
  auto grid = make_empty_grid();
  const grid_map::Index corner(10, 10);
  const grid_map::Index below(11, 10);
  const grid_map::Index below_right(11, 11);
  mark_cell(grid, corner, occupied);
  mark_cell(grid, below, occupied);
  mark_cell(grid, below_right, occupied);

  // Act
  const auto components = connected_components(grid, {corner, below, below_right});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 3u);
}

TEST(ObstacleGridUtils, ConnectedComponentsJoinADiagonalPairUnder8Connectivity)
{
  // Arrange: the two cells touch only at a corner, which 4-connectivity would separate.
  auto grid = make_empty_grid();
  const grid_map::Index cell(10, 10);
  const grid_map::Index diagonal_neighbor(11, 11);
  mark_cell(grid, cell, occupied);
  mark_cell(grid, diagonal_neighbor, occupied);

  // Act
  const auto components = connected_components(grid, {cell, diagonal_neighbor});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 2u);
}

TEST(ObstacleGridUtils, ConnectedComponentsSplitCellsSeparatedByAGap)
{
  // Arrange: (10, 11) is deliberately left out, so the two cells are not adjacent.
  auto grid = make_empty_grid();
  const grid_map::Index left(10, 10);
  const grid_map::Index right(10, 12);
  mark_cell(grid, left, occupied);
  mark_cell(grid, right, occupied);

  // Act
  const auto components = connected_components(grid, {left, right});

  // Assert
  EXPECT_EQ(components.size(), 2u);
}

TEST(ObstacleGridUtils, ConnectedComponentsHandleBorderCellsWithoutProbingOffGrid)
{
  // Arrange: cells on the (0, 0) corner, whose neighbours like (-1, -1) are off-grid and must
  // neither be probed nor alias an in-grid cell through the linear key.
  auto grid = make_empty_grid();
  const grid_map::Index corner(0, 0);
  const grid_map::Index beside_corner(0, 1);
  mark_cell(grid, corner, occupied);
  mark_cell(grid, beside_corner, occupied);

  // Act
  const auto components = connected_components(grid, {corner, beside_corner});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 2u);
}

TEST(ObstacleGridUtils, ConnectedComponentsDoNotJoinCellsAcrossARowWrap)
{
  // Arrange: 10 m / 0.2 m = 50 columns. The last cell of one row and the first cell of the next
  // are NOT 8-adjacent, but a naive row * columns + column key would alias (row, 50) onto
  // (row + 1, 0). The off-grid bounds guard has to keep them apart.
  auto grid = make_empty_grid();
  ASSERT_EQ(grid.getSize()(1), expected_grid_columns);
  const grid_map::Index row_end(10, expected_grid_columns - 1);
  const grid_map::Index next_row_start(11, 0);
  mark_cell(grid, row_end, occupied);
  mark_cell(grid, next_row_start, occupied);

  // Act
  const auto components = connected_components(grid, {row_end, next_row_start});

  // Assert
  EXPECT_EQ(components.size(), 2u);
}

TEST(ObstacleGridUtils, ConnectedComponentsCountADuplicateIndexOnce)
{
  // Arrange: the same index supplied twice must be absorbed by the visited set rather than
  // forming a second component or double-counting its points.
  auto grid = make_empty_grid();
  const grid_map::Index only_cell(10, 10);
  mark_cell(grid, only_cell, CellContent{25.0f, 1.5f});

  // Act
  const auto components = connected_components(grid, {only_cell, only_cell});

  // Assert
  ASSERT_EQ(components.size(), 1u);
  EXPECT_NEAR(components[0].point_sum, 25.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsSkipANanPointCountInTheSum)
{
  // Arrange: two adjacent cells, one of which has a height but no point count. Without the NaN
  // guard the whole component's sum would go NaN and the real cluster's point total be lost.
  auto grid = make_empty_grid();
  const grid_map::Index counted(10, 10);
  const grid_map::Index uncounted(10, 11);
  mark_cell(grid, counted, CellContent{20.0f, 1.5f});
  mark_cell(grid, uncounted, CellContent{std::numeric_limits<float>::quiet_NaN(), 1.5f});

  // Act
  const auto components = connected_components(grid, {counted, uncounted});

  // Assert: both cells belong to the component, but only the counted one contributes.
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 2u);
  EXPECT_NEAR(components[0].point_sum, 20.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsMergeAcrossTheBufferWrapSeam)
{
  // Arrange: a move()d grid has a non-zero circular-buffer start index, so raw buffer-index
  // adjacency no longer equals geometric adjacency at the seam. Pick two geometrically adjacent
  // cells whose buffer indices land on opposite ends of the buffer.
  auto grid = make_empty_grid();
  grid.move(grid_map::Position(1.0, 0.0));  // shift the buffer origin by 5 cells along x
  ASSERT_FALSE(grid.isDefaultStartIndex());
  const grid_map::Size size = grid.getSize();
  const grid_map::Index start = grid.getStartIndex();
  ASSERT_NE(start(0), 0);
  // Unwrapped row u maps to buffer row (u + start(0)) % size(0), so the seam sits at this row.
  const int seam_row = size(0) - 1 - start(0);
  const int column = 5;
  const auto below_seam =
    grid_map::getBufferIndexFromIndex(grid_map::Index(seam_row, column), size, start);
  const auto above_seam =
    grid_map::getBufferIndexFromIndex(grid_map::Index(seam_row + 1, column), size, start);
  ASSERT_EQ(below_seam(0) - above_seam(0), size(0) - 1);  // they really do straddle the seam
  mark_cell(grid, below_seam, occupied);
  mark_cell(grid, above_seam, occupied);

  // Act
  const auto components = connected_components(grid, {below_seam, above_seam});

  // Assert: geometric adjacency wins over buffer-index distance.
  ASSERT_EQ(components.size(), 1u);
  EXPECT_EQ(components[0].cells.size(), 2u);
}

}  // namespace autoware::obstacle_grid_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
