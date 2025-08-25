
#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_

#include <autoware/cuda_pointcloud_preprocessor/point_types.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <iostream>
#include <memory>

namespace autoware::cuda_ground_segmentation
{

using autoware::vehicle_info_utils::VehicleInfo;
enum SegmentationMode : uint8_t { UNINITIALIZED = 0, CONTINUOUS, DISCONTINUOUS, BREAK };

struct PointTypeStruct
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
};

enum class PointType : uint8_t {
  INIT = 0,
  GROUND,
  NON_GROUND,
  POINT_FOLLOW,
  UNKNOWN,
  VIRTUAL_GROUND,
  OUT_OF_RANGE
};

struct ClassifiedPointTypeStruct
{
  float z;
  PointType type;
  float radius;
  size_t origin_index;  // index in the original point cloud

  ClassifiedPointTypeStruct() : z(0.0), type(PointType::INIT), radius(-1.0), origin_index(0) {}
};

struct CellCentroid
{
  int cell_id;  // cell_id = sector_id * number_cells_per_sector + grid_index
  int sector_id;
  int cell_id_in_sector;
  int num_points;
  size_t start_point_index;  // start index of points in classified_points_dev
  float gnd_radius_avg;
  float gnd_height_avg;
  float gnd_height_min;
  int num_ground_points;
  // initialize constructor
  CellCentroid()
  : cell_id(0),
    sector_id(0),
    cell_id_in_sector(0),
    num_points(0),
    start_point_index(0),
    gnd_radius_avg(0.0f),
    gnd_height_avg(0.0f),
    gnd_height_min(0.0f),
    num_ground_points(0)
  {
  }
};

// structure to hold parameter values
struct FilterParameters
{
  float max_radius;
  float non_ground_height_threshold;
  float center_pcl_shift{0.0f};  // virtual center of pcl to center mass

  // common thresholds
  float global_slope_max_angle_rad;  // radians
  float local_slope_max_angle_rad;   // radians
  float global_slope_max_ratio;
  float local_slope_max_ratio;
  float split_points_distance_tolerance;  // distance in meters between concentric divisions

  // non-cell mode parameters
  bool use_virtual_ground_point;
  float split_height_distance;  // minimum height threshold regardless the slope,
                                // useful for close points

  // common parameters
  float sector_angle_rad;  // radial sector angle in radians
  float inv_sector_angle_rad;
  uint16_t num_sectors;  // number of radial sectors
  VehicleInfo vehicle_info;

  // cell mode parameters
  bool use_recheck_ground_cluster;  // to enable recheck ground cluster
  float recheck_start_distance;     // distance to start rechecking ground cluster
  bool use_lowest_point;  // to select lowest point for reference in recheck ground cluster,
                          // otherwise select middle point
  float detection_range_z_max;

  // cell parameters
  float cell_divider_size_m;
  int max_num_cells_per_sector;  // number of cells per sector
  int max_num_cells;
  uint16_t gnd_cell_buffer_size;
  float virtual_lidar_z;
  float center_x{0.0f};
  float center_y{0.0f};
  uint16_t gnd_grid_continual_thresh{3};  // threshold for continual ground grid
};

class CudaScanGroundSegmentationFilter
{
public:
  explicit CudaScanGroundSegmentationFilter(
    const FilterParameters & filter_parameters, const int64_t max_mem_pool_size_in_byte);
  ~CudaScanGroundSegmentationFilter() = default;

  // Method to process the point cloud data and filter ground points
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> classifyPointcloud(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

  size_t number_input_points_;
  size_t input_pointcloud_step_;
  size_t input_xyzi_offset_[4];
  float center_x_{0.0f};
  float center_y_{0.0f};
  const uint16_t gnd_grid_continual_thresh_{3};  // threshold for continual ground grid

  // Parameters
  FilterParameters filter_parameters_;

private:
  // Internal methods for ground segmentation logic

  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);
  void scanObstaclePoints(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    PointTypeStruct * output_points_dev, size_t * num_output_points,
    CellCentroid * centroid_cells_list_dev);
  /*
   * This function calc the cell_id for each point
   * Assign the point with initialized class into temp memory for classification
   * Memory size of each cell is depend on predefined cell point num
   *
   */
  void assignPointToClassifyPoint(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    const CellCentroid * centroid_cells_list_dev, const FilterParameters * filter_parameters_dev,
    int * cell_counts_dev, ClassifiedPointTypeStruct * classified_points_dev);

  void getCellFirstPointIndex(
    const FilterParameters * filter_parameters_dev, CellCentroid * centroid_cells_list_dev,
    int * num_points_per_cell_dev, int * cell_first_point_indices_dev);
  void sortPointsInCells(
    const int * num_points_per_cell_dev, ClassifiedPointTypeStruct * classified_points_dev);
  void scanPerSectorGroundReference(
    ClassifiedPointTypeStruct * classified_points_dev, CellCentroid * centroid_cells_list_dev,
    const FilterParameters * filter_parameters_dev, int * last_gnd_cells_dev);

  /*
   * Extract obstacle points from classified_points_dev into
   */
  void extractNonGroundPoints(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    ClassifiedPointTypeStruct * classified_points_dev, PointTypeStruct * output_points_dev,
    size_t & num_output_points_host);

  void getObstaclePointcloud(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    PointTypeStruct * output_points, size_t & num_output_points);
  /*
   * This function splits the input point cloud into radial divisions.
   * Each division corresponds to a specific angle range defined by the radial_divider_angle_rad.
   * The points in each division are sorted by their distance from the center of the point cloud.
   * @param input_points The input point cloud data.
   * @param indices_list_dev point to device memory where array of radial division indices will be
   * stored.
   * @note This function assumes that the input point cloud is already allocated in device memory.f
   */
  void calcPointNumInCell(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    CellCentroid * indices_list_dev, const FilterParameters * filter_parameters_dev);

  cudaStream_t ground_segment_stream_{};
  cudaMemPool_t mem_pool_{};
};
}  // namespace autoware::cuda_ground_segmentation

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_
