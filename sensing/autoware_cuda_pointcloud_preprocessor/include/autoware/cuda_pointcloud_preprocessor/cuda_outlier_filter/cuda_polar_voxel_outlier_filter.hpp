// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* *INDENT-OFF* */
#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cub_executor.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cuda/std/optional>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cassert>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
// Helper structs

struct CudaPolarVoxelOutlierFilterParameters
{
  // Polar voxel parameters
  double radial_resolution_m;                // Resolution in radial direction (meters)
  double azimuth_resolution_rad;             // Resolution in azimuth direction (radians)
  double elevation_resolution_rad;           // Resolution in elevation direction (radians)
  int voxel_points_threshold;                // Minimum points required per voxel
  double min_radius_m;                       // Minimum radius to consider
  double max_radius_m;                       // Maximum radius to consider
  double visibility_estimation_max_range_m;  // Maximum range for visibility estimation (meters)
  int visibility_estimation_max_secondary_voxel_count;  // Maximum secondary voxel count for
                                                        // visibility estimation
  bool visibility_estimation_only;  // Only estimate visibility without pointcloud processing

  // Return type classification parameters
  bool use_return_type_classification;  // Whether to use return type classification
  bool filter_secondary_returns;        // Whether to filter secondary returns
  int secondary_noise_threshold;        // Threshold for primary to secondary return classification
  int intensity_threshold;              // Maximum intensity for secondary returns

  // Diagnostics parameters
  double visibility_error_threshold;  // Threshold for visibility diagnostics

  double visibility_warn_threshold;     // Warning threshold for visibility diagnostics
  double filter_ratio_error_threshold;  // Error threshold for filter ratio diagnostics
  double filter_ratio_warn_threshold;   // Warning threshold for filter ratio diagnostics

  bool publish_noise_cloud;  // Generate and publish noise cloud for debugging
};

// Helper struct to make passing data to CUDA Kernel easy
enum class FieldDataIndex : uint8_t { radius = 0, azimuth, elevation };
//// define iterators to enable range-based loop
FieldDataIndex begin(FieldDataIndex)
{
  /*
   * begin function returns the first enum value
   */
  return FieldDataIndex::radius;
}
FieldDataIndex end(FieldDataIndex)
{
  /*
   * end function returns the enum value past the last element
   */
  using base_type = std::underlying_type<FieldDataIndex>::type;
  return static_cast<FieldDataIndex>(static_cast<base_type>(FieldDataIndex::elevation) + 1);
}
FieldDataIndex operator*(FieldDataIndex f)
{
  /*
   * dereference operator
   */
  return f;
}
FieldDataIndex operator++(FieldDataIndex & f)
{
  /*
   * increment operator for enum class
   */
  using base_type = std::underlying_type<FieldDataIndex>::type;
  return f = static_cast<FieldDataIndex>(static_cast<base_type>(f) + 1);
}

template <typename T>
struct FieldDataComposer
{
  T radius;
  T azimuth;
  T elevation;

  // Non-const version (modifiable element)
  __host__ __device__ T & operator[](FieldDataIndex i)
  {
    switch (i) {
      case FieldDataIndex::radius:
        return radius;
      case FieldDataIndex::azimuth:
        return azimuth;
      case FieldDataIndex::elevation:
        return elevation;
      default:
        assert(0);  // Since std::runtime_error can not be called from __device__function, here use
                    // assert
    }
  }

  // Const version (read-only element)
  __host__ __device__ const T & operator[](FieldDataIndex i) const
  {
    switch (i) {
      case FieldDataIndex::radius:
        return radius;
      case FieldDataIndex::azimuth:
        return azimuth;
      case FieldDataIndex::elevation:
        return elevation;
      default:
        assert(0);  // Since std::runtime_error can not be called from __device__function, here use
                    // assert
    }
  }
};

struct ReturnTypeCandidates
{
  int * return_types = nullptr;
  size_t num_candidates = 0;
};

template <typename T>
using CudaPooledUniquePtr = autoware::cuda_utils::CudaPooledUniquePtr<T>;

class CudaPolarVoxelOutlierFilter
{
public:
  struct FilterReturn
  {
    std::unique_ptr<cuda_blackboard::CudaPointCloud2> filtered_cloud;
    std::unique_ptr<cuda_blackboard::CudaPointCloud2> noise_cloud;
    double filter_ratio;
    double visibility;
  };

  enum class PolarDataType : uint8_t { PreComputed, DeriveFromCartesian };

  CudaPolarVoxelOutlierFilter();

  /**
   * \brief Filters a point cloud based on polar voxel grid parameters.
   *
   * This function performs the main filtering process, dividing the point cloud into a
   * polar voxel grid and removing voxels with insufficient points.
   *
   * \param input_cloud A shared pointer to the input point cloud.
   * \param params Parameters controling the filtering process (resolution, thresholds, etc.).
   * \param polar_type Specifies how polar data is handled (pre-computed or derived from Cartesian).
   *
   * \return A FilterReturn struct containing the filtered cloud, noise cloud, filter ratio, and
   * visibility.
   */
  FilterReturn filter(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
    const CudaPolarVoxelOutlierFilterParameters & params, const PolarDataType polar_type);

  /**
   * \brief Sets the primary return types for filtering.
   *
   * This function sets the return types that are considered as primary returns,
   * used for filtering secondary returns or noise.
   *
   * \param primary_types A vector of integers representing the primary return types.
   */
  void set_primary_return_types(const std::vector<int> & primary_types)
  {
    set_return_types(primary_types, primary_return_type_dev_);
  }

protected:
  enum class ReductionType : uint8_t { Min, Max, Sum };

  /** \brief Copy `types`, which lies on the host memory, to `types_dev`, which is on the device
   *  memory
   */
  void set_return_types(
    const std::vector<int> & types, std::optional<ReturnTypeCandidates> & types_dev);

  /**
   * \brief Calculates the voxel index for each point in the point cloud.
   *
   * This function determines the index of the voxel that each point belongs to within the polar
   * voxel grid.
   *
   * \param polar_voxel_indices A pointer to the array of polar voxel indices.
   * \param num_points The number of points in the point cloud.
   *
   * \return A tuple containing the number of valid voxels, the point index array, and
   * corresponding voxel index array for each points
   */
  std::tuple<int, CudaPooledUniquePtr<::cuda::std::optional<int>>, CudaPooledUniquePtr<int>>
  calculate_voxel_index(
    const FieldDataComposer<::cuda::std::optional<int32_t> *> & polar_voxel_indices,
    const size_t & num_points);

  /**
   * \brief Calculates the indices of points after filtering
   *
   * This function identifies the points that meet the criteria for being considered valid points.
   *
   * \param valid_points A pointer to the array indicating valid points.
   * \param num_points The number of points in the point cloud.
   *
   * \return A tuple containing the filtered(valid) point indices and the number of valid points.
   */
  std::tuple<CudaPooledUniquePtr<int>, size_t> calculate_filtered_point_indices(
    const CudaPooledUniquePtr<bool> & valid_points, const size_t & num_points);

  /** \brief perform device reduction operation on the specified array on the device and copy the
   *  result to the host.
   *
   *  Since this function calls cudaMemcpyAsync and does not call cudaStreamSynchronize inside
   * (to make synchronization control under the caller), the device memory region that
   * the reduction result will be stored needs to be valid (not released) until
   * cudaStreamSynchronize is called. Hence, this function takes it as argument because allocating
   * such region in the function may cause potential memory release before synchronization (i.e.,
   * memory copy complete)
   * \param reduction_type The type of reduction operation to perform (Min, Max, Sum).
   * \param dev_array The array to reduce on the device.
   * \param array_length The length of the array.
   * \param result_dev A pointer to the device memory where the result will be stored.
   * \param result_host A pointer to the host memory where the result will be copied.
   */
  template <typename T, typename U>
  void reduce_and_copy_to_host(
    const ReductionType reduction_type, const T & dev_array, const size_t & array_length,
    U * result_dev, U & result_host);

  /**
   * \brief Creates the output point cloud from the filtered points.
   *
   * This function creates a new point cloud containing the filtered points.
   *
   * \param input_cloud A shared pointer to the input point cloud.
   * \param points_mask A unique pointer to the mask indicating the filtered points.
   * \param num_points The number of points in the point cloud.
   * \param output_cloud A unique pointer to the output point cloud.
   *
   * \return The number of points in the output point cloud.
   */
  size_t create_output(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
    const CudaPooledUniquePtr<bool> & points_mask, const size_t & num_points,
    std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_cloud);

private:
  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};
  CubExecutor cub_executor_;

  std::optional<ReturnTypeCandidates> primary_return_type_dev_;
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_HPP_
/* *INDENT-ON* */
