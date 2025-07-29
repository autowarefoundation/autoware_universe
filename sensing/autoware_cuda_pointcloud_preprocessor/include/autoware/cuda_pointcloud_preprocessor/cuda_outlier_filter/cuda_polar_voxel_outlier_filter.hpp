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
  double radius_resolution;     // Resolution in radial direction (meters)
  double azimuth_resolution;    // Resolution in azimuth direction (radians)
  double elevation_resolution;  // Resolution in elevation direction (radians)
  int voxel_points_threshold;   // Minimum points required per voxel
  double min_radius;            // Minimum radius to consider
  double max_radius;            // Maximum radius to consider

  // Return type classification parameters
  bool use_return_type_classification;  // Whether to use return type classification
  bool filter_secondary_returns;        // Whether to filter secondary returns
  int secondary_noise_threshold;        // Threshold for primary to secondary return classification

  // Diagnostics parameters
  double visibility_error_threshold;  // Threshold for visibility diagnostics

  double visibility_warn_threshold;     // Warning threshold for visibility diagnostics
  double filter_ratio_error_threshold;  // Error threshold for filter ratio diagnostics
  double filter_ratio_warn_threshold;   // Warning threshold for filter ratio diagnostics
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

  // Cosnt version (read-only element)
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
  int64_t * return_types =
    nullptr;  // This data type aligns the nature of rclcpp that return integer parameter as int64_t
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
  explicit CudaPolarVoxelOutlierFilter();

  // /** \brief Filter using regular PointXYZ (compute polar coordinates)
  //  * Uses simple occupancy threshold filtering - voxels with >= voxel_points_threshold points are
  //  kept.
  //  * No return type classification or ratio-based filtering is applied.
  //  */
  // void filterPointXyz(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points);

  /** \brief Filter using PointXYZIRCAEDT (use pre-computed polar coordinates)
   * Uses advanced two-criteria filtering when return type classification is enabled:
   * 1. Primary returns >= voxel_points_threshold
   * 2. Primary-to-secondary ratio >= secondary_noise_threshold
   * Both criteria must be satisfied for a voxel to be kept.
   */
  FilterReturn filter_point_xyzircaedt(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
    const CudaPolarVoxelOutlierFilterParameters & params);

  void set_primary_return_types(const std::vector<int64_t> & primary_types)
  {
    set_return_types(primary_types, primary_return_type_dev_);
  }
  void set_secondary_return_types(const std::vector<int64_t> & secondary_types)
  {
    set_return_types(secondary_types, secondary_return_type_dev_);
  }

protected:
  enum class ReductionType : uint8_t { Min, Max, Sum };
  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};
  CubExecutor cub_executor_;

  std::optional<ReturnTypeCandidates> primary_return_type_dev_;
  std::optional<ReturnTypeCandidates> secondary_return_type_dev_;
  static constexpr int invalid_index_ = -1;

  void set_return_types(
    const std::vector<int64_t> & types, std::optional<ReturnTypeCandidates> & types_dev);

  // std::tuple<FieldDataComposer<int>, CudaPooledUniquePtr<::cuda::std::optional<int>>>
  // calculate_voxel_index(
  std::tuple<int, CudaPooledUniquePtr<::cuda::std::optional<int>>, CudaPooledUniquePtr<int>>
  calculate_voxel_index(
    const FieldDataComposer<::cuda::std::optional<int> *> & polar_voxel_indices,
    const size_t & num_points);

  std::tuple<CudaPooledUniquePtr<int>, size_t> calculate_filtered_point_indices(
    const CudaPooledUniquePtr<bool> & valid_points, const size_t & num_points);

  /** \bried perform device reduction operation on the specified array on the device and copy the
   *  result to the host.
   *
   *  Since this function calls cudaMemcpyAsync and does not call cudaStreamSynchronize inside
   * (to make synchronization controll under the contoll of caller), the device memory region that
   * the reduction result will be stored needs to be valid (not released) until
   * cudaStreamSynchronize is called. Hence, this function takes it as argument because allocating
   * such region in the function may cause potential memory release before synchronization (i.e.,
   * memory copy complete)
   */
  template <typename T, typename U>
  void reduce_and_copy_to_host(
    const ReductionType reduction_type, const T & dev_array, const size_t & array_length,
    U * result_dev, U & result_host);
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_OUTLIER_FILTER__CUDA_POLAR_VOXEL_OUTLIER_FILTER_HPP_
/* *INDENT-ON* */
