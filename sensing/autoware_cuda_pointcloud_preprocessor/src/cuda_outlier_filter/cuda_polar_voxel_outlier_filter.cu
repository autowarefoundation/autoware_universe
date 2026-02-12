// Copyright 2025 TIER IV, Inc.
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_outlier_filter/cuda_polar_voxel_outlier_filter.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"
#include "autoware/cuda_utils/cuda_memory_pool.hpp"
#include "autoware/cuda_utils/cuda_unique_ptr.hpp"

#include <cub/cub.cuh>
#include <cuda/functional>    // for cuda::proclaim_return_type
#include <cuda/std/optional>  // for cuda::std::optional
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

#include <thrust/iterator/transform_iterator.h>

#include <cstdint>
#include <limits>
#include <locale>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
constexpr size_t point_cloud_height_organized = 1;

struct ValidAndNotEqualTo
{
  /**
   * Functor (originally intended being consumed by
   * cub::DeviceAdjacentDifference::SubtractLeft(Copy))
   * which calculate `output[i] = difference_op(input[i], input[i-1])` for each element

   * |...| i-2 | i-1 (=lhs) | i (=rhs) | i+1 |...|
   *
   * Since the operator takes optional, the comparison criteria is as follows considering nullopt
   *
   * | lhs \ rhs | nullopt | has_value                  |
   * | nullopt   | false   | true                       |
   * | has_value | false   | lhs.value() != rhs.value() |
   */
  template <typename DataType>
  __host__ __device__ bool operator()(
    const ::cuda::std::optional<DataType> & lhs, const ::cuda::std::optional<DataType> & rhs)
  {
    return rhs && (!lhs || lhs.value() != rhs.value());
  }
};

struct NulloptToMax
{
  /*
   * Functor to think cuda::std::nullopt as ::cuda::std::numeric_limits<int>::max()
   */
  template <typename DataType>
  __host__ __device__ __forceinline__ DataType
  operator()(const ::cuda::std::optional<DataType> & opt) const
  {
    return opt.has_value() ? opt.value() : ::cuda::std::numeric_limits<DataType>::max();
  }
};

struct NulloptToLowest
{
  /*
   * Functor to think cuda::std::nullopt as ::cuda::std::numeric_limits<int>::lowest()
   * NOTE: std::numeric_limits::min returns the minimum, non-zero, positive value while
   * std::numeric_limits::lowest() returns the negative minimum value
   */
  template <typename DataType>
  __host__ __device__ __forceinline__ DataType
  operator()(const ::cuda::std::optional<DataType> & opt) const
  {
    return opt.has_value() ? opt.value() : ::cuda::std::numeric_limits<DataType>::lowest();
  }
};

/** \brief factory utility function to allocate unique_ptr and FieldDataComposer with allocated
 * pointer
 */
template <typename T>
[[nodiscard]] std::tuple<
  CudaPooledUniquePtr<T>, CudaPooledUniquePtr<T>, CudaPooledUniquePtr<T>, FieldDataComposer<T *>>
generate_field_data_composer(const size_t & num_elems, cudaStream_t & stream, cudaMemPool_t & pool)
{
  auto radius = autoware::cuda_utils::make_unique<T>(num_elems, stream, pool);
  auto azimuth = autoware::cuda_utils::make_unique<T>(num_elems, stream, pool);
  auto elevation = autoware::cuda_utils::make_unique<T>(num_elems, stream, pool);

  FieldDataComposer<T *> composer;
  composer.radius = radius.get();
  composer.azimuth = azimuth.get();
  composer.elevation = elevation.get();
  return std::make_tuple(
    std::move(radius), std::move(azimuth), std::move(elevation), std::move(composer));
}

/** \brief atomicAdd operation for size_t
 *
 * Since cuda's builtin atomicAdd does not have overloaded function for size_t,
 * and the definition may differ according to platforms, this function define atomicAdd operation
 * with size check
 */
__device__ void atomic_add_size_t(size_t * addr, size_t val)
{
  if constexpr (sizeof(size_t) == sizeof(unsigned int)) {
    atomicAdd(reinterpret_cast<unsigned int *>(addr), static_cast<unsigned int>(val));
  } else if constexpr (sizeof(size_t) == sizeof(unsigned long long int)) {
    atomicAdd(
      reinterpret_cast<unsigned long long int *>(addr), static_cast<unsigned long long int>(val));
  } else {
    static_assert(
      sizeof(size_t) == sizeof(unsigned int) || sizeof(size_t) == sizeof(unsigned long long int),
      "atomicAdd_size_t is only supported for size_t sizes equal to unsigned int or unsigned long "
      "long int.");
  }
}

__device__ [[nodiscard]] inline bool meets_primary_threshold(
  const size_t & count, const int & threshold)
{
  return count >= static_cast<size_t>(threshold);
}

__device__ [[nodiscard]] inline bool meets_secondary_threshold(
  const size_t & count, const int & threshold)
{
  return count <= static_cast<size_t>(threshold);
}

__device__ [[nodiscard]] inline bool meets_intensity_threshold(
  const uint8_t intensity, const uint8_t threshold)
{
  return intensity <= threshold;
}

template <typename T>
__device__ T
get_element_value(const uint8_t * data, const size_t index, const size_t step, const size_t offset)
{
  return *reinterpret_cast<const T *>(data + index * step + offset);
}

template <typename T>
__device__ bool check_within_radius_range(
  const FieldDataIndex & field_index, const T & field_data, const double & min_val,
  const double & max_val)
{
  return (field_index == FieldDataIndex::radius)
           ? (min_val <= field_data) && (field_data <= max_val)
           : true;
}

template <typename T>
__device__ bool check_sufficient_radius(const FieldDataIndex & field_index, const T & field_data)
{
  return (field_index == FieldDataIndex::radius)
           ? ::cuda::std::abs(field_data) >= ::cuda::std::numeric_limits<T>::epsilon()
           : true;
}

template <typename TFieldData>
__device__ void assign_polar_index(
  const TFieldData & field_data, const size_t & point_index, const FieldDataIndex & field_index,
  const double & min_radius, const double & max_radius,
  const FieldDataComposer<double> & resolutions,
  FieldDataComposer<::cuda::std::optional<int32_t> *> & outputs)
{
  bool is_finite = isfinite(field_data);
  bool is_within_radius_range =
    check_within_radius_range(field_index, field_data, min_radius, max_radius);

  bool has_sufficient_radius = check_sufficient_radius(field_index, field_data);

  auto output = outputs[field_index];
  if (!is_finite || !is_within_radius_range || !has_sufficient_radius) {
    // Assign invalid index for points with invalid value and/or points outside radius range
    output[point_index] = ::cuda::std::nullopt;
    return;
  }

  if constexpr (::cuda::std::is_same<TFieldData, double>::value) {
    output[point_index] = static_cast<int32_t>(floor(field_data / resolutions[field_index]));
  } else {
    output[point_index] = static_cast<int32_t>(floorf(field_data / resolutions[field_index]));
  }
}

template <typename TFieldData>
__global__ void polar_to_polar_voxel_kernel(
  const uint8_t * __restrict__ data, const size_t num_points, const uint32_t step,
  const FieldDataComposer<size_t> offsets, const FieldDataComposer<double> resolutions,
  const double min_radius, const double max_radius,
  FieldDataComposer<::cuda::std::optional<int32_t> *> outputs)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  // treat 3 field (radius, azimuth, elevation) in parallel using y dimension
  auto field_index = static_cast<FieldDataIndex>(blockIdx.y);

  TFieldData field_data =
    get_element_value<TFieldData>(data, point_index, step, offsets[field_index]);

  assign_polar_index(
    field_data, point_index, field_index, min_radius, max_radius, resolutions, outputs);
}

template <typename TCartesianData, typename TPolarData>
__global__ void cartesian_to_polar_voxel_kernel(
  const uint8_t * __restrict__ data, const size_t num_points, const uint32_t step,
  const FieldDataComposer<size_t> offsets, const FieldDataComposer<double> resolutions,
  const double min_radius, const double max_radius,
  FieldDataComposer<::cuda::std::optional<int32_t> *> outputs)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  TCartesianData x =
    get_element_value<TCartesianData>(data, point_index, step, offsets[FieldDataIndex::radius]);
  TCartesianData y =
    get_element_value<TCartesianData>(data, point_index, step, offsets[FieldDataIndex::azimuth]);
  TCartesianData z =
    get_element_value<TCartesianData>(data, point_index, step, offsets[FieldDataIndex::elevation]);

  // treat 3 field (radius, azimuth, elevation) in parallel using
  // y dimension
  auto field_index = static_cast<FieldDataIndex>(blockIdx.y);

  auto field_data = static_cast<TPolarData>(0);
  if constexpr (::cuda::std::is_same<TPolarData, double>::value) {
    switch (field_index) {
      case FieldDataIndex::radius:
        field_data = sqrt(x * x + y * y + z * z);
        break;
      case FieldDataIndex::azimuth:
        field_data = atan2(y, x);
        break;
      case FieldDataIndex::elevation:
        field_data = atan2(z, sqrt(x * x + y * y));
        break;
    }
  } else {
    switch (field_index) {
      case FieldDataIndex::radius:
        field_data = sqrtf(x * x + y * y + z * z);
        break;
      case FieldDataIndex::azimuth:
        field_data = atan2f(y, x);
        break;
      case FieldDataIndex::elevation:
        field_data = atan2f(z, sqrtf(x * x + y * y));
        break;
    }
  }

  assign_polar_index(
    field_data, point_index, field_index, min_radius, max_radius, resolutions, outputs);
}

__global__ void calculate_voxel_index_kernel(
  const FieldDataComposer<::cuda::std::optional<int32_t> *> field_indices, const size_t num_points,
  const FieldDataComposer<int> field_dimensions, const FieldDataComposer<int> field_mins,
  ::cuda::std::optional<int> * point_indices, ::cuda::std::optional<int> * voxel_indices)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  auto radius_idx = field_indices[FieldDataIndex::radius][point_index];
  auto azimuth_idx = field_indices[FieldDataIndex::azimuth][point_index];
  auto elevation_idx = field_indices[FieldDataIndex::elevation][point_index];

  auto radius_idx_min = field_mins[FieldDataIndex::radius];
  auto azimuth_idx_min = field_mins[FieldDataIndex::azimuth];
  auto elevation_idx_min = field_mins[FieldDataIndex::elevation];

  auto radius_dim = field_dimensions[FieldDataIndex::radius];
  auto azimuth_dim = field_dimensions[FieldDataIndex::azimuth];

  // Save point index and corresponding voxel index in the same index position of two arrays
  point_indices[point_index] = point_index;
  if (!radius_idx || !azimuth_idx || !elevation_idx) {
    // If any of field indices has invalid value, voxel index for this point will also be invalid
    voxel_indices[point_index] = ::cuda::std::nullopt;
    return;
  }

  // Because the following index calculation assumes all indices are zero started, positive values,
  // make the conditions meet by subtracting the minimum values of each filed
  auto radius_idx_shifted = radius_idx.value() - radius_idx_min;
  auto azimuth_idx_shifted = azimuth_idx.value() - azimuth_idx_min;
  auto elevation_idx_shifted = elevation_idx.value() - elevation_idx_min;

  voxel_indices[point_index] = elevation_idx_shifted * (radius_dim * azimuth_dim) +
                               azimuth_idx_shifted * radius_dim + radius_idx_shifted;
}

__global__ void subtract_left_optional_kernel(
  const ::cuda::std::optional<int> * __restrict__ input_array, const size_t array_length,
  bool * __restrict__ output_array)
{
  auto array_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (array_index >= array_length) {
    return;
  }

  // Specially handle the very first element
  if (array_index == 0) {
    output_array[array_index] = input_array[array_index].has_value();
    return;
  }

  auto difference_op = ValidAndNotEqualTo();
  output_array[array_index] = difference_op(input_array[array_index - 1], input_array[array_index]);
}

__global__ void minus_one_kernel(int * __restrict__ indices, const size_t num_points)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  indices[point_index] -= 1;
}

template <typename TReturnType, typename TIntensity>
__global__ void classify_point_by_return_type_and_intensity_kernel(
  const uint8_t * __restrict__ data, const size_t num_points, const int num_voxels,
  const size_t step, const size_t return_type_offset, const size_t intensity_offset,
  const ReturnTypeCandidates primary_return_type, const uint8_t intensity_threshold,
  const ::cuda::std::optional<int> * __restrict__ point_indices,
  const int * __restrict__ voxel_indices,
  const ::cuda::std::optional<int> * __restrict__ radius_indices, const double radial_resolution_m,
  const double visibility_estimation_max_range_m, size_t * __restrict__ primary_returns,
  size_t * __restrict__ secondary_returns, int * __restrict__ is_in_visibility_range,
  bool * __restrict__ is_primary_returns, bool * __restrict__ is_secondary_returns)
{
  auto array_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (array_index >= num_points) {
    return;
  }

  auto point_index = point_indices[array_index];  // target point this thread treats
  if (!point_index) {
    return;
  }

  auto voxel_index = voxel_indices[array_index];  // which voxel does this point belongs to
  if (voxel_index < 0 || num_voxels <= voxel_index) {
    return;
  }

  auto return_type =
    get_element_value<TReturnType>(data, point_index.value(), step, return_type_offset);

  auto intensity = get_element_value<TIntensity>(data, point_index.value(), step, intensity_offset);

  auto find = [] __device__(ReturnTypeCandidates candidates, TReturnType value) -> bool {
    bool is_found = false;
    for (size_t i = 0; i < candidates.num_candidates; i++) {
      is_found = is_found || (candidates.return_types[i] == value);
    }
    return is_found;
  };

  auto is_primary_return_type = find(primary_return_type, return_type);
  is_primary_returns[point_index.value()] = is_primary_return_type;
  auto is_secondary_return_type = meets_intensity_threshold(intensity, intensity_threshold);
  is_secondary_returns[point_index.value()] = !is_primary_return_type && is_secondary_return_type;

  if (is_primary_return_type) {
    atomic_add_size_t(&(primary_returns[voxel_index]), 1);
  } else if (is_secondary_return_type) {
    atomic_add_size_t(&(secondary_returns[voxel_index]), 1);
  }

  // Add range information for visibility calculation
  if (!radius_indices[point_index.value()].has_value()) {
    atomicExch(&(is_in_visibility_range[voxel_index]), false);
  } else {
    double voxel_max_radius =
      (radius_indices[point_index.value()].value() + 1) * radial_resolution_m;
    atomicExch(
      &(is_in_visibility_range[voxel_index]),
      voxel_max_radius <= visibility_estimation_max_range_m);
  }
}

__global__ void criterion_check_kernel(
  const size_t * __restrict__ primary_returns, const size_t * __restrict__ secondary_returns,
  const size_t num_total_voxels, const int voxel_points_threshold,
  const int secondary_noise_threshold, bool * __restrict__ primary_meets_threshold,
  bool * __restrict__ secondary_meets_threshold)
{
  auto voxel_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (voxel_index >= num_total_voxels) {
    return;
  }

  // Check criterion 1: Primary returns meet the threshold
  primary_meets_threshold[voxel_index] =
    meets_primary_threshold(primary_returns[voxel_index], voxel_points_threshold);

  // Check criterion 2: Number of secondary returns is less than the threshold
  secondary_meets_threshold[voxel_index] =
    meets_secondary_threshold(secondary_returns[voxel_index], secondary_noise_threshold);
}

__global__ void point_validity_check_kernel(
  const bool * __restrict__ primary_meets_threshold,
  const bool * __restrict__ secondary_meets_threshold,
  const ::cuda::std::optional<int> * __restrict__ point_indices,
  const int * __restrict__ voxel_indices, const bool * __restrict__ is_primary_returns,
  const size_t num_points, const int num_voxels, const bool filter_secondary_returns,
  bool * __restrict__ valid_points_mask)
{
  auto array_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (array_index >= num_points) {
    return;
  }

  auto point_index = point_indices[array_index];
  if (!point_index) {
    return;
  }

  // voxel index that this point belongs to
  auto voxel_index = voxel_indices[array_index];
  if (voxel_index < 0 || num_voxels <= voxel_index) {  // Invalid voxel index means this point is
                                                       // invalid
    valid_points_mask[point_index.value()] = false;
    return;
  }

  // Voxel is kept if BOTH criteria are met
  auto meet_voxel_criteria =
    primary_meets_threshold[voxel_index] && secondary_meets_threshold[voxel_index];

  valid_points_mask[point_index.value()] =
    meet_voxel_criteria && (!filter_secondary_returns || is_primary_returns[point_index.value()]);
}

__global__ void copy_valid_points_kernel(
  const uint8_t * __restrict__ input_cloud, const bool * __restrict__ valid_points_mask,
  const int * __restrict__ filtered_indices, const size_t num_points, const size_t step,
  uint8_t * __restrict__ output_points)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  bool is_valid = valid_points_mask[point_index];
  int destination_index = filtered_indices[point_index];

  if (is_valid) {
    memcpy(output_points + destination_index * step, input_cloud + point_index * step, step);
  }
}

__global__ void bool_flip_kernel(bool * __restrict__ flags, const size_t num_points)
{
  auto point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  flags[point_index] = !flags[point_index];
}

__global__ void is_low_visibility_voxel_kernel(
  const bool * __restrict__ secondary_meets_threshold,
  const int * __restrict__ is_in_visibility_range, const int num_voxels,
  bool * __restrict__ is_low_visibility_voxels)
{
  auto voxel_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (voxel_index >= num_voxels) {
    return;
  }

  is_low_visibility_voxels[voxel_index] = static_cast<bool>(is_in_visibility_range[voxel_index]) &&
                                          !secondary_meets_threshold[voxel_index];
}

// Helper function to get field offset
template <typename T>
size_t get_offset(const T & fields, const std::string & field_name)
{
  int index = -1;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (fields[i].name == field_name) {
      index = static_cast<int>(i);
      break;
    }
  }
  if (index < 0) {
    std::stringstream ss;
    ss << "input cloud does not contain field named '" << field_name << "'";
    throw std::runtime_error(ss.str());
  }
  return fields[index].offset;
}
}  // namespace

CudaPolarVoxelOutlierFilter::CudaPolarVoxelOutlierFilter() : primary_return_type_dev_(std::nullopt)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // create memory pool to make repeated allocation efficient
  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  size_t max_mem_pool_size_in_byte = 1e9;  // 1GB
  mem_pool_ =
    autoware::cuda_utils::create_memory_pool(max_mem_pool_size_in_byte, current_device_id);
}

CudaPolarVoxelOutlierFilter::FilterReturn CudaPolarVoxelOutlierFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
  const CudaPolarVoxelOutlierFilterParameters & params, const PolarDataType polar_type)
{
  if (!primary_return_type_dev_) {
    return FilterReturn{nullptr, nullptr, 0., 0.};
  }

  size_t num_points = input_cloud->width * input_cloud->height;
  if (num_points == 0) {
    // sometimes topic might contain zero point even the pointer is valid
    // For such cases, this filter returns empty results
    auto empty_filtered_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    auto empty_noise_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    return FilterReturn{std::move(empty_filtered_cloud), std::move(empty_noise_cloud), 0., 0.};
  }

  FieldDataComposer<size_t> offsets{};
  switch (polar_type) {
    case PolarDataType::PreComputed:
      offsets.radius = get_offset(input_cloud->fields, "distance");
      offsets.azimuth = get_offset(input_cloud->fields, "azimuth");
      offsets.elevation = get_offset(input_cloud->fields, "elevation");
      break;
    case PolarDataType::DeriveFromCartesian:
      // Though struct member names assume polar coordinates one,
      // fill offset for cartesian coordinates to compute polar coordinates
      offsets.radius = get_offset(input_cloud->fields, "x");
      offsets.azimuth = get_offset(input_cloud->fields, "y");
      offsets.elevation = get_offset(input_cloud->fields, "z");
      break;
    default:
      throw std::runtime_error("undefined polar_data_type is specified");
  }

  int num_total_voxels = 0;
  CudaPooledUniquePtr<::cuda::std::optional<int>> point_indices = nullptr;
  CudaPooledUniquePtr<int> voxel_indices = nullptr;
  // the number of points that have (primary|secondary) returns per voxels
  CudaPooledUniquePtr<size_t> num_primary_returns = nullptr;
  CudaPooledUniquePtr<size_t> num_secondary_returns = nullptr;
  CudaPooledUniquePtr<int> is_in_visibility_range =
    nullptr;  // Due to lack of bool support for CUDA's atomic operations, use int here

  // flags whether each points has (primary|secondary) return type (for later use)
  CudaPooledUniquePtr<bool> is_primary_returns = nullptr;
  CudaPooledUniquePtr<bool> is_secondary_returns = nullptr;

  // First pass: count points in each polar voxel using pre-computed coordinates
  {
    auto [radius_idx, azimuth_idx, elevation_idx, polar_voxel_indices] =
      generate_field_data_composer<::cuda::std::optional<int32_t>>(num_points, stream_, mem_pool_);

    FieldDataComposer<double> resolutions{};
    resolutions.radius = params.radial_resolution_m;
    resolutions.azimuth = params.azimuth_resolution_rad;
    resolutions.elevation = params.elevation_resolution_rad;

    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x, 3, 1);

    switch (polar_type) {
      case PolarDataType::PreComputed:
        polar_to_polar_voxel_kernel<float><<<grid_dim, block_dim, 0, stream_>>>(
          input_cloud->data.get(), num_points, input_cloud->point_step, offsets, resolutions,
          params.min_radius_m, params.max_radius_m, polar_voxel_indices);
        break;
      case PolarDataType::DeriveFromCartesian:
        cartesian_to_polar_voxel_kernel<float, float><<<grid_dim, block_dim, 0, stream_>>>(
          input_cloud->data.get(), num_points, input_cloud->point_step, offsets, resolutions,
          params.min_radius_m, params.max_radius_m, polar_voxel_indices);
        break;
      default:
        throw std::runtime_error("undefined polar_data_type is specified");
    }

    // calculate unique voxel index that each point belongs to
    std::tie(num_total_voxels, point_indices, voxel_indices) =
      calculate_voxel_index(polar_voxel_indices, num_points);

    //  count the number of primary/secondary returns of each voxel
    num_primary_returns =
      autoware::cuda_utils::make_unique<size_t>(num_total_voxels, stream_, mem_pool_);
    num_secondary_returns =
      autoware::cuda_utils::make_unique<size_t>(num_total_voxels, stream_, mem_pool_);
    is_in_visibility_range =
      autoware::cuda_utils::make_unique<int>(num_total_voxels, stream_, mem_pool_);
    is_primary_returns = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);
    is_secondary_returns = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);

    // Add points to appropriate vector based on return type
    size_t return_type_offset = get_offset(input_cloud->fields, "return_type");
    size_t intensity_offset = get_offset(input_cloud->fields, "intensity");
    grid_dim = dim3((num_points + block_dim.x - 1) / block_dim.x);
    classify_point_by_return_type_and_intensity_kernel<uint8_t, uint8_t>
      <<<grid_dim, block_dim, 0, stream_>>>(
        input_cloud->data.get(), num_points, num_total_voxels, input_cloud->point_step,
        return_type_offset, intensity_offset, primary_return_type_dev_.value(),
        static_cast<uint8_t>(params.intensity_threshold), point_indices.get(), voxel_indices.get(),
        radius_idx.get(), resolutions.radius, params.visibility_estimation_max_range_m,
        num_primary_returns.get(), num_secondary_returns.get(), is_in_visibility_range.get(),
        is_primary_returns.get(), is_secondary_returns.get());
  }

  // Collect valid point indices and visibility statistics
  auto valid_points_mask = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);
  auto primary_meets_threshold =
    autoware::cuda_utils::make_unique<bool>(num_total_voxels, stream_, mem_pool_);
  auto secondary_meets_threshold =
    autoware::cuda_utils::make_unique<bool>(num_total_voxels, stream_, mem_pool_);
  {
    // Here we check if the voxel meets the following criteria:
    // 1. Has more points in the primary returns than the threshold
    // 2. Has fewer points in the secondary returns than the secondary noise threshold
    dim3 block_dim(512);
    dim3 grid_dim((num_total_voxels + block_dim.x - 1) / block_dim.x);

    criterion_check_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      num_primary_returns.get(), num_secondary_returns.get(), num_total_voxels,
      params.voxel_points_threshold, params.secondary_noise_threshold,
      primary_meets_threshold.get(), secondary_meets_threshold.get());

    grid_dim = dim3((num_points + block_dim.x - 1) / block_dim.x);

    point_validity_check_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      primary_meets_threshold.get(), secondary_meets_threshold.get(), point_indices.get(),
      voxel_indices.get(), is_primary_returns.get(), num_points, num_total_voxels,
      params.filter_secondary_returns, valid_points_mask.get());
  }

  // Create filtered output
  size_t valid_count = 0;
  auto filtered_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  valid_count = create_output(input_cloud, valid_points_mask, num_points, filtered_cloud);

  // Create noise cloud with filtered-out points
  auto noise_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  if (!params.visibility_estimation_only && params.publish_noise_cloud) {
    // Flip valid flag to get filtered-out (= noise) points
    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);

    bool_flip_kernel<<<grid_dim, block_dim, 0, stream_>>>(valid_points_mask.get(), num_points);

    std::ignore = create_output(input_cloud, valid_points_mask, num_points, noise_cloud);
  }

  // Calculate filter ratio and visibility (only when return type classification is enabled)
  auto filter_ratio =
    num_points > 0 ? static_cast<double>(valid_count) / static_cast<double>(num_points) : 0.0;

  double visibility = 0.0;
  if (params.use_return_type_classification) {
    auto is_low_visibility_voxels =
      autoware::cuda_utils::make_unique<bool>(num_total_voxels, stream_, mem_pool_);

    dim3 block_dim(512);
    dim3 grid_dim((num_total_voxels + block_dim.x - 1) / block_dim.x);

    is_low_visibility_voxel_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      secondary_meets_threshold.get(), is_in_visibility_range.get(), num_total_voxels,
      is_low_visibility_voxels.get());

    int low_visibility_voxels_count = 0;
    auto tmp_low_visibility_voxels_dev = autoware::cuda_utils::make_unique<int>(stream_, mem_pool_);
    reduce_and_copy_to_host(
      ReductionType::Sum, is_low_visibility_voxels.get(), num_total_voxels,
      tmp_low_visibility_voxels_dev.get(), low_visibility_voxels_count);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));  // wait till device to host copy finish

    visibility = std::max(
      0.0, 1.0 - static_cast<double>(low_visibility_voxels_count) /
                   static_cast<double>(params.visibility_estimation_max_secondary_voxel_count));
  }

  return {std::move(filtered_cloud), std::move(noise_cloud), filter_ratio, visibility};
}

void CudaPolarVoxelOutlierFilter::set_return_types(
  const std::vector<int> & types, std::optional<ReturnTypeCandidates> & types_dev)
{
  if (types_dev) {
    // Reset previously allocated region to refresh the parameters
    CHECK_CUDA_ERROR(cudaFreeAsync(types_dev.value().return_types, stream_));
  }

  auto num_candidates = types.size();
  using return_type_t = decltype(ReturnTypeCandidates::return_types);
  return_type_t return_type = nullptr;

  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(
    &return_type, num_candidates * sizeof(return_type_t), mem_pool_, stream_));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    return_type, types.data(), num_candidates * sizeof(return_type_t), cudaMemcpyHostToDevice,
    stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  types_dev = ReturnTypeCandidates{return_type, types.size()};
}

std::tuple<int, CudaPooledUniquePtr<::cuda::std::optional<int>>, CudaPooledUniquePtr<int>>
CudaPolarVoxelOutlierFilter::calculate_voxel_index(
  const FieldDataComposer<::cuda::std::optional<int32_t> *> & polar_voxel_indices,
  const size_t & num_points)
{
  // Step 1: determine the range of each field indices by taking min and max
  FieldDataComposer<int> polar_voxel_indices_min{0, 0, 0};
  FieldDataComposer<int> polar_voxel_indices_max{0, 0, 0};
  auto reduction_result_tmp_dev = autoware::cuda_utils::make_unique<int>(stream_, mem_pool_);
  for (const auto & i : FieldDataIndex()) {
    auto polar_voxel_index = polar_voxel_indices[i];
    auto & min_val_host = polar_voxel_indices_min[i];
    auto & max_val_host = polar_voxel_indices_max[i];

    // Because `operator<=` for cuda::std::optional considers nullopt is less than any valid value,
    // this conversion helps searching minimum valid value from the array of cuda::std::optional
    thrust::transform_iterator<NulloptToMax, ::cuda::std::optional<int> *>
      transformed_in_null_to_max(polar_voxel_index, NulloptToMax{});

    // Take Minimum value
    reduce_and_copy_to_host(
      ReductionType::Min, transformed_in_null_to_max, num_points, reduction_result_tmp_dev.get(),
      min_val_host);

    // Though comparison operators are defined for cuda::std::optional,
    // cuda::std::optional does not have ::Lowest() member, which is required for
    // cub::DeviceReduce::Max. Here, cuda::std::optional is wrapped to transform into its contained
    // value (if nullopt, then return numeric_limits::lowest) to make cub::DeviceReduce::Max work
    thrust::transform_iterator<NulloptToLowest, ::cuda::std::optional<int> *>
      transformed_in_null_to_lowest(polar_voxel_index, NulloptToLowest{});

    // Take maximum value
    reduce_and_copy_to_host(
      ReductionType::Max, transformed_in_null_to_lowest, num_points, reduction_result_tmp_dev.get(),
      max_val_host);
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));  // make sure all device to host copies complete

  // Step 2: calculate voxel's (geometric-based) linear index based of determined range by Step 1
  FieldDataComposer<int> polar_voxel_range{0, 0, 0};
  for (const auto & i : FieldDataIndex()) {
    polar_voxel_range[i] = polar_voxel_indices_max[i] - (polar_voxel_indices_min[i] - 1);
  }

  auto point_indices =
    autoware::cuda_utils::make_unique<::cuda::std::optional<int>>(num_points, stream_, mem_pool_);
  auto voxel_indices_raw =
    autoware::cuda_utils::make_unique<::cuda::std::optional<int>>(num_points, stream_, mem_pool_);

  dim3 block_dim(512);
  dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);
  calculate_voxel_index_kernel<<<grid_dim, block_dim, 0, stream_>>>(
    polar_voxel_indices, num_points, polar_voxel_range, polar_voxel_indices_min,
    point_indices.get(), voxel_indices_raw.get());

  // Step 3: Calculate the voxel indices on memory with corresponding point indices
  auto voxel_indices = autoware::cuda_utils::make_unique<int>(num_points, stream_, mem_pool_);
  int valid_voxel_num = 0;
  {
    // Sort indices
    auto sort_pairs = [](auto &&... args) {
      // Because radix sort cannot be applied to cuda::std::optional<int> without any conversion,
      // use merge sort here
      return cub::DeviceMergeSort::SortPairs(std::forward<decltype(args)>(args)...);
    };

    cub_executor_.run_with_temp_storage(
      sort_pairs, stream_, mem_pool_, voxel_indices_raw.get(), point_indices.get(), num_points,
      ::cuda::std::less<::cuda::std::optional<int>>(), stream_);

    auto voxel_indices_bool =
      autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);

    // Because implicit data conversion from cuda::std::optional<int> to bool is not supported by
    // nvcc (even though std::optional has a operator bool() to check it contains a value),
    // cub::DeviceAdjacentDifference::SubtractLeftCopy cannot be applied here. To handle this case
    // execute dedicated CUDA kernel that accepts cuda::std::optional<int> input and bool output
    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);
    subtract_left_optional_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      voxel_indices_raw.get(), num_points, voxel_indices_bool.get());

    // calculate mapped index (from geometry-based linear voxel indices to memory-based indices)
    auto inclusive_sum = [](auto &&... args) {
      return cub::DeviceScan::InclusiveSum(std::forward<decltype(args)>(args)...);
    };
    cub_executor_.run_with_temp_storage(
      inclusive_sum, stream_, mem_pool_, voxel_indices_bool.get(), voxel_indices.get(), num_points,
      stream_);

    // get the number of valid voxels
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &valid_voxel_num,
      voxel_indices.get() +
        (num_points - 1),  // the end of array contains the number of valid voxels
      sizeof(int), cudaMemcpyDeviceToHost, stream_));

    // Since the current voxel_indices contain index values starting from 1,
    //  Subtract 1 from all elements to make 0 started index
    //// NOTE: equivalent operation can be achieved cud::DeviceFor::Forereach that is introduced
    /// from / cub v2.4.0
    minus_one_kernel<<<grid_dim, block_dim, 0, stream_>>>(voxel_indices.get(), num_points);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));  // Make sure Device to Host copy complete
  }

  return std::make_tuple(valid_voxel_num, std::move(point_indices), std::move(voxel_indices));
}

std::tuple<CudaPooledUniquePtr<int>, size_t>
CudaPolarVoxelOutlierFilter::calculate_filtered_point_indices(
  const CudaPooledUniquePtr<bool> & valid_points_mask, const size_t & num_points)
{
  // Scan valid_points_mask to calculate the total number of filtered points and map from the source
  // point index to filtered point index
  auto filtered_point_indices =
    autoware::cuda_utils::make_unique<int>(num_points, stream_, mem_pool_);

  auto inclusive_scan = [](auto &&... args) {
    return cub::DeviceScan::InclusiveSum(std::forward<decltype(args)>(args)...);
  };
  cub_executor_.run_with_temp_storage(
    inclusive_scan, stream_, mem_pool_, valid_points_mask.get(), filtered_point_indices.get(),
    num_points, stream_);

  int num_filtered_points = 0;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_filtered_points,
    filtered_point_indices.get() +
      (num_points - 1),  // the end of array contains the number of filtered points
    sizeof(int), cudaMemcpyDeviceToHost, stream_));

  dim3 block_dim(512);
  dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);
  // Subtract 1 from all elements to make 0 started index
  minus_one_kernel<<<grid_dim, block_dim, 0, stream_>>>(filtered_point_indices.get(), num_points);

  // Making sure memcpy device to host operation completed
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return std::make_tuple(std::move(filtered_point_indices), num_filtered_points);
}

template <typename T, typename U>
void CudaPolarVoxelOutlierFilter::reduce_and_copy_to_host(
  const ReductionType reduction_type, const T & dev_array, const size_t & array_length,
  U * result_dev, U & result_host)
{
  auto reduction_op = [reduction_type](auto &&... args) {
    switch (reduction_type) {
      case ReductionType::Min:
        return cub::DeviceReduce::Min(std::forward<decltype(args)>(args)...);
      case ReductionType::Max:
        return cub::DeviceReduce::Max(std::forward<decltype(args)>(args)...);
      case ReductionType::Sum:
        return cub::DeviceReduce::Sum(std::forward<decltype(args)>(args)...);
      default:
        throw std::runtime_error("Invalid reduction type was specified");
    }
  };

  // Execute reduction
  cub_executor_.run_with_temp_storage(
    reduction_op, stream_, mem_pool_, dev_array, result_dev, array_length, stream_);

  // Copy result asynchronously.
  // To make synchronization timing under the control of the caller, this function does not call
  // synchronization operation such as cudaStreamSynchronize
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(&result_host, result_dev, sizeof(U), cudaMemcpyDeviceToHost, stream_));
}

size_t CudaPolarVoxelOutlierFilter::create_output(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
  const CudaPooledUniquePtr<bool> & points_mask, const size_t & num_points,
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_cloud)
{
  auto [filtered_indices, count] = calculate_filtered_point_indices(points_mask, num_points);

  output_cloud->header = input_cloud->header;
  output_cloud->fields = input_cloud->fields;
  output_cloud->is_bigendian = input_cloud->is_bigendian;
  output_cloud->point_step = input_cloud->point_step;
  output_cloud->is_dense = input_cloud->is_dense;
  output_cloud->height = point_cloud_height_organized;
  output_cloud->width = count;
  output_cloud->row_step = output_cloud->width * output_cloud->point_step;
  output_cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(output_cloud->row_step);

  dim3 block_dim(512);
  dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);

  copy_valid_points_kernel<<<grid_dim, block_dim, 0, stream_>>>(
    input_cloud->data.get(), points_mask.get(), filtered_indices.get(), num_points,
    output_cloud->point_step, output_cloud->data.get());

  return count;
}

}  // namespace autoware::cuda_pointcloud_preprocessor
