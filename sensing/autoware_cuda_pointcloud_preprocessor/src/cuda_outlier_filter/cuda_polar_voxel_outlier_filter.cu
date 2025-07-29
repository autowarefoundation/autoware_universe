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

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cub/cub.cuh>
#include <cuda/functional>    // for cuda::proclaim_return_type
#include <cuda/std/optional>  // for cuda::std::optional
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/exceptions/exceptions.hpp>

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
    bool ret = false;
    if (!rhs) {
      ret = false;
    } else {  // rhs has value
      if (!lhs) {
        ret = true;
      } else {
        ret = lhs.value() != rhs.value();
      }
    }
    return ret;
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

template <typename T>
__device__ T
get_element_value(const uint8_t * data, const size_t index, const size_t step, const size_t offset)
{
  return *reinterpret_cast<const T *>(data + index * step + offset);
}

template <typename TFieldData>
__global__ void polar_to_polar_voxel_kernel(
  const uint8_t * __restrict__ data, const size_t num_points, const uint32_t step,
  const FieldDataComposer<size_t> offsets, const FieldDataComposer<double> resolutions,
  const double min_radius, const double max_radius,
  FieldDataComposer<::cuda::std::optional<int> *> outputs)
{
  size_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  // treat 3 field (radius, azimuth, elevation) in parallel using y dimension
  auto field_index = static_cast<FieldDataIndex>(blockIdx.y);

  TFieldData field_data =
    get_element_value<TFieldData>(data, point_index, step, offsets[field_index]);

  bool is_finite = isfinite(field_data);
  bool is_valid_radius = (field_index == FieldDataIndex::radius)
                           ? (min_radius <= field_data) && (field_data <= max_radius)
                           : true;

  auto output = outputs[field_index];
  if (!is_finite || !is_valid_radius) {
    // Assign invalid_index for points with invalid value and/or points outside radius range
    // output[point_index] = invalid_index;
    output[point_index] = ::cuda::std::nullopt;
    return;
  }

  output[point_index] = static_cast<int>(floor(field_data / resolutions[field_index]));
}

__global__ void calculate_voxel_index_kernel(
  const FieldDataComposer<::cuda::std::optional<int> *> field_indices, const size_t num_points,
  const FieldDataComposer<int> field_dimensions, ::cuda::std::optional<int> * point_indices,
  ::cuda::std::optional<int> * voxel_indices)
{
  size_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  auto radius_idx = field_indices[FieldDataIndex::radius][point_index];
  auto azimuth_idx = field_indices[FieldDataIndex::azimuth][point_index];
  auto elevation_idx = field_indices[FieldDataIndex::elevation][point_index];

  auto radius_dim = field_dimensions[FieldDataIndex::radius];
  auto azimuth_dim = field_dimensions[FieldDataIndex::azimuth];

  // Save point index and corresponding voxel index in the same index position of two arrays
  point_indices[point_index] = point_index;
  if (!radius_idx || !azimuth_idx || !elevation_idx) {
    // If any of field indices has invalid value, voxel index for this point will also be invalid
    voxel_indices[point_index] = ::cuda::std::nullopt;
    return;
  }

  voxel_indices[point_index] = elevation_idx.value() * (radius_dim * azimuth_dim) +
                               azimuth_idx.value() * radius_dim + radius_idx.value();
}

__global__ void subtract_left_optional_kernel(
  const ::cuda::std::optional<int> * __restrict__ input_array, const size_t array_length,
  bool * __restrict__ output_array)
{
  size_t array_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (array_index >= array_length) {
    return;
  }

  // Specially handle the very first element
  if (array_index == 0) {
    output_array[array_index] == input_array[array_index].has_value();
    return;
  }

  auto difference_op = ValidAndNotEqualTo();
  output_array[array_index] = difference_op(input_array[array_index - 1], input_array[array_index]);
}

__global__ void minus_one_kernel(int * __restrict__ indices, const size_t num_points)
{
  size_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  indices[point_index] -= 1;
}

template <typename TReturnType>
__global__ void classify_point_by_return_type_kernel(
  const uint8_t * __restrict__ data, const size_t num_points, const int num_voxels,
  const size_t step, const size_t offset, const ReturnTypeCandidates primary_return_type,
  const ReturnTypeCandidates secondary_return_type,
  const ::cuda::std::optional<int> * __restrict__ point_indices,
  const int * __restrict__ voxel_indices, size_t * __restrict__ primary_returns,
  size_t * __restrict__ secondary_returns, bool * __restrict__ is_primary_returns,
  bool * __restrict__ is_secondary_returns)
{
  size_t array_index = blockIdx.x * blockDim.x + threadIdx.x;
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

  auto return_type = get_element_value<TReturnType>(data, point_index.value(), step, offset);

  auto find = [] __device__(ReturnTypeCandidates candidates, TReturnType value) -> bool {
    bool is_found = false;
    for (size_t i = 0; i < candidates.num_candidates; i++) {
      is_found = is_found || (candidates.return_types[i] == value);
    }
    return is_found;
  };

  auto is_primary_return_type = find(primary_return_type, return_type);
  is_primary_returns[point_index.value()] = is_primary_return_type;
  auto is_secondary_return_type = find(secondary_return_type, return_type);
  is_secondary_returns[point_index.value()] = is_secondary_return_type;

  if (is_primary_return_type) {
    atomic_add_size_t(&(primary_returns[voxel_index]), 1);
  } else if (is_secondary_return_type) {
    atomic_add_size_t(&(secondary_returns[voxel_index]), 1);
  }
}

__global__ void criterion_check_kernel(
  const size_t * __restrict__ primary_returns, const size_t * __restrict__ secondary_returns,
  const size_t num_total_voxels, const int voxel_points_threshold,
  const int secondary_noise_threshold, bool * __restrict__ primary_meets_threshold,
  bool * __restrict__ secondary_meets_threshold)
{
  size_t voxel_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (voxel_index >= num_total_voxels) {
    return;
  }

  // Check criterion 1: Primary returns meet the threshold
  primary_meets_threshold[voxel_index] = primary_returns[voxel_index] >= voxel_points_threshold;

  // Check criterion 2: Number of secondary returns is less than the threshold
  secondary_meets_threshold[voxel_index] =
    secondary_returns[voxel_index] <= secondary_noise_threshold;
}

__global__ void point_validity_check_kernel(
  const bool * __restrict__ primary_meets_threshold,
  const bool * __restrict__ secondary_meets_threshold,
  const ::cuda::std::optional<int> * __restrict__ point_indices,
  const int * __restrict__ voxel_indices, const bool * __restrict__ is_primary_returns,
  const bool * __restrict__ is_secondary_returns, const size_t num_points, const int num_voxels,
  const bool filter_secondary_returns, bool * __restrict__ valid_points)
{
  size_t array_index = blockIdx.x * blockDim.x + threadIdx.x;
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
    valid_points[point_index.value()] = false;
  }

  // Voxel is kept if BOTH criteria are met
  auto meet_criteria =
    primary_meets_threshold[voxel_index] && secondary_meets_threshold[voxel_index];

  // Add primary returns
  valid_points[point_index.value()] = (meet_criteria) && (is_primary_returns[point_index.value()]);

  // Add secondary returns only if not filtering them
  if (!filter_secondary_returns && is_secondary_returns[point_index.value()]) {
    valid_points[point_index.value()] = meet_criteria;
  }
}

__global__ void copy_valid_points_kernel(
  const uint8_t * __restrict__ input_cloud, const bool * __restrict__ valid_points,
  const int * __restrict__ filtered_indices, const size_t num_points, const size_t step,
  uint8_t * __restrict__ output_points)
{
  size_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  bool is_valid = valid_points[point_index];
  int destination_index = filtered_indices[point_index];

  if (is_valid) {
    memcpy(output_points + destination_index * step, input_cloud + point_index * step, step);
  }
}

__global__ void bool_flip_kernel(bool * __restrict__ flags, const size_t num_points)
{
  size_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= num_points) {
    return;
  }

  flags[point_index] = !flags[point_index];
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

CudaPolarVoxelOutlierFilter::CudaPolarVoxelOutlierFilter()
: primary_return_type_dev_(std::nullopt), secondary_return_type_dev_(std::nullopt)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // create memory pool to make repeated allocation efficient
  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  size_t max_mem_pool_size_in_byte = 1e9;  // 1GB
  mem_pool_ =
    autoware::cuda_utils::create_memory_pool(max_mem_pool_size_in_byte, current_device_id);
}

CudaPolarVoxelOutlierFilter::FilterReturn CudaPolarVoxelOutlierFilter::filter_point_xyzircaedt(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_cloud,
  const CudaPolarVoxelOutlierFilterParameters & params)
{
  if (!primary_return_type_dev_ || !secondary_return_type_dev_) {
    return FilterReturn{nullptr, nullptr, 0., 0.};
  }

  size_t num_points = input_cloud->width * input_cloud->height;

  FieldDataComposer<size_t> offsets{};
  offsets.radius = get_offset(input_cloud->fields, "distance");
  offsets.azimuth = get_offset(input_cloud->fields, "azimuth");
  offsets.elevation = get_offset(input_cloud->fields, "elevation");

  auto [radius_idx, azimuth_idx, elevation_idx, polar_voxel_indices] =
    generate_field_data_composer<::cuda::std::optional<int>>(num_points, stream_, mem_pool_);

  FieldDataComposer<double> resolutions{};
  resolutions.radius = params.radius_resolution;
  resolutions.azimuth = params.azimuth_resolution;
  resolutions.elevation = params.elevation_resolution;

  int num_total_voxels = 0;
  CudaPooledUniquePtr<::cuda::std::optional<int>> point_indices = nullptr;
  CudaPooledUniquePtr<int> voxel_indices = nullptr;
  // the number of points that have (primary|secondary) returns per voxels
  CudaPooledUniquePtr<size_t> num_primary_returns = nullptr;
  CudaPooledUniquePtr<size_t> num_secondary_returns = nullptr;
  // flags whether each points has (primary|secondary) return type (for later use)
  CudaPooledUniquePtr<bool> is_primary_returns = nullptr;
  CudaPooledUniquePtr<bool> is_secondary_returns = nullptr;

  // First pass: count points in each polar voxel using pre-computed coordinates
  {
    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x, 3, 1);

    polar_to_polar_voxel_kernel<float><<<grid_dim, block_dim, 0, stream_>>>(
      input_cloud->data.get(), num_points, input_cloud->point_step, offsets, resolutions,
      params.min_radius, params.max_radius, polar_voxel_indices);

    // calculate unique voxel index that each point belongs to
    std::tie(num_total_voxels, point_indices, voxel_indices) =
      calculate_voxel_index(polar_voxel_indices, num_points);

    //  count the number of primary/secondary returns of each voxel
    num_primary_returns =
      autoware::cuda_utils::make_unique<size_t>(num_total_voxels, stream_, mem_pool_);
    num_secondary_returns =
      autoware::cuda_utils::make_unique<size_t>(num_total_voxels, stream_, mem_pool_);
    is_primary_returns = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);
    is_secondary_returns = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);

    // Add points to appropriate vector based on return type
    size_t return_type_offset = get_offset(input_cloud->fields, "return_type");
    grid_dim = dim3((num_points + block_dim.x - 1) / block_dim.x);
    classify_point_by_return_type_kernel<uint8_t><<<grid_dim, block_dim, 0, stream_>>>(
      input_cloud->data.get(), num_points, num_total_voxels, input_cloud->point_step,
      return_type_offset, primary_return_type_dev_.value(), secondary_return_type_dev_.value(),
      point_indices.get(), voxel_indices.get(), num_primary_returns.get(),
      num_secondary_returns.get(), is_primary_returns.get(), is_secondary_returns.get());
  }

  // Collect valid point indices and visibility statistics
  auto valid_points = autoware::cuda_utils::make_unique<bool>(num_points, stream_, mem_pool_);
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
      voxel_indices.get(), is_primary_returns.get(), is_secondary_returns.get(), num_points,
      num_total_voxels, params.filter_secondary_returns, valid_points.get());
  }

  // Create filtered output
  size_t valid_count = 0;
  CudaPooledUniquePtr<int> filtered_indices = nullptr;
  auto filtered_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  {
    std::tie(filtered_indices, valid_count) =
      calculate_filtered_point_indices(valid_points, num_points);

    filtered_cloud->header = input_cloud->header;
    filtered_cloud->fields = input_cloud->fields;
    filtered_cloud->is_bigendian = input_cloud->is_bigendian;
    filtered_cloud->point_step = input_cloud->point_step;
    filtered_cloud->is_dense = input_cloud->is_dense;
    filtered_cloud->height = 1;
    filtered_cloud->width = valid_count;
    filtered_cloud->row_step = filtered_cloud->width * filtered_cloud->point_step;
    filtered_cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(filtered_cloud->row_step);

    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);

    copy_valid_points_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      input_cloud->data.get(), valid_points.get(), filtered_indices.get(), num_points,
      filtered_cloud->point_step, filtered_cloud->data.get());
  }

  // Create noise cloud with filtered-out points
  size_t invalid_count = 0;
  CudaPooledUniquePtr<int> noise_indices = nullptr;
  auto noise_cloud = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  {
    // Flip valid flag to get filtered-out (= noise) points
    dim3 block_dim(512);
    dim3 grid_dim((num_points + block_dim.x - 1) / block_dim.x);

    bool_flip_kernel<<<grid_dim, block_dim, 0, stream_>>>(valid_points.get(), num_points);

    std::tie(noise_indices, invalid_count) =
      calculate_filtered_point_indices(valid_points, num_points);

    noise_cloud->header = input_cloud->header;
    noise_cloud->fields = input_cloud->fields;
    noise_cloud->is_bigendian = input_cloud->is_bigendian;
    noise_cloud->point_step = input_cloud->point_step;
    noise_cloud->is_dense = input_cloud->is_dense;
    noise_cloud->height = 1;
    noise_cloud->width = invalid_count;
    noise_cloud->row_step = noise_cloud->width * noise_cloud->point_step;
    noise_cloud->data = cuda_blackboard::make_unique<std::uint8_t[]>(noise_cloud->row_step);

    copy_valid_points_kernel<<<grid_dim, block_dim, 0, stream_>>>(
      input_cloud->data.get(), valid_points.get(), noise_indices.get(), num_points,
      noise_cloud->point_step, noise_cloud->data.get());
  }

  // Calculate filter ratio and visibility (only when return type classification is enabled)
  auto filter_ratio =
    num_points > 0 ? static_cast<double>(valid_count) / static_cast<double>(num_points) : 0.0;

  double visibility = 0.0;
  if (params.use_return_type_classification) {
    int total_voxels_with_points = num_total_voxels;

    // calculate the number of voxels that secondary test passed
    int voxels_passed_secondary_test = 0;
    auto reduction_result_tmp_dev = autoware::cuda_utils::make_unique<int>(stream_, mem_pool_);
    reduce_and_copy_to_host(
      ReductionType::Sum, secondary_meets_threshold.get(), num_total_voxels,
      reduction_result_tmp_dev.get(), voxels_passed_secondary_test);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));  // wait till device to host copy finish

    visibility = total_voxels_with_points > 0 ? static_cast<double>(voxels_passed_secondary_test) /
                                                  static_cast<double>(total_voxels_with_points)
                                              : 0.0;
  }

  return {std::move(filtered_cloud), std::move(noise_cloud), filter_ratio, visibility};
}

void CudaPolarVoxelOutlierFilter::set_return_types(
  const std::vector<int64_t> & types, std::optional<ReturnTypeCandidates> & types_dev)
{
  if (types_dev) {
    // Reset allocated region previously to reflesh the parameters
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

// std::tuple<FieldDataComposer<int>, CudaPooledUniquePtr<::cuda::std::optional<int>>>
std::tuple<int, CudaPooledUniquePtr<::cuda::std::optional<int>>, CudaPooledUniquePtr<int>>
CudaPolarVoxelOutlierFilter::calculate_voxel_index(
  const FieldDataComposer<::cuda::std::optional<int> *> & polar_voxel_indices,
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
    cub::TransformInputIterator<int, NulloptToMax, ::cuda::std::optional<int> *>
      transformed_in_null_to_max(polar_voxel_index, NulloptToMax{});

    // Take Minimum value
    reduce_and_copy_to_host(
      ReductionType::Min, transformed_in_null_to_max, num_points, reduction_result_tmp_dev.get(),
      min_val_host);

    // Though comparison operators are defined for cuda::std::optional,
    // cuda::std::optional does not have ::Lowest() member, which is required for
    // cub::DeviceReduce::Max. Here, cuda::std::optional is wrapped to transform into its contained
    // value (if nullopt, then return numeric_limits::lowest) to make cub::DeviceReduce::Max work
    cub::TransformInputIterator<int, NulloptToLowest, ::cuda::std::optional<int> *>
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
    polar_voxel_indices, num_points, polar_voxel_range, point_indices.get(),
    voxel_indices_raw.get());

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

    // calculate mapped index (from geometry-based liner voxel indices to memory-based indices)
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

    // Since the current polar_voxel_index_seq contains index value starting from 1,
    //  Subtract 1 from all elements to make 0 started index
    //// NOTE: equivalent operation can be achieved cud::DeviceFor::Forereach that is introduced
    /// from / cub v2.4.0
    minus_one_kernel<<<grid_dim, block_dim, 0, stream_>>>(voxel_indices.get(), num_points);
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));  // Make sure Device to Host copy complete

  return std::make_tuple(valid_voxel_num, std::move(point_indices), std::move(voxel_indices));
}

std::tuple<CudaPooledUniquePtr<int>, size_t>
CudaPolarVoxelOutlierFilter::calculate_filtered_point_indices(
  const CudaPooledUniquePtr<bool> & valid_points, const size_t & num_points)
{
  // Scan valid_points to calculate the total number of filtered points and map from the source
  // point index to filtered point index
  auto filtered_point_indices =
    autoware::cuda_utils::make_unique<int>(num_points, stream_, mem_pool_);

  auto inclusive_scan = [](auto &&... args) {
    return cub::DeviceScan::InclusiveSum(std::forward<decltype(args)>(args)...);
  };
  cub_executor_.run_with_temp_storage(
    inclusive_scan, stream_, mem_pool_, valid_points.get(), filtered_point_indices.get(),
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

}  // namespace autoware::cuda_pointcloud_preprocessor
