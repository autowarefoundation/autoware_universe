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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_voxel_grid_downsample_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/thrust_custom_allocator.hpp"

#include <cub/device/device_radix_sort.cuh>
#include <cub/device/device_run_length_encode.cuh>

#include <sensor_msgs/msg/point_field.hpp>

#include <thrust/adjacent_difference.h>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/extrema.h>
#include <thrust/scan.h>
#include <thrust/unique.h>

#include <cmath>
#include <memory>
#include <optional>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
template <typename T>
__device__ T * getElementPointer(
  uint8_t * data, const size_t point_index, const size_t point_step, const size_t offset)
{
  return reinterpret_cast<T *>(data + point_index * point_step + offset);
}

template <typename T>
__device__ const T getElementValue(
  const uint8_t * data, const size_t point_index, const size_t point_step, const size_t offset)
{
  return *reinterpret_cast<const T *>(data + point_index * point_step + offset);
}

__global__ void extractCoordKernel(
  const uint8_t * __restrict__ data, const size_t num_points, const size_t point_step,
  const size_t offset, float * __restrict__ coord_buf)
{
  size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= num_points) {
    return;
  }

  coord_buf[index] = getElementValue<float>(data, index, point_step, offset);
}

__constant__ CudaVoxelGridDownsampleFilter::VoxelInfo voxel_info_dev;

__global__ void calculateVoxelIndexKernel(
  const uint8_t * __restrict__ data,
  const CudaVoxelGridDownsampleFilter::ThreeDim<size_t> num_voxels,
  size_t * __restrict__ voxel_indices, size_t * __restrict__ point_indices,
  uint8_t * __restrict__ return_type_field, uint16_t * __restrict__ channel_field)
{
  size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= voxel_info_dev.num_input_points) {
    return;
  }

  auto x = getElementValue<float>(
    data, index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[0]);
  auto y = getElementValue<float>(
    data, index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[1]);
  auto z = getElementValue<float>(
    data, index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[2]);

  auto voxel_x =
    static_cast<int>(floorf(x / voxel_info_dev.voxel_size.x) - voxel_info_dev.min_coord.x);
  auto voxel_y =
    static_cast<int>(floorf(y / voxel_info_dev.voxel_size.y) - voxel_info_dev.min_coord.y);
  auto voxel_z =
    static_cast<int>(floorf(z / voxel_info_dev.voxel_size.z) - voxel_info_dev.min_coord.z);

  // Here defines voxel 1D index in x major
  voxel_indices[index] = static_cast<size_t>(
    voxel_x + (num_voxels.x) * voxel_y + (num_voxels.x * num_voxels.y) * voxel_z);
  point_indices[index] = index;

  // If `return_type` and `channel` fields are contained in the input,
  // gather the representative value to be copied to the output
  if (index == 0) {
    if (voxel_info_dev.input_return_type_offset.is_valid) {
      *return_type_field = getElementValue<uint8_t>(
        data, index, voxel_info_dev.input_point_step,
        voxel_info_dev.input_return_type_offset.offset);
    }
    if (voxel_info_dev.input_channel_offset.is_valid) {
      *channel_field = getElementValue<uint16_t>(
        data, index, voxel_info_dev.input_point_step, voxel_info_dev.input_channel_offset.offset);
    }
  }
}

template <typename IntensityType>
__global__ void accumulatePointsKernel(
  const uint8_t * __restrict__ input_data, const size_t * __restrict__ index_map,
  const size_t * __restrict__ point_indices,
  CudaVoxelGridDownsampleFilter::Centroid * __restrict__ centroids)
{
  size_t thread_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (thread_index >= voxel_info_dev.num_input_points) {
    return;
  }

  auto point_index = point_indices[thread_index];
  // Due to the nature of the adjacent_difference's behavior,
  // this subtraction will be required to get 0 started index
  auto voxel_index = index_map[thread_index] - index_map[0];

  auto x = getElementValue<float>(
    input_data, point_index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[0]);
  auto y = getElementValue<float>(
    input_data, point_index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[1]);
  auto z = getElementValue<float>(
    input_data, point_index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[2]);
  auto i = getElementValue<IntensityType>(
    input_data, point_index, voxel_info_dev.input_point_step, voxel_info_dev.input_xyzi_offset[3]);

  atomicAdd(&(centroids[voxel_index].x), x);
  atomicAdd(&(centroids[voxel_index].y), y);
  atomicAdd(&(centroids[voxel_index].z), z);
  atomicAdd(&(centroids[voxel_index].i), static_cast<float>(i));
  atomicAdd(&(centroids[voxel_index].count), static_cast<unsigned int>(1));
}

__global__ void packCentroidKernel(
  const CudaVoxelGridDownsampleFilter::Centroid * __restrict__ centroids,
  const size_t num_valid_voxel, const size_t output_point_step, uint8_t * __restrict__ output,
  uint8_t * __restrict__ return_type_field, uint16_t * __restrict__ channel_field)
{
  size_t index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= num_valid_voxel) {
    return;
  }

  auto dst = getElementPointer<OutputPointType>(output, index, output_point_step, 0);

  dst->x = static_cast<decltype(OutputPointType::x)>(centroids[index].x / centroids[index].count);
  dst->y = static_cast<decltype(OutputPointType::y)>(centroids[index].y / centroids[index].count);
  dst->z = static_cast<decltype(OutputPointType::z)>(centroids[index].z / centroids[index].count);
  dst->intensity =
    static_cast<decltype(OutputPointType::intensity)>(centroids[index].i / centroids[index].count);
  // Since `return_type` and `channel` fields cannot be accumulated, fill the representative values
  // for `return_type` and `channel` fields if they are available in the input pointcloud.
  dst->return_type =
    voxel_info_dev.input_return_type_offset.is_valid ? *return_type_field : 0;  // 0: UNKNOWN
  dst->channel =
    voxel_info_dev.input_channel_offset.is_valid ? *channel_field : 0;  // dummy laser channel ID
}
}  // namespace

CudaVoxelGridDownsampleFilter::CudaVoxelGridDownsampleFilter(
  const float voxel_size_x, const float voxel_size_y, const float voxel_size_z,
  const int64_t max_mem_pool_size_in_byte)
{
  voxel_info_.voxel_size.x = voxel_size_x;
  voxel_info_.voxel_size.y = voxel_size_y;
  voxel_info_.voxel_size.z = voxel_size_z;

  voxel_info_.output_offsets[0] = 0;
  voxel_info_.output_offsets[1] = voxel_info_.output_offsets[0] + sizeof(OutputPointType::x);
  voxel_info_.output_offsets[2] = voxel_info_.output_offsets[1] + sizeof(OutputPointType::y);
  voxel_info_.output_offsets[3] = voxel_info_.output_offsets[2] + sizeof(OutputPointType::z);
  voxel_info_.output_offsets[4] =
    voxel_info_.output_offsets[3] + sizeof(OutputPointType::intensity);
  voxel_info_.output_offsets[5] =
    voxel_info_.output_offsets[4] + sizeof(OutputPointType::return_type);

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // create memory pool to make repeated allocation efficient
  {
    int current_device_id = 0;
    CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;
    CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

    // Configure the memory pool reusing allocation
    // we set a high release threshold so that the allocated memory region will be reused
    uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
    CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
      mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
  }

  thrust_custom_allocator_ = std::make_unique<ThrustCustomAllocator>(stream_, mem_pool_);
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaVoxelGridDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  voxel_info_.num_input_points = input_points->width * input_points->height;
  voxel_info_.input_point_step = input_points->point_step;

  auto get_offset = [&](const std::string & field_name) -> size_t {
    int index = -1;
    for (size_t i = 0; i < input_points->fields.size(); ++i) {
      if (input_points->fields[i].name == field_name) {
        index = static_cast<int>(i);
      }
    }
    if (index < 0) {
      std::stringstream ss;
      ss << "input cloud does not contain filed named '" << field_name << "'";
      throw std::runtime_error(ss.str());
    }
    return input_points->fields[index].offset;
  };

  voxel_info_.input_xyzi_offset[0] = get_offset("x");
  voxel_info_.input_xyzi_offset[1] = get_offset("y");
  voxel_info_.input_xyzi_offset[2] = get_offset("z");
  voxel_info_.input_xyzi_offset[3] = get_offset("intensity");
  try {
    voxel_info_.input_return_type_offset.offset = get_offset("return_type");
    voxel_info_.input_return_type_offset.is_valid = true;
  } catch (const std::runtime_error & e) {
    voxel_info_.input_return_type_offset.is_valid = false;
  }

  try {
    voxel_info_.input_channel_offset.offset = get_offset("channel");
    voxel_info_.input_channel_offset.is_valid = true;
  } catch (const std::runtime_error & e) {
    voxel_info_.input_channel_offset.is_valid = false;
  }

  // Pull working buffer from the pooled region
  auto coord_buffer_dev = allocateBufferFromPool<float>(voxel_info_.num_input_points);
  auto voxel_index_buffer_dev = allocateBufferFromPool<size_t>(voxel_info_.num_input_points);
  auto point_index_buffer_dev = allocateBufferFromPool<size_t>(voxel_info_.num_input_points);
  auto index_map_dev = allocateBufferFromPool<size_t>(voxel_info_.num_input_points);
  auto return_type_field_dev = allocateBufferFromPool<decltype(OutputPointType::return_type)>(1);
  auto channel_field_dev = allocateBufferFromPool<decltype(OutputPointType::channel)>(1);

  // Get min/max value for x, y, z in input_points to determine computation target range
  getVoxelMinMaxCoordinate(input_points, coord_buffer_dev);

  // Calculate unique voxel indices that all points in input_points belong to
  // Create lookup table to show correspondence between spatial voxel grid position
  // and memory index
  auto num_valid_voxel = searchValidVoxel(
    input_points, voxel_index_buffer_dev, point_index_buffer_dev, return_type_field_dev,
    channel_field_dev);

  // Allocate region actually result filtered pointcloud requires and fill them
  auto filtered_output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  filtered_output->data =
    cuda_blackboard::make_unique<std::uint8_t[]>(num_valid_voxel * sizeof(OutputPointType));

  // Calculate actual voxel-filtered points
  auto centroid_buffer_dev = allocateBufferFromPool<Centroid>(num_valid_voxel * sizeof(Centroid));
  getCentroid(
    input_points, num_valid_voxel, voxel_index_buffer_dev, point_index_buffer_dev, index_map_dev,
    centroid_buffer_dev, filtered_output, return_type_field_dev, channel_field_dev);

  // Fill topic fields
  {
    auto generate_point_field = [](
                                  const std::string name, const uint32_t offset,
                                  const uint8_t datatype,
                                  const uint32_t count) -> sensor_msgs::msg::PointField {
      sensor_msgs::msg::PointField field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = count;
      return field;
    };
    filtered_output->header = input_points->header;
    filtered_output->height = input_points->height;
    filtered_output->width = num_valid_voxel;
    filtered_output->fields.push_back(generate_point_field(
      "x", voxel_info_.output_offsets[0], sensor_msgs::msg::PointField::FLOAT32, 1));
    filtered_output->fields.push_back(generate_point_field(
      "y", voxel_info_.output_offsets[1], sensor_msgs::msg::PointField::FLOAT32, 1));
    filtered_output->fields.push_back(generate_point_field(
      "z", voxel_info_.output_offsets[2], sensor_msgs::msg::PointField::FLOAT32, 1));
    filtered_output->fields.push_back(generate_point_field(
      "intensity", voxel_info_.output_offsets[3], sensor_msgs::msg::PointField::UINT8, 1));
    filtered_output->fields.push_back(generate_point_field(
      "return_type", voxel_info_.output_offsets[4], sensor_msgs::msg::PointField::UINT8, 1));
    filtered_output->fields.push_back(generate_point_field(
      "channel", voxel_info_.output_offsets[5], sensor_msgs::msg::PointField::UINT16, 1));
    filtered_output->is_bigendian = input_points->is_bigendian;
    filtered_output->point_step = sizeof(OutputPointType);
    filtered_output->row_step = sizeof(OutputPointType) * num_valid_voxel;
    filtered_output->is_dense = input_points->is_dense;
  }

  // Return (but not actual releasing) working buffer to the pool
  returnBufferToPool(coord_buffer_dev);
  returnBufferToPool(voxel_index_buffer_dev);
  returnBufferToPool(point_index_buffer_dev);
  returnBufferToPool(index_map_dev);
  returnBufferToPool(centroid_buffer_dev);
  returnBufferToPool(return_type_field_dev);
  returnBufferToPool(channel_field_dev);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return filtered_output;
}

template <typename T>
T * CudaVoxelGridDownsampleFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));

  return buffer;
}

template <typename T>
void CudaVoxelGridDownsampleFilter::returnBufferToPool(T * buffer)
{
  // Return (but not actual) working buffer to the pool
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

void CudaVoxelGridDownsampleFilter::getVoxelMinMaxCoordinate(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & points, float * buffer_dev)
{
  // Define GPU kernel configuration
  dim3 block_dim(512);
  dim3 grid_dim((voxel_info_.num_input_points + block_dim.x - 1) / block_dim.x);

  auto find_min_max = [&](const size_t offset_index, float & min_field, float & max_field) -> void {
    // extract coordinate value
    extractCoordKernel<<<grid_dim, block_dim, 0, stream_>>>(
      points->data.get(), voxel_info_.num_input_points, voxel_info_.input_point_step,
      voxel_info_.input_xyzi_offset[offset_index], buffer_dev);

    // get min and max coordinate
    auto min_max = thrust::minmax_element(
      thrust::cuda::par_nosync(*thrust_custom_allocator_).on(stream_),
      thrust::device_ptr<float>(buffer_dev),
      thrust::device_ptr<float>(buffer_dev + voxel_info_.num_input_points));

    // Fill the result
    // Since referring pointer contents by `*min_max.first` and `*min_max.second`
    // causes implicit data copy from device to host, download data using stream explicitly
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &min_field, thrust::raw_pointer_cast(min_max.first.get()), sizeof(float),
      cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &max_field, thrust::raw_pointer_cast(min_max.second.get()), sizeof(float),
      cudaMemcpyDeviceToHost, stream_));

    return;
  };  // auto & find_min_max

  ThreeDim<float> min_coord{}, max_coord{};
  find_min_max(0, min_coord.x, max_coord.x);
  find_min_max(1, min_coord.y, max_coord.y);
  find_min_max(2, min_coord.z, max_coord.z);

  CHECK_CUDA_ERROR(
    cudaStreamSynchronize(stream_));  // Ensure the values on the host side ready to use

  voxel_info_.min_coord.x = static_cast<int>(std::floor(min_coord.x / voxel_info_.voxel_size.x));
  voxel_info_.max_coord.x = static_cast<int>(std::floor(max_coord.x / voxel_info_.voxel_size.x));
  voxel_info_.min_coord.y = static_cast<int>(std::floor(min_coord.y / voxel_info_.voxel_size.y));
  voxel_info_.max_coord.y = static_cast<int>(std::floor(max_coord.y / voxel_info_.voxel_size.y));
  voxel_info_.min_coord.z = static_cast<int>(std::floor(min_coord.z / voxel_info_.voxel_size.z));
  voxel_info_.max_coord.z = static_cast<int>(std::floor(max_coord.z / voxel_info_.voxel_size.z));
}

size_t CudaVoxelGridDownsampleFilter::searchValidVoxel(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & points, size_t * voxel_index_buffer_dev,
  size_t * point_index_buffer_dev, decltype(OutputPointType::return_type) * return_type_field_dev,
  decltype(OutputPointType::channel) * channel_field_dev)
{
  // Update computation parameters on GPU constant memory
  CHECK_CUDA_ERROR(cudaMemcpyToSymbolAsync(
    voxel_info_dev, &voxel_info_, sizeof(VoxelInfo), 0, cudaMemcpyDefault, stream_));

  // calculate the number of voxel in each axis
  ThreeDim<size_t> num_voxels = {
    static_cast<size_t>(voxel_info_.max_coord.x - voxel_info_.min_coord.x) + 1,
    static_cast<size_t>(voxel_info_.max_coord.y - voxel_info_.min_coord.y) + 1,
    static_cast<size_t>(voxel_info_.max_coord.z - voxel_info_.min_coord.z) + 1,
  };

  // Define GPU kernel configuration
  dim3 block_dim(512);
  dim3 grid_dim((voxel_info_.num_input_points + block_dim.x - 1) / block_dim.x);

  // calculate voxel index that each input point belong to
  calculateVoxelIndexKernel<<<grid_dim, block_dim, 0, stream_>>>(
    points->data.get(), num_voxels, voxel_index_buffer_dev, point_index_buffer_dev,
    return_type_field_dev, channel_field_dev);

  // get number of valid voxels
  auto tmp_key_in = allocateBufferFromPool<size_t>(voxel_info_.num_input_points);
  auto tmp_val_in = allocateBufferFromPool<size_t>(voxel_info_.num_input_points);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    tmp_key_in, voxel_index_buffer_dev, voxel_info_.num_input_points * sizeof(size_t),
    cudaMemcpyDeviceToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    tmp_val_in, point_index_buffer_dev, voxel_info_.num_input_points * sizeof(size_t),
    cudaMemcpyDeviceToDevice, stream_));

  //// Determine temporary device strage requirements for sort operation
  void * tmp_storage = nullptr;
  size_t tmp_storage_bytes = 0;
  ////// check how many bit is actually needed to express the number of voxels
  size_t most_significant_bit_index =
    std::floor(std::log2(num_voxels.x * num_voxels.y * num_voxels.z));
  cub::DeviceRadixSort::SortPairs(
    tmp_storage, tmp_storage_bytes, tmp_key_in, voxel_index_buffer_dev, tmp_val_in,
    point_index_buffer_dev, voxel_info_.num_input_points, 0, most_significant_bit_index, stream_);

  //// allocate temporary storage
  tmp_storage = allocateBufferFromPool<std::byte>(tmp_storage_bytes);

  // Run sorting operation
  cub::DeviceRadixSort::SortPairs(
    tmp_storage, tmp_storage_bytes, tmp_key_in, voxel_index_buffer_dev, tmp_val_in,
    point_index_buffer_dev, voxel_info_.num_input_points, 0, most_significant_bit_index, stream_);
  returnBufferToPool(tmp_storage);

  //// Determine temporary device strage requirements for run-length encoding operation
  auto num_unique_voxels_dev = allocateBufferFromPool<size_t>(1);
  cub::DeviceRunLengthEncode::Encode(
    nullptr, tmp_storage_bytes, voxel_index_buffer_dev, tmp_key_in,
    tmp_val_in,  // use these storages for tamporal use
    num_unique_voxels_dev, voxel_info_.num_input_points, stream_);

  //// allocate temporary storage
  tmp_storage = allocateBufferFromPool<std::byte>(tmp_storage_bytes);

  //// Run encoding (count the number of valid voxels)
  cub::DeviceRunLengthEncode::Encode(
    tmp_storage, tmp_storage_bytes, voxel_index_buffer_dev, tmp_key_in,
    tmp_val_in,  // use these storages for tamporal use
    num_unique_voxels_dev, voxel_info_.num_input_points, stream_);

  //// wait until num_unique_voxels available
  size_t num_unique_voxels = 0;
  CHECK_CUDA_ERROR(cudaMemcpy(
    &num_unique_voxels, num_unique_voxels_dev, sizeof(size_t),
    cudaMemcpyDeviceToHost));  // use default stream for implicit sync

  returnBufferToPool(tmp_key_in);
  returnBufferToPool(tmp_val_in);
  returnBufferToPool(tmp_storage);

  return num_unique_voxels;
}

void CudaVoxelGridDownsampleFilter::getCentroid(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
  const size_t num_valid_voxel, const size_t * voxel_index_dev, const size_t * point_index_dev,
  size_t * index_map_dev, Centroid * buffer_dev,
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_points,
  decltype(OutputPointType::return_type) * return_type_field_dev,
  decltype(OutputPointType::channel) * channel_field_dev)
{
  // create map between index on spatial voxel and index on memory
  thrust::fill(
    thrust::cuda::par_nosync(*thrust_custom_allocator_).on(stream_),
    thrust::device_ptr<size_t>(index_map_dev),
    thrust::device_ptr<size_t>(index_map_dev + voxel_info_.num_input_points), 0);

  //// index_map_dev[i] will be 1 if voxel_index_dev[i-1] != voxel_index_dev[i]
  //// voxel_index_dev is assumed to be sorted
  //// Since adjacent_difference will always copy voxel_index_dev[0] to index_map_dev[0]
  //// actual memory location index should be calculated by subtracting index_map_dev[0]
  thrust::adjacent_difference(
    thrust::cuda::par_nosync(*thrust_custom_allocator_).on(stream_),
    thrust::device_ptr<const size_t>(voxel_index_dev),
    thrust::device_ptr<const size_t>(voxel_index_dev + voxel_info_.num_input_points),
    thrust::device_ptr<size_t>(index_map_dev), ::cuda::std::not_equal_to<size_t>());

  //// take accumulated sum to get index on memory for each valid voxels
  thrust::inclusive_scan(
    thrust::cuda::par_nosync(*thrust_custom_allocator_).on(stream_),
    thrust::device_ptr<size_t>(index_map_dev),
    thrust::device_ptr<size_t>(index_map_dev + voxel_info_.num_input_points),
    thrust::device_ptr<size_t>(index_map_dev));

  // Define GPU kernel configuration
  dim3 block_dim(512);
  dim3 grid_dim_point((voxel_info_.num_input_points + block_dim.x - 1) / block_dim.x);

  // get data type for intensity field
  auto get_intensity_type = [](const auto & points) -> const uint8_t {
    std::optional<uint8_t> intensity_index = std::nullopt;
    for (size_t i = 0; i < points->fields.size(); i++) {
      // Here assumes input points surely has "intensity" field
      if (points->fields[i].name == "intensity") {
        intensity_index = i;
        break;
      }
    }
    if (!intensity_index) {
      throw std::runtime_error("intensity field is not found in input");
    }
    return points->fields[intensity_index.value()].datatype;
  };

  // calculate voxel index that each input point belong to
  switch (get_intensity_type(input_points)) {
    case sensor_msgs::msg::PointField::INT8:
      accumulatePointsKernel<int8_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::UINT8:
      accumulatePointsKernel<uint8_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::INT16:
      accumulatePointsKernel<int16_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::UINT16:
      accumulatePointsKernel<uint16_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::INT32:
      accumulatePointsKernel<int32_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::UINT32:
      accumulatePointsKernel<uint32_t><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::FLOAT32:
      accumulatePointsKernel<float><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    case sensor_msgs::msg::PointField::FLOAT64:
      accumulatePointsKernel<double><<<grid_dim_point, block_dim, 0, stream_>>>(
        input_points->data.get(), index_map_dev, point_index_dev, buffer_dev);
      break;
    default:
      throw std::runtime_error("unsupported intensity data type is detected.");
  }

  dim3 grid_dim_voxel((num_valid_voxel + block_dim.x - 1) / block_dim.x);
  packCentroidKernel<<<grid_dim_voxel, block_dim, 0, stream_>>>(
    buffer_dev, num_valid_voxel, sizeof(OutputPointType), output_points->data.get(),
    return_type_field_dev, channel_field_dev);
}
}  // namespace autoware::cuda_pointcloud_preprocessor
