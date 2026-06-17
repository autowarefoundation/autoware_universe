//
// Created by jason on 1/5/22.
//

#include "two_d_texture_helper.cuh"

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::allocateCudaTexture(int index)
{
  TextureHelper<TwoDTextureHelper<DATA_T>, DATA_T>::allocateCudaTexture(index);

  TextureParams<DATA_T>* param = &this->textures_[index];

  int w = param->extent.width;
  int h = param->extent.height;

  HANDLE_ERROR(cudaMallocArray(&(param->array_d), &(param->channelDesc), w, h));
}

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::updateTexture(const int index, std::vector<DATA_T>& values, bool column_major)
{
  TextureParams<DATA_T>* param = &this->textures_buffer_[index];
  int w = param->extent.width;
  int h = param->extent.height;

  // check that the sizes are correct
  if (values.size() != w * h)
  {
    throw std::runtime_error(std::string("Error: invalid size to updateTexture ") + std::to_string(values.size()) +
                             " != " + std::to_string(w * h));
  }

  // copy over values to cpu side holder
  this->cpu_buffer_values_[index].resize(w * h);
  if (column_major)
  {
    for (int j = 0; j < w; j++)
    {
      for (int i = 0; i < h; i++)
      {
        int columnMajorIndex = j * h + i;
        int rowMajorIndex = i * w + j;
        this->cpu_buffer_values_[index][rowMajorIndex] = values[columnMajorIndex];
      }
    }
  }
  else
  {
    // std::copy(values.begin(), values.end(), cpu_buffer_values_[index].begin());
    this->cpu_buffer_values_[index] = std::move(values);
  }
  // tells the object to copy it over next time that happens
  param->update_data = true;
}

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::updateTexture(const int index, std::vector<DATA_T>& data, cudaExtent& extent,
                                              bool column_major)
{
  setExtent(index, extent);
  updateTexture(index, data, column_major);
}

template <class DATA_T>
__host__ __device__ DATA_T TwoDTextureHelper<DATA_T>::queryTexture(const int index, const float3& point)
{
#ifdef __CUDA_ARCH__
  return tex2D<DATA_T>(this->textures_d_[index].tex_d, point.x, point.y);
#else
  return queryTextureCPU(index, point);
#endif
}

template <class DATA_T>
bool TwoDTextureHelper<DATA_T>::setExtent(int index, cudaExtent& extent)
{
  if (extent.depth != 0)
  {
    throw std::runtime_error(std::string("Error: extent in setExtent invalid,"
                                         " cannot use depth != 0 in 2D texture: using ") +
                             std::to_string(extent.depth));
  }

  return TextureHelper<TwoDTextureHelper<DATA_T>, DATA_T>::setExtent(index, extent);
}

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::copyDataToGPU(int index, bool sync)
{
  TextureParams<DATA_T>* param = &this->textures_[index];
  int w = param->extent.width;
  int h = param->extent.height;
  HANDLE_ERROR(cudaMemcpy2DToArrayAsync(param->array_d, 0, 0, this->cpu_values_[index].data(), w * sizeof(DATA_T),
                                        w * sizeof(DATA_T), h, cudaMemcpyHostToDevice, this->stream_));
  if (sync)
  {
    cudaStreamSynchronize(this->stream_);
  }
  param->update_data = false;
}

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::updateTexture(
    const int index,
    const Eigen::Ref<const Eigen::Matrix<DATA_T, Eigen::Dynamic, Eigen::Dynamic>, 0,
                     Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
        values,
    bool column_major)
{
  TextureParams<DATA_T>* param = &this->textures_buffer_[index];
  int w = param->extent.width;
  int h = param->extent.height;
  this->cpu_buffer_values_[index].resize(w * h);

  if (column_major)
  {
    // if we are column major transform to row major
    for (int j = 0; j < w; j++)
    {
      for (int i = 0; i < h; i++)
      {
        int columnMajorIndex = j * h + i;
        int rowMajorIndex = i * w + j;
        this->cpu_buffer_values_[index][rowMajorIndex] = values.data()[columnMajorIndex];
      }
    }
  }
  else
  {
    // if we row major copy directly
    memcpy(this->cpu_buffer_values_[index].data(), values.data(), values.size() * sizeof(DATA_T));
  }
  // tells the object to copy it over next time that happens
  param->update_data = true;
}

template <class DATA_T>
void TwoDTextureHelper<DATA_T>::updateTexture(
    int index,
    const Eigen::Ref<const Eigen::Matrix<DATA_T, Eigen::Dynamic, Eigen::Dynamic>, 0,
                     Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>
        values,
    cudaExtent& extent, bool column_major)
{
  setExtent(index, extent);
  updateTexture(index, values, column_major);
}

template <class DATA_T>
DATA_T TwoDTextureHelper<DATA_T>::queryTextureCPU(const int index, const float3& point)
{
  TextureParams<DATA_T>* param = &this->textures_[index];

  // convert normalized to array index
  float2 query = make_float2(point.x * param->extent.width, point.y * param->extent.height);

  // we subtract half a grid cell since the elevation map is the elevation at the center of the grid cell
  query.x = query.x - 0.5f;
  query.y = query.y - 0.5f;
  // if (this->cpu_values_[index].size() < param->extent.width * param->extent.height)
  // {
  //   return DATA_T();
  // }
  if (param->texDesc.addressMode[0] == cudaAddressModeClamp)
  {
    if (query.x > param->extent.width - 1)
    {
      query.x = param->extent.width - 1;
    }
    else if (query.x <= 0.0)
    {
      query.x = 0.0;
    }
  }
  else if (param->texDesc.addressMode[0] == cudaAddressModeBorder)
  {
    if (query.x > param->extent.width - 1 || query.x <= 0.0)
    {
      return createPartialCudaTuple<DATA_T>(param->texDesc.borderColor[0], param->texDesc.borderColor[1],
                                            param->texDesc.borderColor[2], param->texDesc.borderColor[3]);
    }
  }
  else
  {
    throw std::runtime_error(std::string("using unsupported address mode on the CPU in texture utils"));
  }
  if (param->texDesc.addressMode[1] == cudaAddressModeClamp)
  {
    if (query.y > param->extent.height - 1)
    {
      query.y = param->extent.height - 1;
    }
    else if (query.y <= 0.0)
    {
      query.y = 0.0;
    }
  }
  else if (param->texDesc.addressMode[1] == cudaAddressModeBorder)
  {
    if (query.y > param->extent.height - 1 || query.y <= 0.0)
    {
      return createPartialCudaTuple<DATA_T>(param->texDesc.borderColor[0], param->texDesc.borderColor[1],
                                            param->texDesc.borderColor[2], param->texDesc.borderColor[3]);
    }
  }
  else
  {
    throw std::runtime_error(std::string("using unsupported address mode on the CPU in texture utils"));
  }
  int w = param->extent.width;
  if (param->texDesc.filterMode == cudaFilterModeLinear)
  {
    // the value is distributed evenly in the space starting at half a cell from 0.0
    int x_min = std::min((int)std::floor(query.x), w - 2);
    int x_max = x_min + 1;
    int y_min = std::min((int)std::floor(query.y), (int)param->extent.height - 2);
    int y_max = y_min + 1;

    // does bilinear interpolation https://en.wikipedia.org/wiki/Bilinear_interpolation

    DATA_T Q_11 = this->cpu_values_[index][y_min * w + x_min];
    DATA_T Q_12 = this->cpu_values_[index][y_min * w + x_max];
    DATA_T Q_21 = this->cpu_values_[index][y_max * w + x_min];
    DATA_T Q_22 = this->cpu_values_[index][y_max * w + x_max];

    DATA_T y_min_interp = Q_11 * ((x_max - query.x) / (x_max - x_min)) + Q_12 * ((query.x - x_min) / (x_max - x_min));
    DATA_T y_max_interp = Q_21 * ((x_max - query.x) / (x_max - x_min)) + Q_22 * ((query.x - x_min) / (x_max - x_min));

    DATA_T result =
        y_min_interp * ((y_max - query.y) / (y_max - y_min)) + y_max_interp * ((query.y - y_min) / (y_max - y_min));

    // does the actual interpolation
    return result;
  }
  else if (param->texDesc.filterMode == cudaFilterModePoint)
  {
    int rowMajorIndex = std::round(query.y) * w + std::round(query.x);
    return this->cpu_values_[index][rowMajorIndex];
  }
  else
  {
    throw std::runtime_error(std::string("using unsupported filter mode on the CPU in texture utils"));
  }
}
