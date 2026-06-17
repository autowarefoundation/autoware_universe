//
// Created by jason on 1/5/22.
//

#ifndef MPPIGENERIC_TWO_D_TEXTURE_HELPER_CUH
#define MPPIGENERIC_TWO_D_TEXTURE_HELPER_CUH

#include <mppi/utils/texture_helpers/texture_helper.cuh>

template <class DATA_T>
class TwoDTextureHelper : public TextureHelper<TwoDTextureHelper<DATA_T>, DATA_T>
{
public:
  TwoDTextureHelper<DATA_T>(int number, cudaStream_t stream = 0)
    : TextureHelper<TwoDTextureHelper<DATA_T>, DATA_T>(number, stream)
  {
  }

  void allocateCudaTexture(int index) override;

  void updateTexture(int index,
                     const Eigen::Ref<const Eigen::Matrix<DATA_T, Eigen::Dynamic, Eigen::Dynamic>, 0,
                                      Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>,
                     cudaExtent& extent, bool column_major = true);
  void updateTexture(const int index,
                     const Eigen::Ref<const Eigen::Matrix<DATA_T, Eigen::Dynamic, Eigen::Dynamic>, 0,
                                      Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>,
                     bool column_major = true);
  void updateTexture(const int index, std::vector<DATA_T>& data, bool column_major = false);
  void updateTexture(const int index, std::vector<DATA_T>& data, cudaExtent& extent, bool column_major = false);
  bool setExtent(int index, cudaExtent& extent) override;
  void copyDataToGPU(int index, bool sync = false);

  __host__ __device__ DATA_T queryTexture(const int index, const float3& point);
  DATA_T queryTextureCPU(const int index, const float3& point);

protected:
};

#if __CUDACC__
#include "two_d_texture_helper.cu"
#endif

#endif  // MPPIGENERIC_TWO_D_TEXTURE_HELPER_CUH
