#include <gtest/gtest.h>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>

#include <mppi/utils/texture_helpers/texture_helper.cuh>

class TextureHelperTest : public testing::Test
{
protected:
  void SetUp() override
  {
    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(100.0, 2.0);
  }

  void TearDown() override
  {
  }

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
};

class TextureHelperImpl : public TextureHelper<TextureHelperImpl, float4>
{
public:
  TextureHelperImpl(int number, cudaStream_t stream = 0) : TextureHelper<TextureHelperImpl, float4>(number, stream)
  {
  }

  ~TextureHelperImpl()
  {
    for (TextureParams<float4>& params : textures_)
    {
      params.allocated = false;
    }
  }

  std::vector<TextureParams<float4>> getTextures()
  {
    return textures_;
  }

  std::vector<TextureParams<float4>> getTexturesBuffer()
  {
    return textures_buffer_;
  }

  TextureParams<float4>* getParamsD()
  {
    return this->params_d_;
  }

  void setTestExtent(int index, cudaExtent extent)
  {
    this->textures_buffer_[index].extent = extent;
    this->textures_[index].extent = extent;
  }

  void setGpuMemStatus(bool flag)
  {
    this->GPUMemStatus_ = flag;
  }

  void setTestFlagsInParam(int index, bool update_mem, bool update_data, bool allocated, bool update_params)
  {
    this->textures_buffer_[index].update_mem = update_mem;
    this->textures_buffer_[index].update_data = update_data;
    this->textures_[index].allocated = allocated;
    this->textures_buffer_[index].update_params = update_params;
  }

  void copyDataToGPU(int index, bool sync = false) override
  {
    copyDataToGPUCalled++;
    this->textures_[index].update_data = false;
  }

  void allocateCudaTexture(int index) override
  {
    if (this->textures_[index].allocated)
    {
      freeCudaMem(this->textures_[index]);
    }
    allocateCudaTextureCalled++;
  }

  void createCudaTexture(int index, bool sync = false) override
  {
    createCudaTextureCalled++;
    this->textures_[index].allocated = true;
    this->textures_[index].update_mem = false;
  }

  void copyParamsToGPU(int index, bool sync = false)
  {
    copyParamsToGPUCalled++;
    this->textures_[index].update_params = false;
  }

  void clearCounters()
  {
    copyDataToGPUCalled = 0;
    allocateCudaTextureCalled = 0;
    createCudaTextureCalled = 0;
    copyParamsToGPUCalled = 0;
    freeCudaMemCalled = 0;
  }

  void freeCudaMem(TextureParams<float4>& texture)
  {
    texture.allocated = false;
    texture.use = false;
    freeCudaMemCalled++;
  }

  TextureParams<float4>* getTextureD()
  {
    return this->textures_d_;
  }

  TextureParams<float4>* getTextureDataPtr()
  {
    return this->textures_.data();
  }

  int copyDataToGPUCalled = 0;
  int allocateCudaTextureCalled = 0;
  int createCudaTextureCalled = 0;
  int copyParamsToGPUCalled = 0;
  int freeCudaMemCalled = 0;

  float4 queryTexture(const int index, const float3& input)
  {
    float4 result;
    result.x = input.x;
    result.y = input.y;
    result.z = input.z;
    result.w = index;
    return result;
  }
};

TEST_F(TextureHelperTest, Constructor)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  std::vector<TextureParams<float4>> textures = helper.getTextures();

  EXPECT_EQ(textures.size(), number);
  EXPECT_EQ(helper.getCpuValues().size(), number);
  EXPECT_EQ(helper.getCpuBufferValues().size(), number);
  EXPECT_NE(helper.getTextureD(), nullptr);
  EXPECT_NE(helper.getTextureDataPtr(), nullptr);
  EXPECT_EQ(helper.ptr_d_, nullptr);
  EXPECT_EQ(helper.getParamsD(), nullptr);
  EXPECT_EQ(helper.getTextureD(), helper.getTextureDataPtr());

  for (const TextureParams<float4> texture : textures)
  {
    EXPECT_EQ(texture.use, false);
    EXPECT_EQ(texture.allocated, false);
    EXPECT_EQ(texture.update_data, false);
    EXPECT_EQ(texture.update_mem, false);

    EXPECT_EQ(texture.array_d, nullptr);
    EXPECT_EQ(texture.tex_d, 0);

    EXPECT_FLOAT_EQ(texture.origin.x, 0);
    EXPECT_FLOAT_EQ(texture.origin.y, 0);
    EXPECT_FLOAT_EQ(texture.origin.z, 0);

    EXPECT_FLOAT_EQ(texture.rotations[0].x, 1);
    EXPECT_FLOAT_EQ(texture.rotations[0].y, 0);
    EXPECT_FLOAT_EQ(texture.rotations[0].z, 0);
    EXPECT_FLOAT_EQ(texture.rotations[1].x, 0);
    EXPECT_FLOAT_EQ(texture.rotations[1].y, 1);
    EXPECT_FLOAT_EQ(texture.rotations[1].z, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].x, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].y, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].z, 1);

    EXPECT_EQ(texture.resDesc.resType, cudaResourceTypeArray);
    EXPECT_EQ(texture.channelDesc.x, 32);
    EXPECT_EQ(texture.channelDesc.y, 32);
    EXPECT_EQ(texture.channelDesc.z, 32);
    EXPECT_EQ(texture.channelDesc.w, 32);

    EXPECT_EQ(texture.texDesc.addressMode[0], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.addressMode[1], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.filterMode, cudaFilterModeLinear);
    EXPECT_EQ(texture.texDesc.readMode, cudaReadModeElementType);
    EXPECT_EQ(texture.texDesc.normalizedCoords, 1);
  }

  textures = helper.getTexturesBuffer();

  EXPECT_EQ(textures.size(), number);
  EXPECT_NE(helper.getTextureD(), nullptr);
  EXPECT_NE(helper.getTextureDataPtr(), nullptr);
  EXPECT_EQ(helper.ptr_d_, nullptr);
  EXPECT_EQ(helper.getParamsD(), nullptr);
  EXPECT_EQ(helper.getTextureD(), helper.getTextureDataPtr());

  for (const TextureParams<float4> texture : textures)
  {
    EXPECT_EQ(texture.use, false);
    EXPECT_EQ(texture.allocated, false);
    EXPECT_EQ(texture.update_data, false);
    EXPECT_EQ(texture.update_mem, false);

    EXPECT_EQ(texture.array_d, nullptr);
    EXPECT_EQ(texture.tex_d, 0);

    EXPECT_FLOAT_EQ(texture.origin.x, 0);
    EXPECT_FLOAT_EQ(texture.origin.y, 0);
    EXPECT_FLOAT_EQ(texture.origin.z, 0);

    EXPECT_FLOAT_EQ(texture.rotations[0].x, 1);
    EXPECT_FLOAT_EQ(texture.rotations[0].y, 0);
    EXPECT_FLOAT_EQ(texture.rotations[0].z, 0);
    EXPECT_FLOAT_EQ(texture.rotations[1].x, 0);
    EXPECT_FLOAT_EQ(texture.rotations[1].y, 1);
    EXPECT_FLOAT_EQ(texture.rotations[1].z, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].x, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].y, 0);
    EXPECT_FLOAT_EQ(texture.rotations[2].z, 1);

    EXPECT_EQ(texture.resDesc.resType, cudaResourceTypeArray);
    EXPECT_EQ(texture.channelDesc.x, 32);
    EXPECT_EQ(texture.channelDesc.y, 32);
    EXPECT_EQ(texture.channelDesc.z, 32);
    EXPECT_EQ(texture.channelDesc.w, 32);

    EXPECT_EQ(texture.texDesc.addressMode[0], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.addressMode[1], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.filterMode, cudaFilterModeLinear);
    EXPECT_EQ(texture.texDesc.readMode, cudaReadModeElementType);
    EXPECT_EQ(texture.texDesc.normalizedCoords, 1);
  }
}

TEST_F(TextureHelperTest, UpdateOriginTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);
  helper.updateOrigin(0, make_float3(1, 2, 3));
  helper.updateOrigin(1, make_float3(4, 5, 6));

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  std::vector<TextureParams<float4>> textures_buffer = helper.getTexturesBuffer();
  EXPECT_FLOAT_EQ(textures[0].origin.x, 0);
  EXPECT_FLOAT_EQ(textures[0].origin.y, 0);
  EXPECT_FLOAT_EQ(textures[0].origin.z, 0);
  EXPECT_FLOAT_EQ(textures_buffer[0].origin.x, 1);
  EXPECT_FLOAT_EQ(textures_buffer[0].origin.y, 2);
  EXPECT_FLOAT_EQ(textures_buffer[0].origin.z, 3);

  EXPECT_FLOAT_EQ(textures[1].origin.x, 0);
  EXPECT_FLOAT_EQ(textures[1].origin.y, 0);
  EXPECT_FLOAT_EQ(textures[1].origin.z, 0);
  EXPECT_FLOAT_EQ(textures_buffer[1].origin.x, 4);
  EXPECT_FLOAT_EQ(textures_buffer[1].origin.y, 5);
  EXPECT_FLOAT_EQ(textures_buffer[1].origin.z, 6);

  EXPECT_TRUE(textures_buffer[0].update_params);
  EXPECT_TRUE(textures_buffer[1].update_params);
  for (int i = 2; i < number; i++)
  {
    EXPECT_FALSE(textures_buffer[i].update_params);
  }
}

TEST_F(TextureHelperTest, UpdateRotationTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(1, 2, 3);
  new_rot_mat[1] = make_float3(4, 5, 6);
  new_rot_mat[2] = make_float3(7, 8, 9);
  helper.updateRotation(0, new_rot_mat);

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  std::vector<TextureParams<float4>> textures_buffer = helper.getTexturesBuffer();

  // buffer is updated
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].x, 1);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].y, 2);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].z, 3);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].x, 4);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].y, 5);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].z, 6);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].x, 7);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].y, 8);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].z, 9);
  // non buffer not updated
  EXPECT_FLOAT_EQ(textures[0].rotations[0].x, 1);
  EXPECT_FLOAT_EQ(textures[0].rotations[0].y, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[0].z, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[1].x, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[1].y, 1);
  EXPECT_FLOAT_EQ(textures[0].rotations[1].z, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[2].x, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[2].y, 0);
  EXPECT_FLOAT_EQ(textures[0].rotations[2].z, 1);

  new_rot_mat[2].x = 22;
  helper.updateRotation(1, new_rot_mat);
  textures_buffer = helper.getTexturesBuffer();

  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].x, 1);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].y, 2);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[0].z, 3);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].x, 4);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].y, 5);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[1].z, 6);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].x, 7);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].y, 8);
  EXPECT_FLOAT_EQ(textures_buffer[0].rotations[2].z, 9);

  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[0].x, 1);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[0].y, 2);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[0].z, 3);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[1].x, 4);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[1].y, 5);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[1].z, 6);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[2].x, 22);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[2].y, 8);
  EXPECT_FLOAT_EQ(textures_buffer[1].rotations[2].z, 9);
}

TEST_F(TextureHelperTest, SetExtentTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.setTestExtent(1, extent);

  helper.setExtent(0, extent);
  helper.setExtent(1, extent);

  extent = make_cudaExtent(4, 5, 0);
  helper.setExtent(2, extent);
  helper.setTestExtent(3, extent);
  helper.setExtent(3, extent);

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  std::vector<TextureParams<float4>> textures_buffer = helper.getTexturesBuffer();

  EXPECT_TRUE(textures_buffer[0].update_mem);
  EXPECT_EQ(textures_buffer[0].extent.width, 4);
  EXPECT_EQ(textures_buffer[0].extent.height, 5);
  EXPECT_EQ(textures_buffer[0].extent.depth, 6);
  EXPECT_FALSE(textures[0].update_mem);
  EXPECT_NE(textures[0].extent.width, 4);
  EXPECT_NE(textures[0].extent.height, 5);
  EXPECT_NE(textures[0].extent.depth, 6);

  EXPECT_FALSE(textures_buffer[1].update_mem);
  EXPECT_EQ(textures_buffer[1].extent.width, 4);
  EXPECT_EQ(textures_buffer[1].extent.height, 5);
  EXPECT_EQ(textures_buffer[1].extent.depth, 6);

  EXPECT_TRUE(textures_buffer[2].update_mem);
  EXPECT_EQ(textures_buffer[2].extent.width, 4);
  EXPECT_EQ(textures_buffer[2].extent.height, 5);
  EXPECT_EQ(textures_buffer[2].extent.depth, 0);

  EXPECT_FALSE(textures_buffer[3].update_mem);
  EXPECT_EQ(textures_buffer[3].extent.width, 4);
  EXPECT_EQ(textures_buffer[3].extent.height, 5);
  EXPECT_EQ(textures_buffer[3].extent.depth, 0);
}

TEST_F(TextureHelperTest, CopyToDeviceTest)
{
  int number = 16;
  TextureHelperImpl helper = TextureHelperImpl(number);

  helper.setTestFlagsInParam(0, false, false, false, false);
  helper.setTestFlagsInParam(1, false, false, false, true);
  helper.setTestFlagsInParam(2, false, false, true, false);
  helper.setTestFlagsInParam(3, false, false, true, true);
  helper.setTestFlagsInParam(4, false, true, false, false);
  helper.setTestFlagsInParam(5, false, true, false, true);
  helper.setTestFlagsInParam(6, false, true, true, false);
  helper.setTestFlagsInParam(7, false, true, true, true);
  helper.setTestFlagsInParam(8, true, false, false, false);
  helper.setTestFlagsInParam(9, true, false, false, true);
  helper.setTestFlagsInParam(10, true, false, true, false);
  helper.setTestFlagsInParam(11, true, false, true, true);
  helper.setTestFlagsInParam(12, true, true, false, false);
  helper.setTestFlagsInParam(13, true, true, false, true);
  helper.setTestFlagsInParam(14, true, true, true, false);
  helper.setTestFlagsInParam(15, true, true, true, true);

  // nothing should have happened since GPUMemStatus=False
  helper.copyToDevice();
  std::vector<TextureParams<float4>> textures = helper.getTextures();
  std::vector<TextureParams<float4>> textures_buffer = helper.getTexturesBuffer();

  for (int i = 0; i < textures_buffer.size(); i++)
  {
    EXPECT_FALSE(textures_buffer[i].update_mem);
    EXPECT_FALSE(textures_buffer[i].update_data);
    EXPECT_FALSE(textures_buffer[i].allocated);
    EXPECT_FALSE(textures_buffer[i].update_params);
  }

  EXPECT_FALSE(textures[0].update_mem);
  EXPECT_FALSE(textures[0].update_data);
  EXPECT_FALSE(textures[0].allocated);
  EXPECT_FALSE(textures[0].update_params);

  EXPECT_FALSE(textures[1].update_mem);
  EXPECT_FALSE(textures[1].update_data);
  EXPECT_FALSE(textures[1].allocated);
  EXPECT_TRUE(textures[1].update_params);

  EXPECT_FALSE(textures[2].update_mem);
  EXPECT_FALSE(textures[2].update_data);
  EXPECT_TRUE(textures[2].allocated);
  EXPECT_FALSE(textures[2].update_params);

  EXPECT_FALSE(textures[3].update_mem);
  EXPECT_FALSE(textures[3].update_data);
  EXPECT_TRUE(textures[3].allocated);
  EXPECT_TRUE(textures[3].update_params);

  EXPECT_FALSE(textures[4].update_mem);
  EXPECT_TRUE(textures[4].update_data);
  EXPECT_FALSE(textures[4].allocated);
  EXPECT_FALSE(textures[4].update_params);

  EXPECT_FALSE(textures[5].update_mem);
  EXPECT_TRUE(textures[5].update_data);
  EXPECT_FALSE(textures[5].allocated);
  EXPECT_TRUE(textures[5].update_params);

  EXPECT_FALSE(textures[6].update_mem);
  EXPECT_TRUE(textures[6].update_data);
  EXPECT_TRUE(textures[6].allocated);
  EXPECT_FALSE(textures[6].update_params);

  EXPECT_FALSE(textures[7].update_mem);
  EXPECT_TRUE(textures[7].update_data);
  EXPECT_TRUE(textures[7].allocated);
  EXPECT_TRUE(textures[7].update_params);

  EXPECT_TRUE(textures[8].update_mem);
  EXPECT_FALSE(textures[8].update_data);
  EXPECT_FALSE(textures[8].allocated);
  EXPECT_FALSE(textures[8].update_params);

  EXPECT_TRUE(textures[9].update_mem);
  EXPECT_FALSE(textures[9].update_data);
  EXPECT_FALSE(textures[9].allocated);
  EXPECT_TRUE(textures[9].update_params);

  EXPECT_TRUE(textures[10].update_mem);
  EXPECT_FALSE(textures[10].update_data);
  EXPECT_TRUE(textures[10].allocated);
  EXPECT_FALSE(textures[10].update_params);

  EXPECT_TRUE(textures[11].update_mem);
  EXPECT_FALSE(textures[11].update_data);
  EXPECT_TRUE(textures[11].allocated);
  EXPECT_TRUE(textures[11].update_params);

  EXPECT_TRUE(textures[12].update_mem);
  EXPECT_TRUE(textures[12].update_data);
  EXPECT_FALSE(textures[12].allocated);
  EXPECT_FALSE(textures[12].update_params);

  EXPECT_TRUE(textures[13].update_mem);
  EXPECT_TRUE(textures[13].update_data);
  EXPECT_FALSE(textures[13].allocated);
  EXPECT_TRUE(textures[13].update_params);

  EXPECT_TRUE(textures[14].update_mem);
  EXPECT_TRUE(textures[14].update_data);
  EXPECT_TRUE(textures[14].allocated);
  EXPECT_FALSE(textures[14].update_params);

  EXPECT_TRUE(textures[15].update_mem);
  EXPECT_TRUE(textures[15].update_data);
  EXPECT_TRUE(textures[15].allocated);
  EXPECT_TRUE(textures[15].update_params);

  EXPECT_EQ(helper.copyDataToGPUCalled, 0);
  EXPECT_EQ(helper.allocateCudaTextureCalled, 0);
  EXPECT_EQ(helper.createCudaTextureCalled, 0);
  EXPECT_EQ(helper.copyParamsToGPUCalled, 0);
  EXPECT_EQ(helper.freeCudaMemCalled, 0);

  helper.clearCounters();
  helper.setGpuMemStatus(true);
  helper.copyToDevice();
  textures = helper.getTextures();

  for (int i = 0; i < textures_buffer.size(); i++)
  {
    EXPECT_FALSE(textures_buffer[i].update_mem);
    EXPECT_FALSE(textures_buffer[i].update_data);
    EXPECT_FALSE(textures_buffer[i].allocated);
    EXPECT_FALSE(textures_buffer[i].update_params);
  }

  EXPECT_FALSE(textures[0].update_mem);
  EXPECT_FALSE(textures[0].update_data);
  EXPECT_FALSE(textures[0].allocated);
  EXPECT_FALSE(textures[0].update_params);

  EXPECT_FALSE(textures[1].update_mem);
  EXPECT_FALSE(textures[1].update_data);
  EXPECT_FALSE(textures[1].allocated);
  EXPECT_TRUE(textures[1].update_params);

  EXPECT_FALSE(textures[2].update_mem);
  EXPECT_FALSE(textures[2].update_data);
  EXPECT_TRUE(textures[2].allocated);
  EXPECT_FALSE(textures[2].update_params);

  EXPECT_FALSE(textures[3].update_mem);
  EXPECT_FALSE(textures[3].update_data);
  EXPECT_TRUE(textures[3].allocated);
  EXPECT_FALSE(textures[3].update_params);

  EXPECT_FALSE(textures[4].update_mem);
  EXPECT_TRUE(textures[4].update_data);
  EXPECT_FALSE(textures[4].allocated);
  EXPECT_FALSE(textures[4].update_params);

  EXPECT_FALSE(textures[5].update_mem);
  EXPECT_TRUE(textures[5].update_data);
  EXPECT_FALSE(textures[5].allocated);
  EXPECT_TRUE(textures[5].update_params);

  // false, true, true, false
  EXPECT_FALSE(textures[6].update_mem);
  EXPECT_FALSE(textures[6].update_data);
  EXPECT_TRUE(textures[6].allocated);
  EXPECT_FALSE(textures[6].update_params);

  EXPECT_FALSE(textures[7].update_mem);
  EXPECT_FALSE(textures[7].update_data);
  EXPECT_TRUE(textures[7].allocated);
  EXPECT_FALSE(textures[7].update_params);

  EXPECT_FALSE(textures[8].update_mem);
  EXPECT_FALSE(textures[8].update_data);
  EXPECT_TRUE(textures[8].allocated);
  EXPECT_FALSE(textures[8].update_params);

  // true, false, false, true
  EXPECT_FALSE(textures[9].update_mem);
  EXPECT_FALSE(textures[9].update_data);
  EXPECT_TRUE(textures[9].allocated);
  EXPECT_FALSE(textures[9].update_params);

  EXPECT_FALSE(textures[10].update_mem);
  EXPECT_FALSE(textures[10].update_data);
  EXPECT_TRUE(textures[10].allocated);
  EXPECT_FALSE(textures[10].update_params);

  EXPECT_FALSE(textures[11].update_mem);
  EXPECT_FALSE(textures[11].update_data);
  EXPECT_TRUE(textures[11].allocated);
  EXPECT_FALSE(textures[11].update_params);

  EXPECT_FALSE(textures[12].update_mem);
  EXPECT_FALSE(textures[12].update_data);
  EXPECT_TRUE(textures[12].allocated);
  EXPECT_FALSE(textures[12].update_params);

  EXPECT_FALSE(textures[13].update_mem);
  EXPECT_FALSE(textures[13].update_data);
  EXPECT_TRUE(textures[13].allocated);
  EXPECT_FALSE(textures[13].update_params);

  EXPECT_FALSE(textures[14].update_mem);
  EXPECT_FALSE(textures[14].update_data);
  EXPECT_TRUE(textures[14].allocated);
  EXPECT_FALSE(textures[14].update_params);

  EXPECT_FALSE(textures[15].update_mem);
  EXPECT_FALSE(textures[15].update_data);
  EXPECT_TRUE(textures[15].allocated);
  EXPECT_FALSE(textures[15].update_params);

  EXPECT_EQ(helper.copyDataToGPUCalled, 6);
  EXPECT_EQ(helper.allocateCudaTextureCalled, 8);
  EXPECT_EQ(helper.createCudaTextureCalled, 8);
  EXPECT_EQ(helper.copyParamsToGPUCalled, 6);
  EXPECT_EQ(helper.freeCudaMemCalled, 4);
}

TEST_F(TextureHelperTest, WorldPoseToMapPoseTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  // set it to be zero
  std::array<float3, 3> new_rot_mat{};
  helper.updateRotation(0, new_rot_mat);
  helper.updateOrigin(0, make_float3(0, 0, 0));

  float3 input = make_float3(0.1, 0.2, 0.3);
  float3 output;
  helper.worldPoseToMapPose(0, input, output);

  EXPECT_FLOAT_EQ(input.x, 0.1);
  EXPECT_FLOAT_EQ(input.y, 0.2);
  EXPECT_FLOAT_EQ(input.z, 0.3);
  EXPECT_FLOAT_EQ(output.x, 0.1);
  EXPECT_FLOAT_EQ(output.y, 0.2);
  EXPECT_FLOAT_EQ(output.z, 0.3);

  helper.copyToDevice();
  helper.worldPoseToMapPose(0, input, output);

  EXPECT_FLOAT_EQ(input.x, 0.1);
  EXPECT_FLOAT_EQ(input.y, 0.2);
  EXPECT_FLOAT_EQ(input.z, 0.3);
  EXPECT_FLOAT_EQ(output.x, 0);
  EXPECT_FLOAT_EQ(output.y, 0);
  EXPECT_FLOAT_EQ(output.z, 0);

  // set rot mat non zero
  new_rot_mat[0] = make_float3(1, 2, 3);
  new_rot_mat[1] = make_float3(4, 5, 6);
  new_rot_mat[2] = make_float3(7, 8, 9);
  helper.updateRotation(0, new_rot_mat);
  helper.copyToDevice();

  helper.worldPoseToMapPose(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 0.1);
  EXPECT_FLOAT_EQ(input.y, 0.2);
  EXPECT_FLOAT_EQ(input.z, 0.3);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 0.2 * 2 + 0.3 * 3);
  EXPECT_FLOAT_EQ(output.y, 0.1 * 4 + 0.2 * 5 + 0.3 * 6);
  EXPECT_FLOAT_EQ(output.z, 0.1 * 7 + 0.2 * 8 + 0.3 * 9);

  // set origin non zero
  new_rot_mat[0] = make_float3(0, 0, 0);
  new_rot_mat[1] = make_float3(0, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 0);
  helper.updateRotation(0, new_rot_mat);
  helper.updateOrigin(0, make_float3(0.1, 0.2, 0.3));
  helper.copyToDevice();

  helper.worldPoseToMapPose(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 0.1);
  EXPECT_FLOAT_EQ(input.y, 0.2);
  EXPECT_FLOAT_EQ(input.z, 0.3);
  EXPECT_FLOAT_EQ(output.x, 0);
  EXPECT_FLOAT_EQ(output.y, 0);
  EXPECT_FLOAT_EQ(output.z, 0);

  // set rot mat non zero
  new_rot_mat[0] = make_float3(1, 2, 3);
  new_rot_mat[1] = make_float3(4, 5, 6);
  new_rot_mat[2] = make_float3(7, 8, 9);
  helper.updateRotation(0, new_rot_mat);
  helper.copyToDevice();
  input = make_float3(0.2, 0.4, 0.6);

  helper.worldPoseToMapPose(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 0.2);
  EXPECT_FLOAT_EQ(input.y, 0.4);
  EXPECT_FLOAT_EQ(input.z, 0.6);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 0.2 * 2 + 0.3 * 3);
  EXPECT_FLOAT_EQ(output.y, 0.1 * 4 + 0.2 * 5 + 0.3 * 6);
  EXPECT_FLOAT_EQ(output.z, 0.1 * 7 + 0.2 * 8 + 0.3 * 9);
}

TEST_F(TextureHelperTest, BodyOffsetToWorldPoseTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  // set it to be zero
  std::array<float3, 3> new_rot_mat{};
  helper.updateRotation(0, new_rot_mat);
  helper.updateOrigin(0, make_float3(0, 0, 0));

  float3 input = make_float3(0.1, 0.2, 0.3);
  float3 offset = make_float3(1.1, 1.3, 1.5);
  float3 rotation = make_float3(0, 0, 0);
  float3 output;
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);

  EXPECT_FLOAT_EQ(input.x, 0.1);
  EXPECT_FLOAT_EQ(input.y, 0.2);
  EXPECT_FLOAT_EQ(input.z, 0.3);
  EXPECT_FLOAT_EQ(output.x, 1.2);
  EXPECT_FLOAT_EQ(output.y, 1.5);
  EXPECT_FLOAT_EQ(output.z, 1.8);

  // rotate by positive 90 degrees yaw
  rotation = make_float3(0, 0, M_PI_2);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, -1.2);
  EXPECT_FLOAT_EQ(output.y, 1.3);
  EXPECT_FLOAT_EQ(output.z, 1.8);

  // rotate by -90 degrees yaw
  rotation = make_float3(0, 0, -M_PI_2);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 1.3);
  EXPECT_FLOAT_EQ(output.y, 0.2 - 1.1);
  EXPECT_FLOAT_EQ(output.z, 1.8);

  // rotate by +45 degrees yaw
  rotation = make_float3(0, 0, M_PI_4);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  // EXPECT_FLOAT_EQ(output.x, -0.041421525);
  EXPECT_NEAR(output.x, -0.041421525, 1.0e-6);
  EXPECT_FLOAT_EQ(output.y, 1.8970563);
  EXPECT_FLOAT_EQ(output.z, 1.8);

  // rotate by +90 degrees roll
  rotation = make_float3(M_PI_2, 0, 0);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 1.1);
  EXPECT_FLOAT_EQ(output.y, 0.2 - 1.5);
  EXPECT_FLOAT_EQ(output.z, 0.3 + 1.3);

  // rotate by -90 degrees roll
  rotation = make_float3(-M_PI_2, 0, 0);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 1.1);
  EXPECT_FLOAT_EQ(output.y, 0.2 + 1.5);
  EXPECT_FLOAT_EQ(output.z, 0.3 - 1.3);

  // rotate by +90 degrees roll
  rotation = make_float3(0, M_PI_2, 0);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, 0.1 + 1.5);
  EXPECT_FLOAT_EQ(output.y, 0.2 + 1.3);
  EXPECT_FLOAT_EQ(output.z, 0.3 - 1.1);

  // rotate by -90 degrees roll
  rotation = make_float3(0, -M_PI_2, 0);
  helper.copyToDevice();
  helper.bodyOffsetToWorldPose(offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, 0.1 - 1.5);
  EXPECT_FLOAT_EQ(output.y, 0.2 + 1.3);
  EXPECT_FLOAT_EQ(output.z, 0.3 + 1.1);
}

TEST_F(TextureHelperTest, MapPoseToTexCoordTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  float3 input = make_float3(1.0, 2.0, 3.0);
  float3 output;

  helper.updateResolution(0, 10);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.setExtent(0, extent);
  helper.copyToDevice();

  helper.mapPoseToTexCoord(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 1.0);
  EXPECT_FLOAT_EQ(input.y, 2.0);
  EXPECT_FLOAT_EQ(input.z, 3.0);
  EXPECT_FLOAT_EQ(output.x, (0.1) / 4);
  EXPECT_FLOAT_EQ(output.y, (0.2) / 5);
  EXPECT_FLOAT_EQ(output.z, (0.3) / 6);
}

TEST_F(TextureHelperTest, MapPoseToTexCoordIndResTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  float3 input = make_float3(1.0, 2.0, 3.0);
  float3 output;

  float3 resolution = make_float3(10, 20, 30);
  helper.updateResolution(0, resolution);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.setExtent(0, extent);
  helper.copyToDevice();

  helper.mapPoseToTexCoord(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 1.0);
  EXPECT_FLOAT_EQ(input.y, 2.0);
  EXPECT_FLOAT_EQ(input.z, 3.0);
  EXPECT_FLOAT_EQ(output.x, (0.1) / 4);
  EXPECT_FLOAT_EQ(output.y, (0.1) / 5);
  EXPECT_FLOAT_EQ(output.z, (0.1) / 6);
}

TEST_F(TextureHelperTest, WorldPoseToTexCoordTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  float3 input = make_float3(0.2, 0.4, 0.6);
  float3 output;

  helper.updateResolution(0, 10);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.setExtent(0, extent);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(1, 2, 3);
  new_rot_mat[1] = make_float3(4, 5, 6);
  new_rot_mat[2] = make_float3(7, 8, 9);
  helper.updateRotation(0, new_rot_mat);
  helper.updateOrigin(0, make_float3(0.1, 0.2, 0.3));
  helper.copyToDevice();

  helper.worldPoseToTexCoord(0, input, output);
  EXPECT_FLOAT_EQ(input.x, 0.2);
  EXPECT_FLOAT_EQ(input.y, 0.4);
  EXPECT_FLOAT_EQ(input.z, 0.6);
  EXPECT_FLOAT_EQ(output.x, ((0.1 * 1 + 0.2 * 2 + 0.3 * 3) / 10) / 4);
  EXPECT_FLOAT_EQ(output.y, ((0.1 * 4 + 0.2 * 5 + 0.3 * 6) / 10) / 5);
  EXPECT_FLOAT_EQ(output.z, ((0.1 * 7 + 0.2 * 8 + 0.3 * 9) / 10) / 6);
}

TEST_F(TextureHelperTest, BodyOffsetToTexCoordTest)
{
  int number = 5;
  TextureHelperImpl helper = TextureHelperImpl(number);

  const float3 input = make_float3(-0.9, -0.9, -0.9);
  float3 output;

  helper.updateResolution(0, 10);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.setExtent(0, extent);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(1, 2, 3);
  new_rot_mat[1] = make_float3(4, 5, 6);
  new_rot_mat[2] = make_float3(7, 8, 9);
  helper.updateRotation(0, new_rot_mat);
  helper.updateOrigin(0, make_float3(0.1, 0.2, 0.3));
  helper.copyToDevice();

  float3 offset = make_float3(1.1, 1.3, 1.5);
  float3 rotation = make_float3(0, 0, 0);

  helper.bodyOffsetWorldToTexCoord(0, offset, input, rotation, output);
  EXPECT_FLOAT_EQ(output.x, ((0.1 * 1 + 0.2 * 2 + 0.3 * 3) / 10) / 4);
  EXPECT_FLOAT_EQ(output.y, ((0.1 * 4 + 0.2 * 5 + 0.3 * 6) / 10) / 5);
  EXPECT_FLOAT_EQ(output.z, ((0.1 * 7 + 0.2 * 8 + 0.3 * 9) / 10) / 6);
}

TEST_F(TextureHelperTest, AddNewTextureTest)
{
  int number = 8;
  TextureHelperImpl helper = TextureHelperImpl(number);
  cudaExtent extent = make_cudaExtent(4, 5, 6);
  helper.addNewTexture(extent);

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  EXPECT_EQ(textures.size(), number + 1);
  for (int i = 0; i < number + 1; i++)
  {
    auto texture = textures.at(i);

    EXPECT_EQ(texture.use, false);
    // since GPUMemStatus_ = false, this should always be false
    EXPECT_EQ(texture.allocated, false);
    EXPECT_EQ(texture.update_data, false);
    EXPECT_EQ(texture.update_mem, false);
    EXPECT_EQ(texture.resDesc.resType, cudaResourceTypeArray);
    EXPECT_EQ(texture.channelDesc.x, 32);
    EXPECT_EQ(texture.channelDesc.y, 32);
    EXPECT_EQ(texture.channelDesc.z, 32);
    EXPECT_EQ(texture.channelDesc.w, 32);

    EXPECT_EQ(texture.texDesc.addressMode[0], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.addressMode[1], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.filterMode, cudaFilterModeLinear);
    EXPECT_EQ(texture.texDesc.readMode, cudaReadModeElementType);
    EXPECT_EQ(texture.texDesc.normalizedCoords, 1);
  }
  EXPECT_EQ(textures[8].extent.width, 4);
  EXPECT_EQ(textures[8].extent.height, 5);
  EXPECT_EQ(textures[8].extent.depth, 6);

  helper.setGpuMemStatus(true);
  helper.addNewTexture(extent);
  textures = helper.getTextures();
  EXPECT_EQ(textures.size(), number + 2);
  for (int i = 0; i < number + 2; i++)
  {
    auto texture = textures.at(i);

    EXPECT_EQ(texture.use, false);
    if (i == 9)
    {
      EXPECT_EQ(texture.allocated, true);
    }
    else
    {
      EXPECT_EQ(texture.allocated, false);
    }
    EXPECT_EQ(texture.update_data, false);
    EXPECT_EQ(texture.update_mem, false);
    EXPECT_EQ(texture.resDesc.resType, cudaResourceTypeArray);
    EXPECT_EQ(texture.channelDesc.x, 32);
    EXPECT_EQ(texture.channelDesc.y, 32);
    EXPECT_EQ(texture.channelDesc.z, 32);
    EXPECT_EQ(texture.channelDesc.w, 32);

    EXPECT_EQ(texture.texDesc.addressMode[0], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.addressMode[1], cudaAddressModeClamp);
    EXPECT_EQ(texture.texDesc.filterMode, cudaFilterModeLinear);
    EXPECT_EQ(texture.texDesc.readMode, cudaReadModeElementType);
    EXPECT_EQ(texture.texDesc.normalizedCoords, 1);
  }
  EXPECT_EQ(textures[9].extent.width, 4);
  EXPECT_EQ(textures[9].extent.height, 5);
  EXPECT_EQ(textures[9].extent.depth, 6);
}
