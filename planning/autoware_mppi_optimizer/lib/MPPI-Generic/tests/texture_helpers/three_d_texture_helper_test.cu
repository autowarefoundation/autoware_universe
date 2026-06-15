#include <gtest/gtest.h>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>

#include <mppi/utils/texture_helpers/three_d_texture_helper.cuh>
#include <kernel_tests/utils/texture_test_kernels.cuh>

class ThreeDTextureHelperTest : public testing::Test
{
protected:
  void SetUp() override
  {
    CudaCheckError();
    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(100.0, 2.0);
  }

  void TearDown() override
  {
    CudaCheckError();
  }

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
};

TEST_F(ThreeDTextureHelperTest, Constructor)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(4);

  EXPECT_EQ(helper.getCpuValues().size(), 4);
  EXPECT_EQ(helper.getLayerCopy().size(), 4);
}

TEST_F(ThreeDTextureHelperTest, ThreeDAllocateCudaTextureTest)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(2);

  cudaExtent extent = make_cudaExtent(10, 20, 1);
  helper.setExtent(0, extent);
  extent = make_cudaExtent(30, 40, 1);
  helper.setExtent(1, extent);

  std::vector<TextureParams<float4>> textures = helper.getBufferTextures();
  EXPECT_EQ(textures[0].array_d, nullptr);
  EXPECT_EQ(textures[0].tex_d, 0);
  EXPECT_TRUE(textures[0].update_mem);
  EXPECT_FALSE(textures[0].allocated);

  helper.GPUSetup();

  textures = helper.getTextures();
  EXPECT_NE(textures[0].array_d, nullptr);
  EXPECT_NE(textures[0].tex_d, 0);
  EXPECT_FALSE(textures[0].update_mem);
  EXPECT_TRUE(textures[0].allocated);
}

TEST_F(ThreeDTextureHelperTest, UpdateTexture)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);

  cudaExtent extent = make_cudaExtent(10, 20, 2);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i, i + 1, i + 2, i + 3);
  }

  helper.updateTexture(0, 0, data_vec);

  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i + 200, i + 200 + 1, i + 200 + 2, i + 200 + 3);
  }
  helper.updateTexture(0, 1, data_vec);

  std::vector<TextureParams<float4>> textures = helper.getBufferTextures();
  EXPECT_TRUE(textures[0].update_data);

  auto cpu_values = helper.getCpuBufferValues()[0];
  for (int i = 0; i < cpu_values.size(); i++)
  {
    EXPECT_FLOAT_EQ(cpu_values[i].x, i);
    EXPECT_FLOAT_EQ(cpu_values[i].y, i + 1);
    EXPECT_FLOAT_EQ(cpu_values[i].z, i + 2);
    EXPECT_FLOAT_EQ(cpu_values[i].w, i + 3);
  }
}

TEST_F(ThreeDTextureHelperTest, UpdateTextureColumnMajor)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(3);
  cudaExtent extent = make_cudaExtent(10, 20, 3);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  // just in case I am stupid
  std::set<float> total_set;
  for (int depth = 0; depth < 3; depth++)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = depth * 20 * 10 + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
      total_set.insert(val);
    }
    EXPECT_EQ(total_set.size(), 200 * (depth + 1));
    helper.updateTexture(0, depth, data_vec, true);
  }

  // returns a rowMajor vector
  auto cpu_values = helper.getCpuBufferValues()[0];

  for (int k = 0; k < 3; k++)
  {
    for (int i = 0; i < 20; i++)
    {
      for (int j = 0; j < 10; j++)
      {
        int columnMajorIndex = (k * 10 * 20) + j * 20 + i;
        int rowVectorIndex = (k * 10 * 20) + i * 10 + j;
        EXPECT_FLOAT_EQ(cpu_values[rowVectorIndex].x, columnMajorIndex) << " at index: " << i;
        EXPECT_EQ(total_set.erase(rowVectorIndex), 1);
      }
    }
  }
  EXPECT_EQ(total_set.size(), 0);
}

TEST_F(ThreeDTextureHelperTest, UpdateTextureException)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(2);

  cudaExtent extent = make_cudaExtent(10, 20, 1);
  helper.setExtent(0, extent);
  extent = make_cudaExtent(30, 40, 1);
  helper.setExtent(1, extent);

  std::vector<float4> data_vec;
  data_vec.resize(30 * 40);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i, i + 1, i + 2, i + 3);
  }

  EXPECT_THROW(helper.updateTexture(0, 0, data_vec), std::runtime_error);
}

TEST_F(ThreeDTextureHelperTest, CopyDataToGPUOneGo)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);
  helper.GPUSetup();

  cudaExtent extent = make_cudaExtent(10, 20, 5);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  for (int depth = 0; depth < 5; depth++)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = depth * 20 * 10 + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
    }
    helper.updateTexture(0, depth, data_vec);
  }

  helper.copyToDevice(true);

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  EXPECT_TRUE(textures[0].allocated);
  EXPECT_FALSE(textures[0].update_data);
  EXPECT_FALSE(textures[0].update_mem);

  std::vector<float4> query_points;
  // X
  query_points.push_back(make_float4(0.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.05, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.95, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(1.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.45, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.5, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.55, 0.0, 0.0, 0.0));

  // Y
  query_points.push_back(make_float4(0.0, 0.025, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.05, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.075, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.975, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 1.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.475, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.5, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.525, 0.0, 0.0));

  // Z
  query_points.push_back(make_float4(0.0, 0.0, 0.1, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.2, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.3, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 1.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.9, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.8, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.7, 0.0));

  auto results = getTextureAtPointsKernel<ThreeDTextureHelper<float4>, float4>(helper, query_points);

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 0.0);
  EXPECT_FLOAT_EQ(results[0].y, 1.0);
  EXPECT_FLOAT_EQ(results[0].z, 2.0);
  EXPECT_FLOAT_EQ(results[0].w, 3.0);
  EXPECT_FLOAT_EQ(results[1].x, 0.0);
  EXPECT_FLOAT_EQ(results[1].y, 1.0);
  EXPECT_FLOAT_EQ(results[1].z, 2.0);
  EXPECT_FLOAT_EQ(results[1].w, 3.0);

  EXPECT_FLOAT_EQ(results[2].x, 9.0);
  EXPECT_FLOAT_EQ(results[2].y, 10.0);
  EXPECT_FLOAT_EQ(results[2].z, 11.0);
  EXPECT_FLOAT_EQ(results[2].w, 12.0);
  EXPECT_FLOAT_EQ(results[3].x, 9.0);
  EXPECT_FLOAT_EQ(results[3].y, 10.0);
  EXPECT_FLOAT_EQ(results[3].z, 11.0);
  EXPECT_FLOAT_EQ(results[3].w, 12.0);

  EXPECT_FLOAT_EQ(results[4].x, 4.0);
  EXPECT_FLOAT_EQ(results[4].y, 5.0);
  EXPECT_FLOAT_EQ(results[4].z, 6.0);
  EXPECT_FLOAT_EQ(results[4].w, 7.0);
  EXPECT_FLOAT_EQ(results[5].x, 4.5);
  EXPECT_FLOAT_EQ(results[5].y, 5.5);
  EXPECT_FLOAT_EQ(results[5].z, 6.5);
  EXPECT_FLOAT_EQ(results[5].w, 7.5);
  EXPECT_FLOAT_EQ(results[6].x, 5.0);
  EXPECT_FLOAT_EQ(results[6].y, 6.0);
  EXPECT_FLOAT_EQ(results[6].z, 7.0);
  EXPECT_FLOAT_EQ(results[6].w, 8.0);

  EXPECT_FLOAT_EQ(results[7].x, 0.0);
  EXPECT_FLOAT_EQ(results[8].x, 5);
  EXPECT_FLOAT_EQ(results[9].x, 10);

  EXPECT_FLOAT_EQ(results[10].x, 190);
  EXPECT_FLOAT_EQ(results[11].x, 190);

  EXPECT_FLOAT_EQ(results[12].x, 90);
  EXPECT_FLOAT_EQ(results[13].x, 95);
  EXPECT_FLOAT_EQ(results[14].x, 100);

  EXPECT_FLOAT_EQ(results[15].x, 0.0);
  EXPECT_FLOAT_EQ(results[16].x, 100.0);
  EXPECT_FLOAT_EQ(results[17].x, 200.0);
  EXPECT_FLOAT_EQ(results[18].x, 800.0);
  EXPECT_FLOAT_EQ(results[19].x, 800.0);
  EXPECT_FLOAT_EQ(results[20].x, 700.0);
  EXPECT_FLOAT_EQ(results[21].x, 600.0);
}

TEST_F(ThreeDTextureHelperTest, TestCPUQuery)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);
  helper.GPUSetup();
  const int WIDTH = 10;
  const int HEIGHT = 20;
  const int DEPTH = 5;

  cudaExtent extent = make_cudaExtent(WIDTH, HEIGHT, DEPTH);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(WIDTH * HEIGHT);
  for (int depth = 0; depth < DEPTH; depth++)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = depth * HEIGHT * WIDTH + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
    }
    helper.updateTexture(0, depth, data_vec);
  }

  helper.copyToDevice(true);

  std::vector<TextureParams<float4>> textures = helper.getTextures();
  EXPECT_TRUE(textures[0].allocated);
  EXPECT_FALSE(textures[0].update_data);
  EXPECT_FALSE(textures[0].update_mem);

  std::vector<float4> query_points;
  // X
  query_points.push_back(make_float4(0.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.05, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.95, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(1.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.45, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.5, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.55, 0.0, 0.0, 0.0));

  // Y
  query_points.push_back(make_float4(0.0, 0.025, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.05, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.075, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.975, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 1.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.475, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.5, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.525, 0.0, 0.0));

  // Z
  query_points.push_back(make_float4(0.0, 0.0, 0.1, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.2, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.3, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 1.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.9, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.8, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.7, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 1.2, 0.0));

  std::vector<float4> results;
  for (const auto& query : query_points)
  {
    float3 xyz_point = make_float3(query.x, query.y, query.z);
    int index = round(query.w);
    results.push_back(helper.queryTexture(index, xyz_point));
  }

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 0.0);
  EXPECT_FLOAT_EQ(results[0].y, 1.0);
  EXPECT_FLOAT_EQ(results[0].z, 2.0);
  EXPECT_FLOAT_EQ(results[0].w, 3.0);
  EXPECT_FLOAT_EQ(results[1].x, 0.0);
  EXPECT_FLOAT_EQ(results[1].y, 1.0);
  EXPECT_FLOAT_EQ(results[1].z, 2.0);
  EXPECT_FLOAT_EQ(results[1].w, 3.0);

  EXPECT_FLOAT_EQ(results[2].x, 9.0);
  EXPECT_FLOAT_EQ(results[2].y, 10.0);
  EXPECT_FLOAT_EQ(results[2].z, 11.0);
  EXPECT_FLOAT_EQ(results[2].w, 12.0);
  EXPECT_FLOAT_EQ(results[3].x, 9.0);
  EXPECT_FLOAT_EQ(results[3].y, 10.0);
  EXPECT_FLOAT_EQ(results[3].z, 11.0);
  EXPECT_FLOAT_EQ(results[3].w, 12.0);

  EXPECT_FLOAT_EQ(results[4].x, 4.0);
  EXPECT_FLOAT_EQ(results[4].y, 5.0);
  EXPECT_FLOAT_EQ(results[4].z, 6.0);
  EXPECT_FLOAT_EQ(results[4].w, 7.0);
  EXPECT_FLOAT_EQ(results[5].x, 4.5);
  EXPECT_FLOAT_EQ(results[5].y, 5.5);
  EXPECT_FLOAT_EQ(results[5].z, 6.5);
  EXPECT_FLOAT_EQ(results[5].w, 7.5);
  EXPECT_FLOAT_EQ(results[6].x, 5.0);
  EXPECT_FLOAT_EQ(results[6].y, 6.0);
  EXPECT_FLOAT_EQ(results[6].z, 7.0);
  EXPECT_FLOAT_EQ(results[6].w, 8.0);

  EXPECT_FLOAT_EQ(results[7].x, 0.0);
  EXPECT_FLOAT_EQ(results[8].x, 5);
  EXPECT_FLOAT_EQ(results[9].x, 10);

  EXPECT_FLOAT_EQ(results[10].x, 190);
  EXPECT_FLOAT_EQ(results[11].x, 190);

  EXPECT_FLOAT_EQ(results[12].x, 90);
  EXPECT_FLOAT_EQ(results[13].x, 95);
  EXPECT_FLOAT_EQ(results[14].x, 100);

  EXPECT_FLOAT_EQ(results[15].x, 0.0);
  EXPECT_FLOAT_EQ(results[16].x, 100.0);
  EXPECT_FLOAT_EQ(results[17].x, 200.0);
  EXPECT_FLOAT_EQ(results[18].x, 800.0);
  EXPECT_FLOAT_EQ(results[19].x, 800.0);
  EXPECT_FLOAT_EQ(results[20].x, 700.0);
  EXPECT_FLOAT_EQ(results[21].x, 600.0);
  EXPECT_FLOAT_EQ(results[22].x, 800.0);
}

TEST_F(ThreeDTextureHelperTest, CopyDataToGPUSplitMiddle)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);
  helper.GPUSetup();

  cudaExtent extent = make_cudaExtent(10, 20, 5);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  // 0, 200, 400, 600, 800
  for (int depth = 0; depth < 5; depth++)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = depth * 20 * 10 + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
    }
    helper.updateTexture(0, depth, data_vec);
  }

  // fill in even indexes
  // 0, 1200, 400, 1600, 800
  for (int depth = 1; depth < 5; depth += 2)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = (depth + 5) * 20 * 10 + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
    }
    helper.updateTexture(0, depth, data_vec);
  }
  helper.copyToDevice(true);

  std::vector<float4> query_points;
  query_points.push_back(make_float4(0.0, 0.0, 0.1, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.2, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.3, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.5, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.6, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.7, 0.0));

  auto results = getTextureAtPointsKernel<ThreeDTextureHelper<float4>, float4>(helper, query_points);

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 0.0);
  EXPECT_FLOAT_EQ(results[1].x, 600);
  EXPECT_FLOAT_EQ(results[2].x, 1200);
  EXPECT_FLOAT_EQ(results[3].x, 400);
  EXPECT_FLOAT_EQ(results[4].x, 1000);
  EXPECT_FLOAT_EQ(results[5].x, 1600);

  // fill in odd indexes
  // 2000, 1200, 2400, 1600, 2800
  for (int depth = 0; depth < 5; depth += 2)
  {
    for (int i = 0; i < data_vec.size(); i++)
    {
      float val = (depth + 10) * 20 * 10 + i;
      data_vec[i] = make_float4(val, val + 1, val + 2, val + 3);
    }
    helper.updateTexture(0, depth, data_vec);
  }
  helper.copyToDevice(true);

  results = getTextureAtPointsKernel<ThreeDTextureHelper<float4>, float4>(helper, query_points);

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 2000.0);
  EXPECT_FLOAT_EQ(results[1].x, 1600);
  EXPECT_FLOAT_EQ(results[2].x, 1200);
  EXPECT_FLOAT_EQ(results[3].x, 2400);
  EXPECT_FLOAT_EQ(results[4].x, 2000);
  EXPECT_FLOAT_EQ(results[5].x, 1600);
}

TEST_F(ThreeDTextureHelperTest, CheckWrapping)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);

  cudaExtent extent = make_cudaExtent(10, 20, 2);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i, i + 1, i + 2, i + 3);
  }

  helper.updateTexture(0, 0, data_vec);

  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i + 200, i + 200 + 1, i + 200 + 2, i + 200 + 3);
  }
  helper.updateTexture(0, 1, data_vec);
  helper.updateAddressMode(0, 2, cudaAddressModeWrap);
  helper.GPUSetup();
  helper.copyToDevice(true);

  std::vector<float4> query_points;
  query_points.push_back(make_float4(0.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.25, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.5, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.75, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 1.0, 0.0));

  auto results = getTextureAtPointsKernel<ThreeDTextureHelper<float4>, float4>(helper, query_points);

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 100.0);
  EXPECT_FLOAT_EQ(results[1].x, 0);
  EXPECT_FLOAT_EQ(results[2].x, 100);
  EXPECT_FLOAT_EQ(results[3].x, 200);
  EXPECT_FLOAT_EQ(results[4].x, 100);
}

TEST_F(ThreeDTextureHelperTest, CheckCPUWrapping)
{
  ThreeDTextureHelper<float4> helper = ThreeDTextureHelper<float4>(1);

  cudaExtent extent = make_cudaExtent(10, 20, 2);
  helper.setExtent(0, extent);

  std::vector<float4> data_vec;
  data_vec.resize(10 * 20);
  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i, i + 1, i + 2, i + 3);
  }

  helper.updateTexture(0, 0, data_vec);

  for (int i = 0; i < data_vec.size(); i++)
  {
    data_vec[i] = make_float4(i + 200, i + 200 + 1, i + 200 + 2, i + 200 + 3);
  }
  helper.updateTexture(0, 1, data_vec);
  helper.updateAddressMode(0, 2, cudaAddressModeWrap);
  helper.GPUSetup();
  helper.copyToDevice(true);

  std::vector<float4> query_points;
  query_points.push_back(make_float4(0.0, 0.0, 0.0, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.25, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.5, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 0.75, 0.0));
  query_points.push_back(make_float4(0.0, 0.0, 1.0, 0.0));

  std::vector<float4> results;
  for (const auto& query : query_points)
  {
    float3 xyz_point = make_float3(query.x, query.y, query.z);
    int index = round(query.w);
    results.push_back(helper.queryTexture(index, xyz_point));
  }

  EXPECT_FLOAT_EQ(results.size(), query_points.size());
  EXPECT_FLOAT_EQ(results[0].x, 100.0);
  EXPECT_FLOAT_EQ(results[1].x, 0);
  EXPECT_FLOAT_EQ(results[2].x, 100);
  EXPECT_FLOAT_EQ(results[3].x, 200);
  EXPECT_FLOAT_EQ(results[4].x, 100);
}
