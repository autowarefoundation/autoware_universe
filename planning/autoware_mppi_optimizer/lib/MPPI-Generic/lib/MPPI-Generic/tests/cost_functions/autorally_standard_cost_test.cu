//
// Created by jason on 1/7/20.
//

#include <gtest/gtest.h>
#include <mppi/cost_functions/autorally/ar_standard_cost.cuh>
#include <kernel_tests/cost_functions/autorally/ar_standard_cost_kernel_test.cuh>

// Auto-generated header file
#include <autorally_test_map.h>

TEST(ARStandardCost, Constructor)
{
  ARStandardCost cost;
}

TEST(ARStandardCost, BindStream)
{
  cudaStream_t stream;

  HANDLE_ERROR(cudaStreamCreate(&stream));

  ARStandardCost cost(stream);

  EXPECT_EQ(cost.stream_, stream) << "Stream binding failure.";

  HANDLE_ERROR(cudaStreamDestroy(stream));
}

TEST(ARStandardCost, SetGetParamsHost)
{
  ARStandardCostParams params;
  params.desired_speed = 25;
  params.speed_coeff = 2;
  params.track_coeff = 100;
  params.max_slip_ang = 1.5;
  params.slip_coeff = 1000;
  params.track_slop = 0.2;
  params.crash_coeff = 10000;
  params.control_cost_coeff[0] = 20;
  params.control_cost_coeff[1] = 10;
  params.boundary_threshold = 10;
  params.discount = 0.9;
  params.grid_res = 2;

  params.r_c1.x = 0;
  params.r_c1.y = 1;
  params.r_c1.z = 2;
  params.r_c2.x = 3;
  params.r_c2.y = 4;
  params.r_c2.z = 5;
  params.trs.x = 6;
  params.trs.y = 7;
  params.trs.z = 8;
  ARStandardCost cost;

  cost.setParams(params);
  ARStandardCostParams result_params = cost.getParams();

  EXPECT_FLOAT_EQ(params.desired_speed, result_params.desired_speed);
  EXPECT_FLOAT_EQ(params.speed_coeff, result_params.speed_coeff);
  EXPECT_FLOAT_EQ(params.track_coeff, result_params.track_coeff);
  EXPECT_FLOAT_EQ(params.max_slip_ang, result_params.max_slip_ang);
  EXPECT_FLOAT_EQ(params.slip_coeff, result_params.slip_coeff);
  EXPECT_FLOAT_EQ(params.track_slop, result_params.track_slop);
  EXPECT_FLOAT_EQ(params.crash_coeff, result_params.crash_coeff);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[0], result_params.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[1], result_params.control_cost_coeff[1]);
  EXPECT_FLOAT_EQ(params.boundary_threshold, result_params.boundary_threshold);
  EXPECT_FLOAT_EQ(params.discount, result_params.discount);
  EXPECT_EQ(params.grid_res, result_params.grid_res);
  EXPECT_FLOAT_EQ(params.r_c1.x, result_params.r_c1.x);
  EXPECT_FLOAT_EQ(params.r_c1.y, result_params.r_c1.y);
  EXPECT_FLOAT_EQ(params.r_c1.z, result_params.r_c1.z);
  EXPECT_FLOAT_EQ(params.r_c2.x, result_params.r_c2.x);
  EXPECT_FLOAT_EQ(params.r_c2.y, result_params.r_c2.y);
  EXPECT_FLOAT_EQ(params.r_c2.z, result_params.r_c2.z);
  EXPECT_FLOAT_EQ(params.trs.x, result_params.trs.x);
  EXPECT_FLOAT_EQ(params.trs.y, result_params.trs.y);
  EXPECT_FLOAT_EQ(params.trs.z, result_params.trs.z);
}

TEST(ARStandardCost, GPUSetupAndParamsToDeviceTest)
{
  // TODO make sre GPUMemstatus is false on the GPU so deallocation can be automatic
  ARStandardCostParams params;
  ARStandardCost cost;
  params.desired_speed = 25;
  params.speed_coeff = 2;
  params.track_coeff = 100;
  params.max_slip_ang = 1.5;
  params.slip_coeff = 1000;
  params.track_slop = 0.2;
  params.crash_coeff = 10000;
  params.control_cost_coeff[0] = 20;
  params.control_cost_coeff[1] = 10;
  params.boundary_threshold = 10;
  params.discount = 0.9;
  params.grid_res = 2;

  params.r_c1.x = 0;
  params.r_c1.y = 1;
  params.r_c1.z = 2;
  params.r_c2.x = 3;
  params.r_c2.y = 4;
  params.r_c2.z = 5;
  params.trs.x = 6;
  params.trs.y = 7;
  params.trs.z = 8;
  cost.setParams(params);

  EXPECT_EQ(cost.GPUMemStatus_, false);
  EXPECT_EQ(cost.cost_d_, nullptr);

  cost.GPUSetup();

  EXPECT_EQ(cost.GPUMemStatus_, true);
  EXPECT_NE(cost.cost_d_, nullptr);

  ARStandardCostParams result_params;
  int width, height;
  launchParameterTestKernel<>(cost, result_params, width, height);

  EXPECT_FLOAT_EQ(params.desired_speed, result_params.desired_speed);
  EXPECT_FLOAT_EQ(params.speed_coeff, result_params.speed_coeff);
  EXPECT_FLOAT_EQ(params.track_coeff, result_params.track_coeff);
  EXPECT_FLOAT_EQ(params.max_slip_ang, result_params.max_slip_ang);
  EXPECT_FLOAT_EQ(params.slip_coeff, result_params.slip_coeff);
  EXPECT_FLOAT_EQ(params.track_slop, result_params.track_slop);
  EXPECT_FLOAT_EQ(params.crash_coeff, result_params.crash_coeff);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[0], result_params.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[1], result_params.control_cost_coeff[1]);
  EXPECT_FLOAT_EQ(params.boundary_threshold, result_params.boundary_threshold);
  EXPECT_FLOAT_EQ(params.discount, result_params.discount);
  EXPECT_EQ(params.grid_res, result_params.grid_res);
  EXPECT_FLOAT_EQ(params.r_c1.x, result_params.r_c1.x);
  EXPECT_FLOAT_EQ(params.r_c1.y, result_params.r_c1.y);
  EXPECT_FLOAT_EQ(params.r_c1.z, result_params.r_c1.z);
  EXPECT_FLOAT_EQ(params.r_c2.x, result_params.r_c2.x);
  EXPECT_FLOAT_EQ(params.r_c2.y, result_params.r_c2.y);
  EXPECT_FLOAT_EQ(params.r_c2.z, result_params.r_c2.z);
  EXPECT_FLOAT_EQ(params.trs.x, result_params.trs.x);
  EXPECT_FLOAT_EQ(params.trs.y, result_params.trs.y);
  EXPECT_FLOAT_EQ(params.trs.z, result_params.trs.z);
  // neither should be set by this sequence
  EXPECT_EQ(width, -1);
  EXPECT_EQ(height, -1);

  params.desired_speed = 5;
  params.r_c1.x = 4;
  params.r_c1.y = 5;
  params.r_c1.z = 6;
  cost.setParams(params);

  launchParameterTestKernel<>(cost, result_params, width, height);

  EXPECT_FLOAT_EQ(params.desired_speed, result_params.desired_speed);
  EXPECT_FLOAT_EQ(params.speed_coeff, result_params.speed_coeff);
  EXPECT_FLOAT_EQ(params.track_coeff, result_params.track_coeff);
  EXPECT_FLOAT_EQ(params.max_slip_ang, result_params.max_slip_ang);
  EXPECT_FLOAT_EQ(params.slip_coeff, result_params.slip_coeff);
  EXPECT_FLOAT_EQ(params.track_slop, result_params.track_slop);
  EXPECT_FLOAT_EQ(params.crash_coeff, result_params.crash_coeff);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[0], result_params.control_cost_coeff[0]);
  EXPECT_FLOAT_EQ(params.control_cost_coeff[1], result_params.control_cost_coeff[1]);
  EXPECT_FLOAT_EQ(params.boundary_threshold, result_params.boundary_threshold);
  EXPECT_FLOAT_EQ(params.discount, result_params.discount);
  EXPECT_EQ(params.grid_res, result_params.grid_res);
  EXPECT_FLOAT_EQ(params.r_c1.x, result_params.r_c1.x);
  EXPECT_FLOAT_EQ(params.r_c1.y, result_params.r_c1.y);
  EXPECT_FLOAT_EQ(params.r_c1.z, result_params.r_c1.z);
  EXPECT_FLOAT_EQ(params.r_c2.x, result_params.r_c2.x);
  EXPECT_FLOAT_EQ(params.r_c2.y, result_params.r_c2.y);
  EXPECT_FLOAT_EQ(params.r_c2.z, result_params.r_c2.z);
  EXPECT_FLOAT_EQ(params.trs.x, result_params.trs.x);
  EXPECT_FLOAT_EQ(params.trs.y, result_params.trs.y);
  EXPECT_FLOAT_EQ(params.trs.z, result_params.trs.z);

  // neither should be set by this sequence
  EXPECT_EQ(width, -1);
  EXPECT_EQ(height, -1);
}

TEST(ARStandardCost, coorTransformTest)
{
  float x, y, u, v, w;

  ARStandardCostParams params;
  ARStandardCost cost;

  x = 0;
  y = 10;

  params.r_c1.x = 0;
  params.r_c1.y = 1;
  params.r_c1.z = 2;
  params.r_c2.x = 3;
  params.r_c2.y = 4;
  params.r_c2.z = 5;
  params.trs.x = 6;
  params.trs.y = 7;
  params.trs.z = 8;
  cost.setParams(params);

  cost.coorTransform(x, y, &u, &v, &w);

  EXPECT_FLOAT_EQ(u, 36);
  EXPECT_FLOAT_EQ(v, 47);
  EXPECT_FLOAT_EQ(w, 58);
}

TEST(ARStandardCost, changeCostmapSizeTestValidInputs)
{
  ARStandardCost cost;
  cost.changeCostmapSize(4, 8);

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);

  std::vector<float4> result;

  // launchCheckCudaArray(result, cost.getCudaArray(), 4*8);

  // EXPECT_EQ(result.size(), 4*8);
  // TODO verify that cuda is properly allocating the memory
  /*
  for(int i = 0; i < 4*8; i++) {
    EXPECT_FLOAT_EQ(result[i].x, 0);
    EXPECT_FLOAT_EQ(result[i].y, 0);
    EXPECT_FLOAT_EQ(result[i].z, 0);
    EXPECT_FLOAT_EQ(result[i].w, 0);
  }
   */
}

TEST(ARStandardCost, changeCostmapSizeTestFail)
{
  ARStandardCost cost;
  cost.changeCostmapSize(4, 8);

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);

  testing::internal::CaptureStderr();
  cost.changeCostmapSize(-1, -1);
  std::string error_msg = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_msg, "");

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);
}

TEST(ARStandardCost, clearCostmapTest)
{
  ARStandardCost cost;
  cost.clearCostmapCPU(4, 8);

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);

  for (int i = 0; i < 4 * 8; i++)
  {
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).x, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).y, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).z, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).w, 0);
  }
}

TEST(ARStandardCost, clearCostmapTestDefault)
{
  ARStandardCost cost;
  cost.clearCostmapCPU(4, 8);

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);

  for (int i = 0; i < 4 * 8; i++)
  {
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).x, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).y, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).z, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).w, 0);
  }

  testing::internal::CaptureStderr();
  cost.clearCostmapCPU();
  std::string error_msg = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_msg, "");

  EXPECT_EQ(cost.getWidth(), 4);
  EXPECT_EQ(cost.getHeight(), 8);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 4 * 8);

  for (int i = 0; i < 4 * 8; i++)
  {
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).x, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).y, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).z, 0);
    EXPECT_FLOAT_EQ(cost.getTrackCostCPU().at(i).w, 0);
  }
}

TEST(ARStandardCost, clearCostmapTestDefaultFail)
{
  ARStandardCost cost;

  testing::internal::CaptureStderr();
  cost.clearCostmapCPU();
  std::string error_msg = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_msg, "");

  EXPECT_EQ(cost.getWidth(), -1);
  EXPECT_EQ(cost.getHeight(), -1);
  EXPECT_EQ(cost.getTrackCostCPU().capacity(), 0);
}

TEST(ARStandardCost, updateTransformCPUTest)
{
  ARStandardCost cost;
  Eigen::MatrixXf R(3, 3);
  Eigen::ArrayXf trs(3);
  R(0, 0) = 1;
  R(0, 1) = 2;
  R(0, 2) = 3;

  R(1, 0) = 4;
  R(1, 1) = 5;
  R(1, 2) = 6;

  R(2, 0) = 7;
  R(2, 1) = 8;
  R(2, 2) = 9;

  trs(0) = 10;
  trs(1) = 11;
  trs(2) = 12;

  cost.updateTransform(R, trs);

  /*
   * Prospective transform matrix
   * r_c1.x, r_c2.x, trs.x
   * r_c1.y, r_c2.y, trs.y
   * r_c1.z, r_c2.z, trs.z
   */

  EXPECT_FLOAT_EQ(cost.getParams().r_c1.x, 1);
  EXPECT_FLOAT_EQ(cost.getParams().r_c2.x, 2);
  EXPECT_FLOAT_EQ(cost.getParams().trs.x, 10);

  EXPECT_FLOAT_EQ(cost.getParams().r_c1.y, 4);
  EXPECT_FLOAT_EQ(cost.getParams().r_c2.y, 5);
  EXPECT_FLOAT_EQ(cost.getParams().trs.y, 11);

  EXPECT_FLOAT_EQ(cost.getParams().r_c1.z, 7);
  EXPECT_FLOAT_EQ(cost.getParams().r_c2.z, 8);
  EXPECT_FLOAT_EQ(cost.getParams().trs.z, 12);

  EXPECT_EQ(cost.GPUMemStatus_, false);
}

TEST(ARStandardCost, updateTransformGPUTest)
{
  ARStandardCost cost;
  cost.GPUSetup();
  Eigen::MatrixXf R(3, 3);
  Eigen::ArrayXf trs(3);
  R(0, 0) = 1;
  R(0, 1) = 2;
  R(0, 2) = 3;

  R(1, 0) = 4;
  R(1, 1) = 5;
  R(1, 2) = 6;

  R(2, 0) = 7;
  R(2, 1) = 8;
  R(2, 2) = 9;

  trs(0) = 10;
  trs(1) = 11;
  trs(2) = 12;

  cost.updateTransform(R, trs);

  /*
   * Prospective transform matrix
   * r_c1.x, r_c2.x, trs.x
   * r_c1.y, r_c2.y, trs.y
   * r_c1.z, r_c2.z, trs.z
   */

  std::vector<float3> results;
  launchTransformTestKernel<>(results, cost);

  EXPECT_EQ(results.size(), 3);

  EXPECT_FLOAT_EQ(results.at(0).x, 1);
  EXPECT_FLOAT_EQ(results.at(0).y, 4);
  EXPECT_FLOAT_EQ(results.at(0).z, 7);

  EXPECT_FLOAT_EQ(results.at(1).x, 2);
  EXPECT_FLOAT_EQ(results.at(1).y, 5);
  EXPECT_FLOAT_EQ(results.at(1).z, 8);

  EXPECT_FLOAT_EQ(results.at(2).x, 10);
  EXPECT_FLOAT_EQ(results.at(2).y, 11);
  EXPECT_FLOAT_EQ(results.at(2).z, 12);

  EXPECT_EQ(cost.GPUMemStatus_, true);
}

TEST(ARStandardCost, LoadTrackDataInvalidLocationTest)
{
  ARStandardCost cost;

  testing::internal::CaptureStderr();
  cost.loadTrackData("/null");
  std::string error_msg = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_msg, "");
  EXPECT_NE(error_msg.find("/null", 0), std::string::npos);
}

TEST(ARStandardCost, LoadTrackDataTest)
{
  ARStandardCost cost;
  // TODO set parameters for cost map
  cost.GPUSetup();

  // load
  std::vector<float4> costmap = cost.loadTrackData(mppi::tests::test_map_file);

  Eigen::Matrix3f R = cost.getRotation();
  Eigen::Array3f trs = cost.getTranslation();

  EXPECT_FLOAT_EQ(costmap.at(0).x, 1);
  EXPECT_FLOAT_EQ(costmap.at(0).y, 10);
  EXPECT_FLOAT_EQ(costmap.at(0).z, 100);
  EXPECT_FLOAT_EQ(costmap.at(0).w, 1000);
  EXPECT_FLOAT_EQ(costmap.at(1).x, 2);
  EXPECT_FLOAT_EQ(costmap.at(1).y, 20);
  EXPECT_FLOAT_EQ(costmap.at(1).z, 200);
  EXPECT_FLOAT_EQ(costmap.at(1).w, 2000);

  // check transformation, should not have a rotation
  EXPECT_FLOAT_EQ(R(0, 0), 1.0 / (10));
  EXPECT_FLOAT_EQ(R(1, 1), 1.0 / (20));
  EXPECT_FLOAT_EQ(R(2, 2), 1.0);

  EXPECT_FLOAT_EQ(R(0, 1), 0);
  EXPECT_FLOAT_EQ(R(0, 2), 0);
  EXPECT_FLOAT_EQ(R(1, 0), 0);
  EXPECT_FLOAT_EQ(R(1, 2), 0);
  EXPECT_FLOAT_EQ(R(2, 0), 0);
  EXPECT_FLOAT_EQ(R(2, 1), 0);

  EXPECT_FLOAT_EQ(trs(0), 0.5);
  EXPECT_FLOAT_EQ(trs(1), 0.5);
  EXPECT_FLOAT_EQ(trs(2), 1);

  // check on the GPU
  std::vector<float3> results;
  launchTransformTestKernel<>(results, cost);

  EXPECT_EQ(results.size(), 3);

  // check diag
  EXPECT_FLOAT_EQ(results.at(0).x, 1.0 / 10);
  EXPECT_FLOAT_EQ(results.at(1).y, 1.0 / (20));

  EXPECT_FLOAT_EQ(results.at(0).y, 0);
  EXPECT_FLOAT_EQ(results.at(0).z, 0);
  EXPECT_FLOAT_EQ(results.at(1).x, 0);
  EXPECT_FLOAT_EQ(results.at(1).z, 0);

  EXPECT_FLOAT_EQ(results.at(2).x, 0.5);
  EXPECT_FLOAT_EQ(results.at(2).y, 0.5);
  EXPECT_FLOAT_EQ(results.at(2).z, 1);

  int counter = 0;
  for (int i = 0; i < 2 * 10; i++)
  {
    for (int j = 0; j < 2 * 20; j++)
    {
      EXPECT_FLOAT_EQ(costmap.at(counter).x, counter + 1);
      EXPECT_FLOAT_EQ(costmap.at(counter).y, (counter + 1) * 10);
      EXPECT_FLOAT_EQ(costmap.at(counter).z, (counter + 1) * 100);
      EXPECT_FLOAT_EQ(costmap.at(counter).w, (counter + 1) * 1000);
      counter++;
    }
  }
}

TEST(ARStandardCost, costmapToTextureNoSizeTest)
{
  ARStandardCost cost;
  cost.GPUSetup();

  testing::internal::CaptureStderr();
  cost.costmapToTexture();
  std::string error_msg = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_msg, "");
}

TEST(ARStandardCost, costmapToTextureNoLoadTest)
{
  ARStandardCost cost;
  cost.GPUSetup();

  cost.changeCostmapSize(4, 8);

  cost.costmapToTexture();

  std::vector<float4> results;
  std::vector<float2> query_points;
  float2 point;
  point.x = 0.0f;
  point.y = 0.0f;
  query_points.push_back(point);
  point.x = 1.0f;
  point.y = 0.0f;
  query_points.push_back(point);

  launchTextureTestKernel<>(cost, results, query_points);

  EXPECT_EQ(query_points.size(), results.size());

  EXPECT_FLOAT_EQ(results.at(0).x, 0);
  EXPECT_FLOAT_EQ(results.at(0).y, 0);
  EXPECT_FLOAT_EQ(results.at(0).z, 0);
  EXPECT_FLOAT_EQ(results.at(0).w, 0);

  EXPECT_FLOAT_EQ(results.at(1).x, 0);
  EXPECT_FLOAT_EQ(results.at(1).y, 0);
  EXPECT_FLOAT_EQ(results.at(1).z, 0);
  EXPECT_FLOAT_EQ(results.at(1).w, 0);
}

TEST(ARStandardCost, costmapToTextureLoadTest)
{
  ARStandardCost cost;
  cost.GPUSetup();

  Eigen::Matrix3f R;
  Eigen::Array3f trs;

  std::vector<float4> costmap = cost.loadTrackData(mppi::tests::test_map_file);
  cost.costmapToTexture();

  std::vector<float4> results;
  std::vector<float2> query_points(8);
  query_points[0].x = 0;
  query_points[0].y = 0;
  query_points[1].x = 0.1;
  query_points[1].y = 0;
  query_points[2].x = 0.5;
  query_points[2].y = 0;
  query_points[3].x = 1.0;
  query_points[3].y = 0;
  query_points[4].x = 0.0;
  query_points[4].y = 0.5;
  query_points[5].x = 0.0;
  query_points[5].y = 1.0;
  query_points[6].x = 0.5;
  query_points[6].y = 0.5;
  query_points[7].x = 1.0;
  query_points[7].y = 1.0;

  launchTextureTestKernel<>(cost, results, query_points);

  EXPECT_EQ(query_points.size(), results.size());
  // 0,0
  EXPECT_FLOAT_EQ(results.at(0).x, 1);
  EXPECT_FLOAT_EQ(results.at(0).y, 10);
  EXPECT_FLOAT_EQ(results.at(0).z, 100);
  EXPECT_FLOAT_EQ(results.at(0).w, 1000);
  // 0.1, 0
  EXPECT_FLOAT_EQ(results.at(1).x, 2);
  EXPECT_FLOAT_EQ(results.at(1).y, 20);
  EXPECT_FLOAT_EQ(results.at(1).z, 200);
  EXPECT_FLOAT_EQ(results.at(1).w, 2000);
  // 0.5, 0
  EXPECT_FLOAT_EQ(results.at(2).x, 11);
  EXPECT_FLOAT_EQ(results.at(2).y, 110);
  EXPECT_FLOAT_EQ(results.at(2).z, 1100);
  EXPECT_FLOAT_EQ(results.at(2).w, 11000);
  // 1.0, 0
  EXPECT_FLOAT_EQ(results.at(3).x, 20);
  EXPECT_FLOAT_EQ(results.at(3).y, 200);
  EXPECT_FLOAT_EQ(results.at(3).z, 2000);
  EXPECT_FLOAT_EQ(results.at(3).w, 20000);
  // 0.0, 0.5
  EXPECT_FLOAT_EQ(results.at(4).x, 401);
  EXPECT_FLOAT_EQ(results.at(4).y, 4010);
  EXPECT_FLOAT_EQ(results.at(4).z, 40100);
  EXPECT_FLOAT_EQ(results.at(4).w, 401000);
  // 0.0, 1.0
  EXPECT_FLOAT_EQ(results.at(5).x, 781);
  EXPECT_FLOAT_EQ(results.at(5).y, 7810);
  EXPECT_FLOAT_EQ(results.at(5).z, 78100);
  EXPECT_FLOAT_EQ(results.at(5).w, 781000);
  // 0.5, 0.5
  EXPECT_FLOAT_EQ(results.at(6).x, 411);
  EXPECT_FLOAT_EQ(results.at(6).y, 4110);
  EXPECT_FLOAT_EQ(results.at(6).z, 41100);
  EXPECT_FLOAT_EQ(results.at(6).w, 411000);
  // 1,1
  EXPECT_FLOAT_EQ(results.at(7).x, 800);
  EXPECT_FLOAT_EQ(results.at(7).y, 8000);
  EXPECT_FLOAT_EQ(results.at(7).z, 80000);
  EXPECT_FLOAT_EQ(results.at(7).w, 800000);
}

TEST(ARStandardCost, costmapToTextureTransformTest)
{
  ARStandardCost cost;
  cost.GPUSetup();

  std::vector<float4> costmap = cost.loadTrackData(mppi::tests::test_map_file);
  cost.costmapToTexture();

  std::vector<float4> results;
  std::vector<float2> query_points(8);
  query_points[0].x = -5;
  query_points[0].y = -10;
  query_points[1].x = -4;
  query_points[1].y = -10;
  query_points[2].x = 0;
  query_points[2].y = -10;
  query_points[3].x = 5;
  query_points[3].y = -10;
  query_points[4].x = -5;
  query_points[4].y = 0;
  query_points[5].x = -5;
  query_points[5].y = 10;
  query_points[6].x = 0;
  query_points[6].y = 0;
  query_points[7].x = 5;
  query_points[7].y = 10;

  launchTextureTransformTestKernel<>(cost, results, query_points);

  EXPECT_EQ(query_points.size(), results.size());
  // 0,0
  EXPECT_FLOAT_EQ(results.at(0).x, 1);
  EXPECT_FLOAT_EQ(results.at(0).y, 10);
  EXPECT_FLOAT_EQ(results.at(0).z, 100);
  EXPECT_FLOAT_EQ(results.at(0).w, 1000);
  // 0.1, 0
  EXPECT_FLOAT_EQ(results.at(1).x, 2);
  EXPECT_FLOAT_EQ(results.at(1).y, 20);
  EXPECT_FLOAT_EQ(results.at(1).z, 200);
  EXPECT_FLOAT_EQ(results.at(1).w, 2000);
  // 0.5, 0
  EXPECT_FLOAT_EQ(results.at(2).x, 11);
  EXPECT_FLOAT_EQ(results.at(2).y, 110);
  EXPECT_FLOAT_EQ(results.at(2).z, 1100);
  EXPECT_FLOAT_EQ(results.at(2).w, 11000);
  // 1.0, 0
  EXPECT_FLOAT_EQ(results.at(3).x, 20);
  EXPECT_FLOAT_EQ(results.at(3).y, 200);
  EXPECT_FLOAT_EQ(results.at(3).z, 2000);
  EXPECT_FLOAT_EQ(results.at(3).w, 20000);
  // 0.0, 0.5
  EXPECT_FLOAT_EQ(results.at(4).x, 401);
  EXPECT_FLOAT_EQ(results.at(4).y, 4010);
  EXPECT_FLOAT_EQ(results.at(4).z, 40100);
  EXPECT_FLOAT_EQ(results.at(4).w, 401000);
  // 0.0, 1.0
  EXPECT_FLOAT_EQ(results.at(5).x, 781);
  EXPECT_FLOAT_EQ(results.at(5).y, 7810);
  EXPECT_FLOAT_EQ(results.at(5).z, 78100);
  EXPECT_FLOAT_EQ(results.at(5).w, 781000);
  // 0.5, 0.5
  EXPECT_FLOAT_EQ(results.at(6).x, 411);
  EXPECT_FLOAT_EQ(results.at(6).y, 4110);
  EXPECT_FLOAT_EQ(results.at(6).z, 41100);
  EXPECT_FLOAT_EQ(results.at(6).w, 411000);
  // 1,1
  EXPECT_FLOAT_EQ(results.at(7).x, 800);
  EXPECT_FLOAT_EQ(results.at(7).y, 8000);
  EXPECT_FLOAT_EQ(results.at(7).z, 80000);
  EXPECT_FLOAT_EQ(results.at(7).w, 800000);
}

TEST(ARStandardCost, TerminalCostTest)
{
  ARStandardCost cost;

  cost.GPUSetup();

  std::vector<std::array<float, 7>> states;

  std::array<float, 7> array = { 0.0 };
  array[0] = 3.0;     // X
  array[1] = 0.0;     // Y
  array[2] = M_PI_2;  // Theta
  array[3] = 0.0;     // Roll
  array[4] = 2.0;     // Vx
  array[5] = 1.0;     // Vy
  array[6] = 0.1;     // Yaw dot
  states.push_back(array);

  std::vector<float> cost_results;

  launchTerminalCostTestKernel<>(cost, states, cost_results);
  EXPECT_FLOAT_EQ(cost_results[0], 0.0);
}

TEST(ARStandardCost, getSpeedCostTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.desired_speed = 25;
  params.speed_coeff = 10;
  cost.setParams(params);

  float state[7];
  for (int i = 0; i < 7; i++)
  {
    state[i] = 0;
  }
  int crash = 0;
  state[4] = 10;

  float result = cost.getSpeedCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 15 * 15 * 10);

  params.desired_speed = 0;
  params.speed_coeff = 100;
  cost.setParams(params);

  result = cost.getSpeedCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 10 * 10 * 100);
}

TEST(ARStandardCost, getStablizingCostTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.slip_coeff = 25;
  params.crash_coeff = 1000;
  params.max_slip_ang = 0.5;
  cost.setParams(params);

  float state[7];
  for (int i = 0; i < 7; i++)
  {
    state[i] = 0;
  }
  state[4] = 0.1;
  state[5] = 0.0;
  int crash = 0;

  float result = cost.getStabilizingCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 0);
  EXPECT_EQ(crash, 0);

  state[5] = 0.01;

  result = cost.getStabilizingCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 0.2483460072);
  EXPECT_EQ(crash, 0);

  state[5] = 0.2;

  result = cost.getStabilizingCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 1030.6444);
  EXPECT_EQ(crash, 0);

  state[3] = 1.6;
  state[5] = 0.0;

  result = cost.getStabilizingCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 0.0);
  EXPECT_EQ(crash, 1);

  state[3] = -1.6;

  result = cost.getStabilizingCost(state, &crash);

  EXPECT_FLOAT_EQ(result, 0.0);
  EXPECT_EQ(crash, 1);
}

TEST(ARStandardCost, getCrashCostTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.crash_coeff = 10000;
  cost.setParams(params);

  float state[7];
  for (int i = 0; i < 7; i++)
  {
    state[i] = 0;
  }
  int crash = 0;
  state[4] = 10;

  float result = cost.getCrashCost(state, &crash, 10);

  EXPECT_FLOAT_EQ(result, 0);

  crash = 1;
  result = cost.getCrashCost(state, &crash, 10);

  EXPECT_FLOAT_EQ(result, 10000);
}

float calculateStandardCostmapValue(ARStandardCost& cost, float3 state, int width, int height, float x_min, float x_max,
                                    float y_min, float y_max, int ppm)
{
  float x_front = state.x + cost.FRONT_D * cosf(state.z);
  float y_front = state.y + cost.FRONT_D * sinf(state.z);
  float x_back = state.x + cost.BACK_D * cosf(state.z);
  float y_back = state.y + cost.BACK_D * sinf(state.z);

  float new_x = max(min(x_front - x_min, x_max - x_min), 0.0f);
  float new_y = max(min(y_front - y_min, y_max - y_min), 0.0f);

  float front = fabs(height / 2.0f - (new_y)) + (new_x) / width;
  // std::cout << "front point = " << new_x << ", " << new_y << " = " << front << std::endl;

  new_x = max(min(x_back - x_min + 1.0 / (width * ppm), x_max - x_min), 0.0f);
  new_y = max(min(y_back - y_min + 1.0 / (height * ppm), y_max - y_min), 0.0f);

  float back = fabs(height / 2.0f - (new_y)) + (new_x) / width;
  // std::cout << "back point = " << new_x << ", " << new_y << " = " << back << std::endl;
  return (front + back) / 2.0f;
}

TEST(ARStandardCost, getTrackCostTest)
{
  std::cout << "==========================\n\n" << std::endl;
  ARStandardCost cost;
  ARStandardCostParams params;
  params.track_coeff = 1;
  params.track_slop = 0.0;
  params.boundary_threshold = 1.0;
  cost.setParams(params);

  cost.GPUSetup();

  cost.loadTrackData(mppi::tests::standard_test_map_file);

  std::vector<float3> states(4);
  states[0].x = -13.5;
  states[0].y = -10;
  states[0].z = 0.0;  // theta
  states[1].x = 0;
  states[1].y = -10.0;
  states[1].z = 0.0;  // theta
  states[2].x = 0.0;
  states[2].y = 0.0;
  states[2].z = M_PI / 2;  // theta
  states[3].x = 3.0;
  states[3].y = 0.0;
  states[3].z = M_PI_2;  // theta

  std::vector<float> cost_results;
  std::vector<int> crash_results;

  launchTrackCostTestKernel<>(cost, states, cost_results, crash_results);

  EXPECT_NEAR(cost_results[0],
              params.track_coeff * calculateStandardCostmapValue(cost, states[0], 30, 30, -13, 17, -10, 20, 20), 0.001);
  EXPECT_FLOAT_EQ(crash_results[0], 1.0);
  EXPECT_NEAR(cost_results[1],
              params.track_coeff * calculateStandardCostmapValue(cost, states[1], 30, 30, -13, 17, -10, 20, 20), 0.001);
  EXPECT_FLOAT_EQ(crash_results[1], 1.0);
  EXPECT_NEAR(cost_results[2],
              params.track_coeff * calculateStandardCostmapValue(cost, states[2], 30, 30, -13, 17, -10, 20, 20), 0.1);
  EXPECT_FLOAT_EQ(crash_results[2], 1.0);
  EXPECT_NEAR(cost_results[3],
              params.track_coeff * calculateStandardCostmapValue(cost, states[3], 30, 30, -13, 17, -10, 20, 20), 0.1);
  EXPECT_FLOAT_EQ(crash_results[3], 1.0);
}

TEST(ARStandardCost, computeCostIndividualTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.track_coeff = 0;
  params.speed_coeff = 0;
  params.crash_coeff = 0.0;
  params.slip_coeff = 0.0;
  params.control_cost_coeff[0] = 0.0;
  params.control_cost_coeff[1] = 0.0;
  params.discount = 0.9;
  cost.setParams(params);

  cost.GPUSetup();

  cost.loadTrackData(mppi::tests::standard_test_map_file);
  params = cost.getParams();

  std::vector<std::array<float, 9>> states;

  std::array<float, 9> array = { 0.0 };
  array[0] = 3.0;     // X
  array[1] = 0.0;     // Y
  array[2] = M_PI_2;  // Theta
  array[3] = 0.0;     // Roll
  array[4] = 2.0;     // Vx
  array[5] = 1.0;     // Vy
  array[6] = 0.1;     // Yaw dot
  array[7] = 0.5;     // steering
  array[8] = 0.3;     // throttle
  states.push_back(array);

  std::vector<float> cost_results;
  std::vector<int> timestep;
  timestep.push_back(1);
  std::vector<int> crash;
  crash.push_back(0);

  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], 0.0);

  params.speed_coeff = 4.25;
  cost.setParams(params);

  float speed_cost = powf(4.0, 2) * 4.25;
  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], speed_cost);

  params.speed_coeff = 0.0;
  params.slip_coeff = 10;
  cost.setParams(params);

  float slip_cost = powf(-atanf(1.0 / 2.0), 2) * 10;
  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], slip_cost);

  params.slip_coeff = 0.0;
  params.track_coeff = 200.0;
  cost.setParams(params);

  float track_cost = 1116.3333;
  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], track_cost);

  params.track_coeff = 0.0;
  params.crash_coeff = 10000;
  cost.setParams(params);

  float crash_cost = 9000;
  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], crash_cost);

  params.speed_coeff = 4.25;
  params.track_coeff = 200;
  params.slip_coeff = 10;
  params.crash_coeff = 10000;
  cost.setParams(params);

  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], speed_cost + slip_cost + track_cost + crash_cost);

  timestep[0] = 4;

  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], speed_cost + slip_cost + track_cost + powf(0.9, timestep[0]) * params.crash_coeff);
}

TEST(ARStandardCost, computeCostOverflowTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.track_coeff = 0;
  params.speed_coeff = 10;
  params.crash_coeff = 0.0;
  params.slip_coeff = 0.0;
  params.control_cost_coeff[0] = 0.0;
  params.control_cost_coeff[1] = 0.0;
  params.desired_speed = ARStandardCost::MAX_COST_VALUE;
  cost.setParams(params);

  cost.GPUSetup();

  cost.loadTrackData(mppi::tests::standard_test_map_file);
  params = cost.getParams();

  std::vector<std::array<float, 9>> states;

  std::array<float, 9> array = { 0.0 };
  array[0] = 3.0;     // X
  array[1] = 0.0;     // Y
  array[2] = M_PI_2;  // Theta
  array[3] = 0.0;     // Roll
  array[4] = 2.0;     // Vx
  array[5] = 1.0;     // Vy
  array[6] = 0.1;     // Yaw dot
  array[7] = 0.5;     // steering
  array[8] = 0.3;     // throttle
  states.push_back(array);

  std::vector<float> cost_results;

  std::vector<int> timestep;
  timestep.push_back(1);
  std::vector<int> crash;
  crash.push_back(0);

  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], ARStandardCost::MAX_COST_VALUE);

  cost_results[0] = 0;

  params.desired_speed = NAN;
  cost.setParams(params);
  launchComputeCostTestKernel<>(cost, states, cost_results, timestep, crash);
  EXPECT_FLOAT_EQ(cost_results[0], ARStandardCost::MAX_COST_VALUE);
}

TEST(ARStandardCost, matchAutoRallyTest)
{
  ARStandardCost cost;
  ARStandardCostParams params;
  params.desired_speed = 6.0;
  params.track_coeff = 200;
  params.speed_coeff = 4.25;
  params.crash_coeff = 10000.0;
  params.max_slip_ang = 1.25;
  params.slip_coeff = 10.0;
  params.track_slop = 0.0;
  params.boundary_threshold = 0.65;
  params.control_cost_coeff[0] = 0.0;
  params.control_cost_coeff[1] = 0.0;
  params.discount = 0.9;
  cost.setParams(params);

  cost.GPUSetup();

  cost.loadTrackData(mppi::tests::ccrf_map);

  params = cost.getParams();

  std::vector<std::array<float, 9>> states;
  std::vector<int> timesteps;
  std::vector<int> crash;

  std::array<float, 9> array = { 0.0 };

  // input state 19.113796 -36.497066 7.431655 0.055569 4.922609 0.069374 -1.785045
  // input control -0.134569 -0.485720
  // timestep 1
  // crash = 0
  // cost returned 122.039955
  array = { 19.113796, -36.497066, 7.431655, 0.055569, 4.922609, 0.069374, -1.785045, -0.134569, -0.485720 };
  states.push_back(array);
  timesteps.push_back(1);
  crash.push_back(0);

  // input state 19.076693 -36.588978 7.397079 0.066307 4.951416 0.068694 -1.694298
  // input control 0.183665 -0.441535
  // timestep 1
  // cost returned 128.190033
  // crash = 0
  array = { 19.076693, -36.588978, 7.397079, 0.066307, 4.951416, 0.068694, -1.694298, 0.183665, -0.441535 };
  states.push_back(array);
  timesteps.push_back(1);
  crash.push_back(0);

  // input state -1.470056 -13.121619 1.015522 -0.076247 5.938981 0.375569 2.230900
  // input control 0.273864 -0.176446
  // timestep, crash 1 0
  // cost returned 9137.753906
  array = { -1.470056, -13.121619, 1.015522, -0.076247, 5.938981, 0.375569, 2.230900, 0.273864, -0.176446 };
  states.push_back(array);
  timesteps.push_back(1);
  crash.push_back(0);

  // input state -1.413821 -13.016726 0.970904 -0.079068 5.893999 0.607982 2.259659
  // input control 0.225313 -0.035700
  // timestep, crash 2 1
  // cost returned 8402.457031
  array = { -1.413821, -13.016726, 0.970904, -0.079068, 5.893999, 0.607982, 2.259659, 0.225313, -0.035700 };
  states.push_back(array);
  timesteps.push_back(2);
  crash.push_back(1);

  // input state 7.273710 -10.255844 -0.030983 -0.067543 4.353069 -0.036942 -0.177069
  // input control -0.038980 -0.335935
  // timestep, crash 99 1
  // cost returned 2857.133545
  array = { 7.273710, -10.255844, -0.030983, -0.067543, 4.353069, -0.036942, -0.177069, -0.038980, -0.335935 };
  states.push_back(array);
  timesteps.push_back(99);
  crash.push_back(1);

  std::vector<float> cost_results;

  launchComputeCostTestKernel<>(cost, states, cost_results, timesteps, crash);
  EXPECT_FLOAT_EQ(cost_results[0], 122.039955);
  EXPECT_EQ(crash[0], 0);
  EXPECT_FLOAT_EQ(cost_results[1], 128.190033);
  EXPECT_EQ(crash[1], 0);
  EXPECT_FLOAT_EQ(cost_results[2], 9137.753906);
  EXPECT_EQ(crash[2], 1);
  EXPECT_FLOAT_EQ(cost_results[3], 8402.457031);
  EXPECT_EQ(crash[3], 1);
  EXPECT_FLOAT_EQ(cost_results[4], 2857.133545);
  EXPECT_EQ(crash[4], 1);
}
