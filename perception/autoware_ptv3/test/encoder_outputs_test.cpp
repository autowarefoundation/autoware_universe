// Copyright 2026 TIER IV, Inc.
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

#include "autoware/ptv3/encoder_outputs.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
namespace
{

using tensorrt_common::NetworkIO;
using tensorrt_common::ProfileDims;
using tensorrt_common::TrtCommon;
using tensorrt_common::TrtCommonConfig;

constexpr std::int64_t kWidth = 4;
const nvinfer1::Dims kRows{2, {-1, kWidth}};
const std::vector<ProfileDims> kEncoderProfile{ProfileDims(
  "rows", nvinfer1::Dims{2, {1, kWidth}}, nvinfer1::Dims{2, {8, kWidth}},
  nvinfer1::Dims{2, {16, kWidth}})};

/// Builds the networks `test/data/generate_encoder_outputs_networks.py` writes. Building an engine
/// needs a CUDA device, so every case skips where there is none.
class EncoderOutputsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    SKIP_TEST_IF_CUDA_UNAVAILABLE();
    work_dir_ = std::filesystem::temp_directory_path() /
                ("encoder_outputs_test_" +
                 std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    std::filesystem::create_directories(work_dir_);
  }

  void TearDown() override
  {
    std::error_code ignored;
    std::filesystem::remove_all(work_dir_, ignored);
  }

  [[nodiscard]] std::unique_ptr<TrtCommon> makeTrt(const std::string & network) const
  {
    return std::make_unique<TrtCommon>(TrtCommonConfig(
      std::string(TEST_DATA_DIR) + "/encoder_outputs_" + network + ".onnx", "fp32",
      (work_dir_ / (network + ".engine")).string(), 1ULL << 26U));
  }

  /// An encoder engine set up with every output offered.
  [[nodiscard]] std::unique_ptr<TrtCommon> setUpEncoder(const std::string & network) const
  {
    auto encoder = makeTrt(network);
    std::vector<NetworkIO> network_io{NetworkIO("rows", kRows)};
    EncoderOutputs::offerOutputs(*encoder, network_io);
    EXPECT_TRUE(encoder->setup(
      std::make_unique<std::vector<ProfileDims>>(kEncoderProfile),
      std::make_unique<std::vector<NetworkIO>>(std::move(network_io))));
    return encoder;
  }

  std::filesystem::path work_dir_;
};

TEST_F(EncoderOutputsTest, HeadReadsTheEncoderOutputsOfTheSameName)
{
  auto encoder = setUpEncoder("encoder");
  EncoderOutputs outputs;
  outputs.allocate(*encoder, kEncoderProfile);

  auto head = makeTrt("head");
  std::vector<NetworkIO> network_io;
  std::vector<ProfileDims> profile_dims;
  const auto names = outputs.offerInputs(*head, network_io, profile_dims);
  ASSERT_EQ(names, (std::vector<std::string>{"feat", "doubled"}));
  for (const auto & io : network_io) {
    EXPECT_EQ(io.dtype, nvinfer1::DataType::kFLOAT) << io.tensor_name;
  }
  // The head's profile is the encoder's, resolved through the encoder: `doubled` has twice the
  // rows.
  ASSERT_EQ(profile_dims.size(), 2U);
  EXPECT_EQ(profile_dims[1].min_dims.d[0], 2);
  EXPECT_EQ(profile_dims[1].opt_dims.d[0], 16);
  EXPECT_EQ(profile_dims[1].max_dims.d[0], 32);

  network_io.emplace_back("out", kRows);
  ASSERT_TRUE(head->setup(
    std::make_unique<std::vector<ProfileDims>>(profile_dims),
    std::make_unique<std::vector<NetworkIO>>(network_io)));
  outputs.bind(*head, names);

  constexpr std::int64_t kNumRows = 5;
  std::vector<float> rows(kNumRows * kWidth);
  for (std::size_t i = 0; i < rows.size(); ++i) {
    rows[i] = static_cast<float>(i) - 7.0F;
  }
  auto rows_d = cuda_utils::make_unique<float[]>(rows.size());
  auto out_d = cuda_utils::make_unique<float[]>(3 * rows.size());
  CHECK_CUDA_ERROR(
    cudaMemcpy(rows_d.get(), rows.data(), rows.size() * sizeof(float), cudaMemcpyHostToDevice));
  cudaStream_t stream{};
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream));

  ASSERT_TRUE(encoder->setTensorAddress("rows", rows_d.get()));
  ASSERT_TRUE(encoder->setInputShape("rows", nvinfer1::Dims{2, {kNumRows, kWidth}}));
  ASSERT_TRUE(encoder->enqueueV3(stream));
  ASSERT_TRUE(head->setTensorAddress("out", out_d.get()));
  ASSERT_TRUE(outputs.setInputShapes(*encoder, *head, names));
  ASSERT_TRUE(head->enqueueV3(stream));

  std::vector<float> out(3 * rows.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out.data(), out_d.get(), out.size() * sizeof(float), cudaMemcpyDeviceToHost, stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  CHECK_CUDA_ERROR(cudaStreamDestroy(stream));

  for (std::size_t i = 0; i < rows.size(); ++i) {
    EXPECT_EQ(out[i], std::max(rows[i], 0.0F)) << i;
    EXPECT_EQ(out[rows.size() + i], rows[i]) << i;
    EXPECT_EQ(out[2 * rows.size() + i], rows[i]) << i;
  }
}

TEST_F(EncoderOutputsTest, DataDependentOutputIsRejected)
{
  // The rows of `nonzero` are known only after the encoder ran, too late to size its buffer.
  auto encoder = setUpEncoder("nonzero");
  EncoderOutputs outputs;
  EXPECT_THROW(outputs.allocate(*encoder, kEncoderProfile), std::runtime_error);
}

}  // namespace
}  // namespace autoware::ptv3
