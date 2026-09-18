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

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInfer.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::TensorInfo;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

constexpr bool kOptional = true;
constexpr std::int64_t kWidth = 4;

// `dummy_network.onnx` declares three inputs of shape [num_rows, 4] that all feed its single
// output, so TensorRT declares every one of them.
const nvinfer1::Dims kShape{2, {-1, kWidth}};
const nvinfer1::Dims kMin{2, {1, kWidth}};
const nvinfer1::Dims kOpt{2, {8, kWidth}};
const nvinfer1::Dims kMax{2, {16, kWidth}};

constexpr const char * kOnnxPath = DUMMY_ONNX_PATH;

}  // namespace

// The tensor metadata carries the optional flag without any TensorRT involvement, so these run
// everywhere.

TEST(TensorInfoTest, TensorsAreRequiredByDefault)
{
  EXPECT_FALSE(TensorInfo("tensor").optional);
  EXPECT_FALSE(NetworkIO("tensor", kShape).optional);
  EXPECT_FALSE(ProfileDims("tensor", kMin, kOpt, kMax).optional);
}

TEST(TensorInfoTest, DerivedConstructorsCarryTheOptionalFlag)
{
  const NetworkIO network_io("tensor", kShape, std::nullopt, kOptional);
  EXPECT_TRUE(network_io.optional);
  EXPECT_EQ(network_io.tensor_name, "tensor");

  const ProfileDims profile_dims("tensor", kMin, kOpt, kMax, kOptional);
  EXPECT_TRUE(profile_dims.optional);
  EXPECT_EQ(profile_dims.tensor_name, "tensor");
}

TEST(TensorInfoTest, IndexAddressedTensorsAreRequiredAndUnnamed)
{
  // Optionality is name-addressed only: an index cannot be resolved against a model that does
  // not declare the tensor, so the index constructors deliberately offer no way to opt in.
  const NetworkIO network_io(0, kShape);
  EXPECT_FALSE(network_io.optional);
  EXPECT_TRUE(network_io.tensor_name.empty());
  EXPECT_EQ(network_io.tensor_index, 0);
}

/// Exercises IO reconciliation against the dummy network. Building an engine needs a CUDA device,
/// so every case skips where there is none.
class TrtCommonIOTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // SetUp is overridden rather than inheriting autoware::cuda_utils::CudaTest, so the macro has
    // to run here, before anything touches the device.
    SKIP_TEST_IF_CUDA_UNAVAILABLE();

    if (!std::filesystem::exists(kOnnxPath)) {
      GTEST_SKIP() << "Dummy network not found: " << kOnnxPath;
    }
    work_dir_ = std::filesystem::temp_directory_path() /
                ("autoware_tensorrt_common_test_" +
                 std::string(::testing::UnitTest::GetInstance()->current_test_info()->name()));
    std::filesystem::create_directories(work_dir_);
  }

  void TearDown() override
  {
    std::error_code ignored;
    std::filesystem::remove_all(work_dir_, ignored);
  }

  /// A fresh instance per case, each with its own engine path so no case loads another's engine.
  std::unique_ptr<TrtCommon> makeTrt(const std::string & tag)
  {
    return std::make_unique<TrtCommon>(
      TrtCommonConfig(kOnnxPath, "fp32", enginePath(tag), 1ULL << 26U));
  }

  [[nodiscard]] std::string enginePath(const std::string & tag) const
  {
    return (work_dir_ / (tag + ".engine")).string();
  }

  /// Everything the dummy network declares, all of it required.
  static std::vector<NetworkIO> declaredNetworkIO()
  {
    return {
      NetworkIO("required_a", kShape), NetworkIO("required_b", kShape),
      NetworkIO("optional_present", kShape), NetworkIO("sum", kShape)};
  }

  static std::vector<ProfileDims> declaredProfileDims()
  {
    return {
      ProfileDims("required_a", kMin, kOpt, kMax), ProfileDims("required_b", kMin, kOpt, kMax),
      ProfileDims("optional_present", kMin, kOpt, kMax)};
  }

  static bool setup(TrtCommon & trt, std::vector<ProfileDims> profile, std::vector<NetworkIO> io)
  {
    return trt.setup(
      std::make_unique<std::vector<ProfileDims>>(std::move(profile)),
      std::make_unique<std::vector<NetworkIO>>(std::move(io)));
  }

  std::filesystem::path work_dir_;
};

TEST_F(TrtCommonIOTest, ModelDeclaringEveryOfferedTensorIsAccepted)
{
  auto trt = makeTrt("all_declared");
  EXPECT_TRUE(setup(*trt, declaredProfileDims(), declaredNetworkIO()));
}

TEST_F(TrtCommonIOTest, UndeclaredOptionalTensorIsDropped)
{
  auto profile = declaredProfileDims();
  auto io = declaredNetworkIO();
  profile.emplace_back("optional_absent", kMin, kOpt, kMax, kOptional);
  io.emplace_back("optional_absent", kShape, std::nullopt, kOptional);

  auto trt = makeTrt("optional_absent");
  ASSERT_TRUE(setup(*trt, std::move(profile), std::move(io)));

  // The caller may keep binding the dropped tensor unconditionally; both setters no-op for it.
  EXPECT_TRUE(trt->setTensorAddress("optional_absent", nullptr));
  EXPECT_TRUE(trt->setInputShape("optional_absent", kOpt));

  // A name that was never offered is still a caller mistake, not a silent no-op.
  EXPECT_FALSE(trt->setInputShape("never_offered", kOpt));
}

TEST_F(TrtCommonIOTest, OptionalTensorOfferedOnlyByTheProfileIsDropped)
{
  // Both offers are reconciled, so an optional entry the caller lists solely as a profile
  // dimension is dropped and recorded just like one that also appears in `network_io`.
  auto profile = declaredProfileDims();
  profile.emplace_back("optional_absent", kMin, kOpt, kMax, kOptional);

  auto trt = makeTrt("profile_only_optional");
  ASSERT_TRUE(setup(*trt, std::move(profile), declaredNetworkIO()));

  EXPECT_TRUE(trt->setInputShape("optional_absent", kOpt));
}

TEST_F(TrtCommonIOTest, ProfileOnlyOfferStillDropsAbsentOptionalTensors)
{
  // Offering no `network_io` at all waives the account of the model's IO, which must not also
  // waive reconciliation of the profile's own optional entries.
  auto profile = declaredProfileDims();
  profile.emplace_back("optional_absent", kMin, kOpt, kMax, kOptional);

  auto trt = makeTrt("profile_only_no_io");
  ASSERT_TRUE(trt->setup(std::make_unique<std::vector<ProfileDims>>(std::move(profile))));

  EXPECT_TRUE(trt->setInputShape("optional_absent", kOpt));
  EXPECT_TRUE(trt->setInputShape("required_a", kOpt));
}

TEST_F(TrtCommonIOTest, UndeclaredRequiredTensorIsRejectedBeforeBuildingAnEngine)
{
  auto profile = declaredProfileDims();
  auto io = declaredNetworkIO();
  profile.emplace_back("required_missing", kMin, kOpt, kMax);
  io.emplace_back("required_missing", kShape);

  auto trt = makeTrt("required_missing");
  EXPECT_FALSE(setup(*trt, std::move(profile), std::move(io)));
  EXPECT_FALSE(std::filesystem::exists(enginePath("required_missing")))
    << "a model that cannot be served should be refused before an engine is built for it";
}

TEST_F(TrtCommonIOTest, RequiredTensorIsRejectedEvenWhenOnlyTheProfileOffersIt)
{
  auto profile = declaredProfileDims();
  profile.emplace_back("required_missing", kMin, kOpt, kMax);

  auto trt = makeTrt("profile_only");
  EXPECT_FALSE(setup(*trt, std::move(profile), declaredNetworkIO()));
}

TEST_F(TrtCommonIOTest, TensorIndexOutsideTheModelThrows)
{
  auto profile = declaredProfileDims();
  profile.emplace_back(99, kMin, kOpt, kMax);

  auto trt = makeTrt("bad_index");
  EXPECT_THROW(setup(*trt, std::move(profile), declaredNetworkIO()), std::invalid_argument);
}

TEST_F(TrtCommonIOTest, ModelDeclaringATensorThatWasNotOfferedIsRejected)
{
  // Offering a strict subset is not the same as offering optional entries: the caller has not
  // said the model may omit `optional_present`, it has failed to account for it.
  std::vector<NetworkIO> io{
    NetworkIO("required_a", kShape), NetworkIO("required_b", kShape), NetworkIO("sum", kShape)};
  std::vector<ProfileDims> profile{
    ProfileDims("required_a", kMin, kOpt, kMax), ProfileDims("required_b", kMin, kOpt, kMax)};

  auto trt = makeTrt("subset");
  EXPECT_FALSE(setup(*trt, std::move(profile), std::move(io)));
  EXPECT_FALSE(std::filesystem::exists(enginePath("subset")))
    << "a model the caller cannot fully bind should be refused before an engine is built for it";
}

TEST_F(TrtCommonIOTest, ASurplusOfferDoesNotExcuseAnUnaccountedTensor)
{
  // The counts match here, so a size comparison alone would let this through and leave
  // `optional_present` unbound at inference time.
  std::vector<NetworkIO> io{
    NetworkIO("required_a", kShape), NetworkIO("required_b", kShape), NetworkIO("sum", kShape),
    NetworkIO("optional_surplus", kShape, std::nullopt, kOptional)};
  std::vector<ProfileDims> profile{
    ProfileDims("required_a", kMin, kOpt, kMax), ProfileDims("required_b", kMin, kOpt, kMax)};

  auto trt = makeTrt("cancelling_counts");
  EXPECT_FALSE(setup(*trt, std::move(profile), std::move(io)));
}

TEST_F(TrtCommonIOTest, AnOfferOfNothingButAbsentOptionalTensorsIsStillAnAccount)
{
  // Reconciliation drops every entry, leaving `network_io` empty. That must not read as a caller
  // which never offered one: this caller did, and it accounts for none of the model's tensors.
  // The profile is complete, so the reconciliation is the only thing that can reject this.
  std::vector<NetworkIO> io{NetworkIO("optional_absent", kShape, std::nullopt, kOptional)};

  auto trt = makeTrt("all_optional_absent");
  EXPECT_FALSE(setup(*trt, declaredProfileDims(), std::move(io)));
  EXPECT_FALSE(std::filesystem::exists(enginePath("all_optional_absent")))
    << "a model the caller cannot fully bind should be refused before an engine is built for it";
}
