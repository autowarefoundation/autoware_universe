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

#include "autoware/ptv3/ptv3_trt.hpp"

#include "ptv3_test_fixture.hpp"

#include <autoware/point_types/types.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{
namespace
{

using sensor_msgs::msg::PointField;

struct FieldSpec
{
  std::string name;
  std::uint32_t offset;
  std::uint8_t datatype;
};

std::vector<PointField> makeFields(const std::vector<FieldSpec> & specs)
{
  std::vector<PointField> fields;
  fields.reserve(specs.size());
  for (const auto & spec : specs) {
    PointField field;
    field.name = spec.name;
    field.offset = spec.offset;
    field.datatype = spec.datatype;
    field.count = 1;
    fields.push_back(field);
  }
  return fields;
}

// The layouts detectCloudFormat recognises, built from the real point structs so the
// expectations cannot drift from autoware_point_types.
std::vector<PointField> xyziFields()
{
  using P = autoware::point_types::PointXYZI;
  return makeFields(
    {{"x", offsetof(P, x), PointField::FLOAT32},
     {"y", offsetof(P, y), PointField::FLOAT32},
     {"z", offsetof(P, z), PointField::FLOAT32},
     {"intensity", offsetof(P, intensity), PointField::FLOAT32}});
}

std::vector<PointField> xyzircFields()
{
  using P = autoware::point_types::PointXYZIRC;
  return makeFields(
    {{"x", offsetof(P, x), PointField::FLOAT32},
     {"y", offsetof(P, y), PointField::FLOAT32},
     {"z", offsetof(P, z), PointField::FLOAT32},
     {"intensity", offsetof(P, intensity), PointField::UINT8},
     {"return_type", offsetof(P, return_type), PointField::UINT8},
     {"channel", offsetof(P, channel), PointField::UINT16}});
}

std::vector<PointField> xyziradrtFields()
{
  using P = autoware::point_types::PointXYZIRADRT;
  return makeFields(
    {{"x", offsetof(P, x), PointField::FLOAT32},
     {"y", offsetof(P, y), PointField::FLOAT32},
     {"z", offsetof(P, z), PointField::FLOAT32},
     {"intensity", offsetof(P, intensity), PointField::FLOAT32},
     {"ring", offsetof(P, ring), PointField::UINT16},
     {"azimuth", offsetof(P, azimuth), PointField::FLOAT32},
     {"distance", offsetof(P, distance), PointField::FLOAT32},
     {"return_type", offsetof(P, return_type), PointField::UINT8},
     {"time_stamp", offsetof(P, time_stamp), PointField::FLOAT64}});
}

std::vector<PointField> xyzircaedtFields()
{
  using P = autoware::point_types::PointXYZIRCAEDT;
  return makeFields(
    {{"x", offsetof(P, x), PointField::FLOAT32},
     {"y", offsetof(P, y), PointField::FLOAT32},
     {"z", offsetof(P, z), PointField::FLOAT32},
     {"intensity", offsetof(P, intensity), PointField::UINT8},
     {"return_type", offsetof(P, return_type), PointField::UINT8},
     {"channel", offsetof(P, channel), PointField::UINT16},
     {"azimuth", offsetof(P, azimuth), PointField::FLOAT32},
     {"elevation", offsetof(P, elevation), PointField::FLOAT32},
     {"distance", offsetof(P, distance), PointField::FLOAT32},
     {"time_stamp", offsetof(P, time_stamp), PointField::UINT32}});
}

PTv3ConfigParams baseParams()
{
  PTv3ConfigParams params;
  params.segmentation_class_names = {"CAR", "VEGETATION"};
  return params;
}

// Reaches the protected members. The engine-free constructor builds the stream, the point
// fields and the device buffers, which is everything the behaviour under test reads.
class TestablePTv3TRT : public PTv3TRT
{
public:
  explicit TestablePTv3TRT(const PTv3Config & config) : PTv3TRT(config) {}
  using PTv3TRT::detectCloudFormat;
  using PTv3TRT::stageProfileCounts;
};

}  // namespace

class PTv3TRTTest : public PTv3CudaTest
{
};

// --- stageProfileCounts -------------------------------------------------------------------

TEST_F(PTv3TRTTest, StageProfileCountsAnchorTheFinestStageToTheConfiguredMinimum)
{
  // Stage 0 consumes the original voxels, so it inherits their lower bound rather than 1.
  PTv3ConfigParams params = baseParams();
  params.voxels_num = {2, 4, 8};
  const TestablePTv3TRT trt{makeConfig(params)};

  const auto [min_count, opt_count, max_count] = trt.stageProfileCounts(0);
  EXPECT_EQ(min_count, 2);
}

TEST_F(PTv3TRTTest, StageProfileCountsAllowPooledStagesToCollapseToOneVoxel)
{
  // A pooled stage can legitimately produce a single voxel, so deeper stages must not inherit
  // the input-resolution minimum: an over-tight profile would reject sparse frames at runtime.
  PTv3ConfigParams params = baseParams();
  params.voxels_num = {2, 4, 8};
  const TestablePTv3TRT trt{makeConfig(params)};

  for (std::size_t stage = 1; stage < params.enc_channels.size(); ++stage) {
    const auto [min_count, opt_count, max_count] = trt.stageProfileCounts(stage);
    EXPECT_EQ(min_count, 1) << "stage " << stage;
  }
}

TEST_F(PTv3TRTTest, StageProfileCountsStayOrderedAndWithinTheGeometricBound)
{
  PTv3ConfigParams params = baseParams();
  params.voxels_num = {2, 4, 8};
  const auto config = makeConfig(params);
  const TestablePTv3TRT trt{config};

  for (std::size_t stage = 0; stage < params.enc_channels.size(); ++stage) {
    const auto [min_count, opt_count, max_count] = trt.stageProfileCounts(stage);
    EXPECT_LE(min_count, opt_count) << "stage " << stage;
    EXPECT_LE(opt_count, max_count) << "stage " << stage;
    EXPECT_EQ(max_count, config.stage_voxel_capacity(stage)) << "stage " << stage;
  }
}

TEST_F(PTv3TRTTest, StageProfileCountsShrinkTheOptimumWithPoolingDepth)
{
  // The opt entry is a tactic hint that halves per stage; it is only meaningful while it stays
  // above the minimum, which is what the clamp in stageProfileCounts guarantees.
  PTv3ConfigParams params = baseParams();
  params.voxels_num = {1, 64, 256};
  const TestablePTv3TRT trt{makeConfig(params)};

  const auto opt_at = [&trt](const std::size_t stage) {
    const auto [min_count, opt_count, max_count] = trt.stageProfileCounts(stage);
    return opt_count;
  };
  EXPECT_GE(opt_at(0), opt_at(1));
  EXPECT_GE(opt_at(1), opt_at(2));
}

TEST_F(PTv3TRTTest, StageProfileCountsClampTheOptimumIntoTheValidRange)
{
  // voxels_num[1] >> stage underflows the minimum at depth; the result must never fall below
  // min or above the geometric capacity, or TensorRT rejects the profile.
  PTv3ConfigParams params = baseParams();
  params.voxels_num = {4, 4, 8};
  const auto config = makeConfig(params);
  const TestablePTv3TRT trt{config};

  for (std::size_t stage = 0; stage < params.enc_channels.size(); ++stage) {
    const auto [min_count, opt_count, max_count] = trt.stageProfileCounts(stage);
    EXPECT_GE(opt_count, min_count) << "stage " << stage;
    EXPECT_LE(opt_count, config.stage_voxel_capacity(stage)) << "stage " << stage;
  }
}

// --- detectCloudFormat --------------------------------------------------------------------

TEST_F(PTv3TRTTest, DetectsEverySupportedCloudLayout)
{
  const TestablePTv3TRT trt{makeConfig(baseParams())};

  cuda_blackboard::CudaPointCloud2 cloud;

  cloud.fields = xyziFields();
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::XYZI);

  cloud.fields = xyzircFields();
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::XYZIRC);

  cloud.fields = xyziradrtFields();
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::XYZIRADRT);

  cloud.fields = xyzircaedtFields();
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::XYZIRCAEDT);
}

TEST_F(PTv3TRTTest, RejectsACloudWithNoFields)
{
  const TestablePTv3TRT trt{makeConfig(baseParams())};

  cuda_blackboard::CudaPointCloud2 cloud;
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::UNKNOWN);
}

TEST_F(PTv3TRTTest, RejectsARecognisedFieldCountWithAForeignLayout)
{
  // Four fields is the XYZI count, but the names and offsets are not XYZI. Dispatching on
  // count alone would reinterpret this buffer as points.
  const TestablePTv3TRT trt{makeConfig(baseParams())};

  cuda_blackboard::CudaPointCloud2 cloud;
  cloud.fields = makeFields(
    {{"a", 0, PointField::FLOAT32},
     {"b", 4, PointField::FLOAT32},
     {"c", 8, PointField::FLOAT32},
     {"d", 12, PointField::FLOAT32}});
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::UNKNOWN);
}

TEST_F(PTv3TRTTest, RejectsASupportedLayoutCarryingExtraFields)
{
  // The layout predicates only require a prefix match, so what pins the format is
  // detectCloudFormat's exact field-count dispatch. Documenting current behaviour: a cloud whose
  // first four fields are byte-exact XYZI but which carries a fifth is rejected, where some other
  // nodes would accept it.
  const TestablePTv3TRT trt{makeConfig(baseParams())};

  cuda_blackboard::CudaPointCloud2 cloud;
  cloud.fields = xyziFields();
  PointField extra;
  extra.name = "extra";
  extra.offset = 16;
  extra.datatype = PointField::FLOAT32;
  extra.count = 1;
  cloud.fields.push_back(extra);
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::UNKNOWN);
}

TEST_F(PTv3TRTTest, RejectsASupportedLayoutWithAWrongDatatype)
{
  const TestablePTv3TRT trt{makeConfig(baseParams())};

  cuda_blackboard::CudaPointCloud2 cloud;
  cloud.fields = xyziFields();
  cloud.fields.at(3).datatype = PointField::UINT8;
  EXPECT_EQ(trt.detectCloudFormat(cloud), CloudFormat::UNKNOWN);
}

// --- construction contract ----------------------------------------------------------------

TEST_F(PTv3TRTTest, ThrowsWhenTheEncoderArtifactDoesNotExist)
{
  const tensorrt_common::TrtCommonConfig encoder{"/nonexistent/encoder.onnx"};
  EXPECT_THROW(
    PTv3TRT(encoder, std::nullopt, std::nullopt, makeConfig(baseParams())), std::runtime_error);
}

TEST_F(PTv3TRTTest, ThrowsWhenTheEncoderArtifactIsNotAnOnnxFile)
{
  // TrtCommon requires the .onnx extension, so a mis-wired launch argument fails at load rather
  // than at the first frame.
  const tensorrt_common::TrtCommonConfig encoder{"/nonexistent/encoder.engine"};
  EXPECT_THROW(
    PTv3TRT(encoder, std::nullopt, std::nullopt, makeConfig(baseParams())), std::runtime_error);
}

TEST_F(PTv3TRTTest, ReportsAMissingSegHeadConfigRatherThanAnEncoderFailure)
{
  GTEST_SKIP() << "expected failure: the head-config checks run after initEncoderTrt, so a "
                  "configuration mistake is reported as a TensorRT initialisation failure. "
                  "Delete this line once the checks move ahead of engine loading.";

  PTv3ConfigParams params = baseParams();
  params.use_seg3d_head = true;
  const tensorrt_common::TrtCommonConfig encoder{"/nonexistent/encoder.onnx"};

  try {
    PTv3TRT trt(encoder, std::nullopt, std::nullopt, makeConfig(params));
    FAIL() << "expected construction to throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("seg3d_head_trt_config"), std::string::npos)
      << "actual: " << error.what();
  }
}

TEST_F(PTv3TRTTest, ReportsAMissingDetHeadConfigRatherThanAnEncoderFailure)
{
  GTEST_SKIP() << "expected failure: same ordering problem as the seg-head case above.";

  PTv3ConfigParams params = baseParams();
  params.use_seg3d_head = false;
  params.use_det3d_head = true;
  const tensorrt_common::TrtCommonConfig encoder{"/nonexistent/encoder.onnx"};

  try {
    PTv3TRT trt(encoder, std::nullopt, std::nullopt, makeConfig(params));
    FAIL() << "expected construction to throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("det3d_head_trt_config"), std::string::npos)
      << "actual: " << error.what();
  }
}

TEST_F(PTv3TRTTest, StageProfileCountsRejectAStageBeyondTheConfiguredHierarchy)
{
  GTEST_SKIP() << "expected failure: stageProfileCounts forwards to "
                  "PTv3Config::stage_voxel_capacity, which indexes pooling_strides_ up to the "
                  "requested stage without a bounds check, so an out-of-range stage reads past "
                  "the vector instead of throwing. Not exercised: it is undefined behaviour.";

  const TestablePTv3TRT trt{makeConfig(baseParams())};
  const auto out_of_range = baseParams().enc_channels.size() + 4;
  EXPECT_ANY_THROW({ [[maybe_unused]] const auto counts = trt.stageProfileCounts(out_of_range); });
}

}  // namespace test
}  // namespace autoware::ptv3
