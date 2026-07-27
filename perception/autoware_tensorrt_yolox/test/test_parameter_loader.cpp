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

#include "autoware/tensorrt_yolox/parameter_loader.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

namespace
{

std::string next_id()
{
  static std::atomic<int> counter{0};
  return std::to_string(counter.fetch_add(1));
}

// Per-test scratch dir with the manifest + any config files under it. Cleaned
// up on destruction so nothing leaks between tests.
class TempBundle
{
public:
  TempBundle()
  : dir_(std::filesystem::temp_directory_path() / ("parameter_loader_test_" + next_id()))
  {
    std::filesystem::create_directories(dir_);
  }

  ~TempBundle()
  {
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  TempBundle(const TempBundle &) = delete;
  TempBundle & operator=(const TempBundle &) = delete;

  const std::filesystem::path & dir() const { return dir_; }

  void write(const std::string & name, const std::string & content) const
  {
    std::ofstream(dir_ / name) << content;
  }

  void write_manifest(const std::vector<std::string> & files) const
  {
    std::ofstream out(dir_ / autoware::tensorrt_yolox::kParameterManifestFilename);
    out << "config_files:\n";
    for (const auto & f : files) {
      out << "  - " << f << "\n";
    }
  }

private:
  std::filesystem::path dir_;
};

class ParamsLoaderTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<rclcpp::Node> make_node()
  {
    return std::make_shared<rclcpp::Node>("parameter_loader_test_node_" + next_id());
  }

  // Convenience: build a bundle with a single yaml body and load it.
  std::shared_ptr<rclcpp::Node> load_single(TempBundle & bundle, const std::string & yaml_body)
  {
    bundle.write("cfg.yaml", yaml_body);
    bundle.write_manifest({"cfg.yaml"});
    auto node = make_node();
    autoware::tensorrt_yolox::load_parameters_from_manifest(*node, bundle.dir());
    return node;
  }
};

}  // namespace

using autoware::tensorrt_yolox::get_parameter_or;
using autoware::tensorrt_yolox::join_relative;
using autoware::tensorrt_yolox::load_parameters_from_manifest;
using autoware::tensorrt_yolox::ParameterReader;

// ---------------------------------------------------------------------------
// Scalar parameter types
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, LoadsBoolTrueAndFalse)
{
  TempBundle b;
  auto node = load_single(b, "enabled: true\ndisabled: false\n");

  EXPECT_EQ(node->get_parameter("enabled").get_type(), rclcpp::ParameterType::PARAMETER_BOOL);
  EXPECT_TRUE(node->get_parameter("enabled").as_bool());
  EXPECT_EQ(node->get_parameter("disabled").get_type(), rclcpp::ParameterType::PARAMETER_BOOL);
  EXPECT_FALSE(node->get_parameter("disabled").as_bool());
}

TEST_F(ParamsLoaderTest, LoadsInteger)
{
  TempBundle b;
  auto node = load_single(b, "count: 42\nnegative: -7\nzero: 0\n");

  EXPECT_EQ(node->get_parameter("count").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(node->get_parameter("count").as_int(), 42);
  EXPECT_EQ(node->get_parameter("negative").as_int(), -7);
  EXPECT_EQ(node->get_parameter("zero").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(node->get_parameter("zero").as_int(), 0);
}

TEST_F(ParamsLoaderTest, LoadsDouble)
{
  TempBundle b;
  auto node = load_single(b, "threshold: 0.35\nnegative: -1.5\nwhole: 2.0\n");

  EXPECT_EQ(node->get_parameter("threshold").get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_DOUBLE_EQ(node->get_parameter("threshold").as_double(), 0.35);
  EXPECT_DOUBLE_EQ(node->get_parameter("negative").as_double(), -1.5);
  EXPECT_DOUBLE_EQ(node->get_parameter("whole").as_double(), 2.0);
}

TEST_F(ParamsLoaderTest, LoadsString)
{
  TempBundle b;
  auto node = load_single(b, "name: \"tensorrt_yolox\"\nprecision: int8\n");

  EXPECT_EQ(node->get_parameter("name").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(node->get_parameter("name").as_string(), "tensorrt_yolox");
  EXPECT_EQ(node->get_parameter("precision").as_string(), "int8");
}

// Numeric literals wrapped in quotes should stay strings, matching what yaml
// authors expect ("42" is a string identifier, not the integer 42).
TEST_F(ParamsLoaderTest, QuotedNumberStaysString)
{
  TempBundle b;
  auto node = load_single(b, "version: \"42\"\nrate: \"0.5\"\n");

  EXPECT_EQ(node->get_parameter("version").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(node->get_parameter("version").as_string(), "42");
  EXPECT_EQ(node->get_parameter("rate").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(node->get_parameter("rate").as_string(), "0.5");
}

// The loader guards against yaml-cpp's numeric-to-bool decoding: `1` must
// stay an int, not become `true`.
TEST_F(ParamsLoaderTest, NumericOneIsIntNotBool)
{
  TempBundle b;
  auto node = load_single(b, "flag_int: 1\nflag_zero: 0\n");

  EXPECT_EQ(node->get_parameter("flag_int").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(node->get_parameter("flag_int").as_int(), 1);
  EXPECT_EQ(node->get_parameter("flag_zero").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(node->get_parameter("flag_zero").as_int(), 0);
}

// ---------------------------------------------------------------------------
// Array parameter types
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, LoadsBoolArray)
{
  TempBundle b;
  auto node = load_single(b, "flags: [true, false, true]\n");

  const auto p = node->get_parameter("flags");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
  EXPECT_EQ(p.as_bool_array(), (std::vector<bool>{true, false, true}));
}

TEST_F(ParamsLoaderTest, LoadsIntegerArray)
{
  TempBundle b;
  auto node = load_single(b, "priorities: [3, 1, 2, -5]\n");

  const auto p = node->get_parameter("priorities");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(p.as_integer_array(), (std::vector<int64_t>{3, 1, 2, -5}));
}

TEST_F(ParamsLoaderTest, LoadsDoubleArray)
{
  TempBundle b;
  auto node = load_single(b, "thresholds: [0.35, 0.5, 0.7]\n");

  const auto p = node->get_parameter("thresholds");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(p.as_double_array(), (std::vector<double>{0.35, 0.5, 0.7}));
}

TEST_F(ParamsLoaderTest, LoadsStringArray)
{
  TempBundle b;
  auto node = load_single(b, "classes: [\"car\", \"truck\", \"pedestrian\"]\n");

  const auto p = node->get_parameter("classes");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(p.as_string_array(), (std::vector<std::string>{"car", "truck", "pedestrian"}));
}

// `[1, 2.5]` should be a double array — ints widen naturally.
TEST_F(ParamsLoaderTest, IntsInDoubleArrayAreWidened)
{
  TempBundle b;
  auto node = load_single(b, "mixed: [1, 2.5, 3]\n");

  const auto p = node->get_parameter("mixed");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(p.as_double_array(), (std::vector<double>{1.0, 2.5, 3.0}));
}

// A sequence with genuinely mixed non-numeric types falls back to a string
// array so nothing is lost.
TEST_F(ParamsLoaderTest, InhomogeneousSequenceFallsBackToStringArray)
{
  TempBundle b;
  auto node = load_single(b, "mixed: [1, \"two\", true]\n");

  const auto p = node->get_parameter("mixed");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_EQ(p.as_string_array(), (std::vector<std::string>{"1", "two", "true"}));
}

// An empty sequence has no element type to infer, so it becomes an empty
// string array. Consumers that need a typed empty array should keep the key
// out of the yaml and rely on `p.optional(name, ParameterValue(vec))`.
TEST_F(ParamsLoaderTest, EmptySequenceIsEmptyStringArray)
{
  TempBundle b;
  auto node = load_single(b, "empty: []\n");

  const auto p = node->get_parameter("empty");
  EXPECT_EQ(p.get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  EXPECT_TRUE(p.as_string_array().empty());
}

// ---------------------------------------------------------------------------
// Structural: nested map flattening and multi-file merge
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, NestedMapsFlattenToDottedKeys)
{
  TempBundle b;
  auto node = load_single(
    b,
    "image_pub:\n"
    "  qos:\n"
    "    depth: 5\n"
    "    reliable: true\n"
    "detector:\n"
    "  score_threshold: 0.35\n");

  EXPECT_EQ(node->get_parameter("image_pub.qos.depth").as_int(), 5);
  EXPECT_TRUE(node->get_parameter("image_pub.qos.reliable").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("detector.score_threshold").as_double(), 0.35);
}

TEST_F(ParamsLoaderTest, LaterFileOverridesEarlierValue)
{
  TempBundle b;
  b.write("base.yaml", "score_threshold: 0.35\n");
  b.write("override.yaml", "score_threshold: 0.7\n");
  b.write_manifest({"base.yaml", "override.yaml"});
  auto node = make_node();
  load_parameters_from_manifest(*node, b.dir());

  EXPECT_DOUBLE_EQ(node->get_parameter("score_threshold").as_double(), 0.7);
}

// ---------------------------------------------------------------------------
// Manifest error paths
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, MissingManifestThrows)
{
  TempBundle b;
  auto node = make_node();
  EXPECT_THROW(load_parameters_from_manifest(*node, b.dir()), std::runtime_error);
}

TEST_F(ParamsLoaderTest, ManifestWithoutConfigFilesThrows)
{
  TempBundle b;
  std::ofstream(b.dir() / autoware::tensorrt_yolox::kParameterManifestFilename)
    << "something_else: true\n";
  auto node = make_node();
  EXPECT_THROW(load_parameters_from_manifest(*node, b.dir()), std::runtime_error);
}

TEST_F(ParamsLoaderTest, ManifestListingNonexistentFileThrows)
{
  TempBundle b;
  b.write_manifest({"does_not_exist.yaml"});
  auto node = make_node();
  EXPECT_THROW(load_parameters_from_manifest(*node, b.dir()), std::runtime_error);
}

// ---------------------------------------------------------------------------
// resolve_relative_path{,_or}
// ---------------------------------------------------------------------------

// `join_relative` does one thing: validate that the second arg is a
// non-empty relative path and concatenate it onto the first. It never opens
// the files, so tests just check the string algebra. The scratch dir is used
// as a root because it exists on any machine (including CI) and is clearly
// test-owned.

TEST_F(ParamsLoaderTest, JoinRelativeJoinsWithRoot)
{
  TempBundle b;
  EXPECT_EQ(join_relative(b.dir(), "sub/label.txt"), (b.dir() / "sub/label.txt").string());
}

TEST_F(ParamsLoaderTest, JoinRelativeAllowsParentDir)
{
  TempBundle b;
  const auto child = b.dir() / "child";  // no need to create — string-only op
  EXPECT_EQ(join_relative(child, "../remap.csv"), (b.dir() / "remap.csv").string());
}

TEST_F(ParamsLoaderTest, JoinRelativeRejectsAbsolutePath)
{
  TempBundle b;
  const auto abs_path = (b.dir() / "not_allowed.txt").string();
  EXPECT_THROW(join_relative(b.dir(), abs_path), std::runtime_error);
}

TEST_F(ParamsLoaderTest, JoinRelativeRejectsEmptyString)
{
  TempBundle b;
  EXPECT_THROW(join_relative(b.dir(), ""), std::runtime_error);
}

// ---------------------------------------------------------------------------
// get_parameter_or
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, GetParamOrReturnsLoadedValue)
{
  TempBundle b;
  auto node = load_single(b, "score_threshold: 0.42\n");

  EXPECT_DOUBLE_EQ(get_parameter_or<double>(*node, "score_threshold", 0.99), 0.42);
}

TEST_F(ParamsLoaderTest, GetParamOrReturnsDefaultWhenMissing)
{
  auto node = make_node();
  EXPECT_DOUBLE_EQ(get_parameter_or<double>(*node, "not_declared", 0.99), 0.99);
  EXPECT_EQ(get_parameter_or<std::string>(*node, "not_declared", "fallback"), "fallback");
}

// ---------------------------------------------------------------------------
// ParameterReader: manifest mode
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, ParameterReaderRequiredReturnsLoadedInManifestMode)
{
  TempBundle b;
  auto node = load_single(b, "score_threshold: 0.35\n");

  ParameterReader p(*node, /*use_manifest=*/true);
  EXPECT_DOUBLE_EQ(p.required("score_threshold").as_double(), 0.35);
}

// When a required key isn't in any yaml the manifest lists, the error must
// name the missing parameter (rclcpp's default only carries the bare name,
// which was hard to read at the terminal).
TEST_F(ParamsLoaderTest, ParameterReaderRequiredMissingInManifestModeGivesDescriptiveError)
{
  TempBundle b;
  auto node = load_single(b, "other: 1\n");

  ParameterReader p(*node, /*use_manifest=*/true);
  try {
    p.required("model_file");
    FAIL() << "expected required() to throw for a missing parameter";
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("model_file"), std::string::npos)
      << "error message should name the missing parameter, got: " << what;
    EXPECT_NE(what.find("not declared"), std::string::npos)
      << "error message should explain the failure, got: " << what;
  }
}

TEST_F(ParamsLoaderTest, ParameterReaderOptionalReturnsLoadedInManifestMode)
{
  TempBundle b;
  auto node = load_single(b, "calibration_image_list_path: \"images/list.txt\"\n");

  ParameterReader p(*node, /*use_manifest=*/true);
  EXPECT_EQ(
    p.optional("calibration_image_list_path", rclcpp::ParameterValue(std::string{})).as_string(),
    "images/list.txt");
}

TEST_F(ParamsLoaderTest, ParameterReaderOptionalReturnsDefaultWhenMissingInManifestMode)
{
  TempBundle b;
  auto node = load_single(b, "other: 1\n");

  ParameterReader p(*node, /*use_manifest=*/true);
  EXPECT_EQ(
    p.optional("not_present", rclcpp::ParameterValue(std::string{"default_val"})).as_string(),
    "default_val");
}

// ---------------------------------------------------------------------------
// ParameterReader: classic mode (no manifest, launcher-supplied overrides)
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, ParameterReaderOptionalDeclaresWithDefaultInClassicMode)
{
  auto node = make_node();

  ParameterReader p(*node, /*use_manifest=*/false);
  const auto value = p.optional("threshold", rclcpp::ParameterValue(0.9)).as_double();
  EXPECT_DOUBLE_EQ(value, 0.9);
  // Optional in classic mode must actually declare the parameter so a
  // launcher-side `<param>` override would win — verify it's now declared.
  EXPECT_TRUE(node->has_parameter("threshold"));
}

TEST_F(ParamsLoaderTest, ParameterReaderClassicHonorsOverrideFromNodeOptions)
{
  // Simulate a launcher `<param name="score_threshold" value="0.99">` by
  // seeding NodeOptions with the override, then confirm classic mode picks
  // it up via declare_parameter.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("score_threshold", 0.99)});
  auto node = std::make_shared<rclcpp::Node>("parameter_loader_test_node_" + next_id(), options);

  ParameterReader p(*node, /*use_manifest=*/false);
  const auto value = p.optional("score_threshold", rclcpp::ParameterValue(0.35)).as_double();
  EXPECT_DOUBLE_EQ(value, 0.99);
}

// Regression: `required()` in classic mode used to pass PARAMETER_NOT_SET as
// a declare_parameter default, which rclcpp rejects at construction time
// ("declare_parameter(): the provided parameter type cannot be
// rclcpp::PARAMETER_NOT_SET"). Fixed by declaring with dynamic_typing = true.
TEST_F(ParamsLoaderTest, ParameterReaderRequiredHonorsOverrideInClassicMode)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("model_path", std::string{"/tmp/model.onnx"})});
  auto node = std::make_shared<rclcpp::Node>("parameter_loader_test_node_" + next_id(), options);

  ParameterReader p(*node, /*use_manifest=*/false);
  EXPECT_EQ(p.required("model_path").as_string(), "/tmp/model.onnx");
}

// Without a launcher override, classic-mode `required()` must NOT throw at
// declare time (that's the bug we're guarding against). The parameter stays
// unset, and a typed read is what surfaces the "missing config" error.
TEST_F(ParamsLoaderTest, ParameterReaderRequiredWithoutOverrideYieldsUnsetValue)
{
  auto node = make_node();
  ParameterReader p(*node, /*use_manifest=*/false);

  EXPECT_NO_THROW(p.required("no_override_provided"));
  EXPECT_EQ(
    node->get_parameter("no_override_provided").get_type(),
    rclcpp::ParameterType::PARAMETER_NOT_SET);
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

// yaml-cpp keeps the last occurrence of a duplicated key. Documenting this so
// downstream users know they can't rely on "first wins" semantics.
TEST_F(ParamsLoaderTest, DuplicateKeyInSameFileLastWins)
{
  TempBundle b;
  auto node = load_single(b, "score_threshold: 0.1\nscore_threshold: 0.9\n");

  EXPECT_DOUBLE_EQ(node->get_parameter("score_threshold").as_double(), 0.9);
}

// YAML has multiple bool spellings; only `true`/`false` (any case of the
// first letter) round-trip to bool. Other spellings stay strings, matching
// how ROS 2's own yaml parser behaves.
TEST_F(ParamsLoaderTest, YamlYesNoOnOffAreNotBools)
{
  TempBundle b;
  auto node = load_single(b, "flag_yes: yes\nflag_no: no\nflag_on: on\nflag_off: off\n");

  EXPECT_EQ(node->get_parameter("flag_yes").get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(node->get_parameter("flag_yes").as_string(), "yes");
  EXPECT_EQ(node->get_parameter("flag_no").as_string(), "no");
  EXPECT_EQ(node->get_parameter("flag_on").as_string(), "on");
  EXPECT_EQ(node->get_parameter("flag_off").as_string(), "off");
}

// int64 limits round-trip correctly. If either bound silently truncated we'd
// silently corrupt configs for things like memory sizes / timestamps.
TEST_F(ParamsLoaderTest, IntegerBoundariesRoundTrip)
{
  TempBundle b;
  auto node = load_single(
    b,
    "int_max: 9223372036854775807\n"
    "int_min: -9223372036854775808\n");

  EXPECT_EQ(node->get_parameter("int_max").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER);
  EXPECT_EQ(node->get_parameter("int_max").as_int(), std::numeric_limits<int64_t>::max());
  EXPECT_EQ(node->get_parameter("int_min").as_int(), std::numeric_limits<int64_t>::min());
}

// YAML's `.nan` and `.inf` literals must land as double-typed values.
TEST_F(ParamsLoaderTest, SpecialFloatValues)
{
  TempBundle b;
  auto node = load_single(b, "nan_v: .nan\npos_inf: .inf\nneg_inf: -.inf\n");

  const auto nan_v = node->get_parameter("nan_v");
  EXPECT_EQ(nan_v.get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_TRUE(std::isnan(nan_v.as_double()));

  const auto pos = node->get_parameter("pos_inf").as_double();
  EXPECT_TRUE(std::isinf(pos));
  EXPECT_GT(pos, 0.0);

  const auto neg = node->get_parameter("neg_inf").as_double();
  EXPECT_TRUE(std::isinf(neg));
  EXPECT_LT(neg, 0.0);
}

// Scientific notation is a common way to spell tight thresholds; make sure it
// still resolves to double.
TEST_F(ParamsLoaderTest, ScientificNotationBecomesDouble)
{
  TempBundle b;
  auto node = load_single(b, "eps: 1e-6\nbig: 2.5e10\n");

  EXPECT_EQ(node->get_parameter("eps").get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE);
  EXPECT_DOUBLE_EQ(node->get_parameter("eps").as_double(), 1e-6);
  EXPECT_DOUBLE_EQ(node->get_parameter("big").as_double(), 2.5e10);
}

// A single-element sequence is still an array. This tripped up a couple of
// hand-rolled parsers in the past — worth pinning down.
TEST_F(ParamsLoaderTest, SingleElementSequenceIsArray)
{
  TempBundle b;
  auto node = load_single(b, "one_int: [7]\none_str: [\"solo\"]\n");

  EXPECT_EQ(
    node->get_parameter("one_int").get_type(), rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
  EXPECT_EQ(node->get_parameter("one_int").as_integer_array(), (std::vector<int64_t>{7}));
  EXPECT_EQ(node->get_parameter("one_str").as_string_array(), (std::vector<std::string>{"solo"}));
}

// Strings with embedded whitespace, quotes, and unicode survive verbatim.
TEST_F(ParamsLoaderTest, StringsPreserveWhitespaceAndUnicode)
{
  TempBundle b;
  auto node = load_single(
    b,
    "greeting: \"hello, world\"\n"
    "japanese: \"こんにちは\"\n"
    "with_spaces: \"  padded  \"\n");

  EXPECT_EQ(node->get_parameter("greeting").as_string(), "hello, world");
  EXPECT_EQ(node->get_parameter("japanese").as_string(), std::string(u8"こんにちは"));
  EXPECT_EQ(node->get_parameter("with_spaces").as_string(), "  padded  ");
}

// A key that is literally `null` (yaml scalar `~`) should be skipped rather
// than declared as an untyped parameter, which would break rclcpp's type
// tracking.
TEST_F(ParamsLoaderTest, NullValueIsSkipped)
{
  TempBundle b;
  auto node = load_single(b, "explicit_null: ~\nempty:\nreal: 1\n");

  EXPECT_FALSE(node->has_parameter("explicit_null"));
  EXPECT_FALSE(node->has_parameter("empty"));
  EXPECT_EQ(node->get_parameter("real").as_int(), 1);
}

// YAML anchors and aliases are resolved by yaml-cpp before we ever see the
// node, so the loader gets flattened values.
TEST_F(ParamsLoaderTest, YamlAnchorsAndAliasesResolve)
{
  TempBundle b;
  auto node = load_single(
    b,
    "default: &default 0.42\n"
    "score_threshold: *default\n"
    "nms_threshold: *default\n");

  EXPECT_DOUBLE_EQ(node->get_parameter("score_threshold").as_double(), 0.42);
  EXPECT_DOUBLE_EQ(node->get_parameter("nms_threshold").as_double(), 0.42);
}

// Deeply nested maps must flatten cleanly — the `.` join has no depth cap.
TEST_F(ParamsLoaderTest, DeeplyNestedMapsFlatten)
{
  TempBundle b;
  auto node = load_single(
    b,
    "a:\n"
    "  b:\n"
    "    c:\n"
    "      d:\n"
    "        e: 42\n");

  EXPECT_EQ(node->get_parameter("a.b.c.d.e").as_int(), 42);
}

// A yaml file whose top-level isn't a map (e.g. a bare sequence or scalar)
// isn't a valid parameter file — reject early rather than silently ignoring.
TEST_F(ParamsLoaderTest, TopLevelNonMapThrows)
{
  TempBundle b;
  b.write("bad.yaml", "- just_a_sequence\n- with_no_keys\n");
  b.write_manifest({"bad.yaml"});
  auto node = make_node();

  EXPECT_THROW(load_parameters_from_manifest(*node, b.dir()), std::runtime_error);
}

// Malformed yaml (syntax error) should surface a clear failure. yaml-cpp
// throws `YAML::Exception`, which is-a `std::exception`.
TEST_F(ParamsLoaderTest, MalformedYamlThrows)
{
  TempBundle b;
  b.write("bad.yaml", "key: [unclosed\n");
  b.write_manifest({"bad.yaml"});
  auto node = make_node();

  EXPECT_THROW(load_parameters_from_manifest(*node, b.dir()), std::exception);
}

// Config dir that doesn't exist should throw (from the missing-manifest
// check) rather than silently produce an empty parameter set.
TEST_F(ParamsLoaderTest, NonexistentConfigDirThrows)
{
  // Build a per-test-unique subpath under the temp dir but do NOT create it.
  // Guarantees non-existence without hard-coding a machine-specific path.
  const auto missing =
    std::filesystem::temp_directory_path() / ("parameter_loader_missing_" + next_id());
  auto node = make_node();
  EXPECT_THROW(load_parameters_from_manifest(*node, missing), std::runtime_error);
}

// A manifest with an empty `config_files:` sequence is legal — it just means
// "load nothing". Useful during migration when a bundle is being staged.
TEST_F(ParamsLoaderTest, EmptyManifestLoadsNothing)
{
  TempBundle b;
  std::ofstream(b.dir() / autoware::tensorrt_yolox::kParameterManifestFilename)
    << "config_files: []\n";
  auto node = make_node();

  EXPECT_NO_THROW(load_parameters_from_manifest(*node, b.dir()));
  // The node should have no user-declared parameters after this.
  EXPECT_FALSE(node->has_parameter("anything"));
}

// A manifest entry with `#` in front is a plain yaml comment — the loader
// only sees the surviving entries. This is the "disable one file during
// testing" workflow.
TEST_F(ParamsLoaderTest, CommentedManifestEntryIsSkipped)
{
  TempBundle b;
  b.write("active.yaml", "score_threshold: 0.5\n");
  b.write("disabled.yaml", "score_threshold: 0.1\n");
  std::ofstream(b.dir() / autoware::tensorrt_yolox::kParameterManifestFilename)
    << "config_files:\n"
    << "  - active.yaml\n"
    << "  # - disabled.yaml\n";
  auto node = make_node();
  load_parameters_from_manifest(*node, b.dir());

  EXPECT_DOUBLE_EQ(node->get_parameter("score_threshold").as_double(), 0.5);
}

// ---------------------------------------------------------------------------
// ROS 2 `--params-file` compatibility: `/<node>: ros__parameters:` wrappers
// are auto-unwrapped so the same yaml can be used as-is either way.
// ---------------------------------------------------------------------------

TEST_F(ParamsLoaderTest, LoadsRos2NodeScopedFormat)
{
  TempBundle b;
  auto node = load_single(
    b,
    "/teleop_turtle:\n"
    "  ros__parameters:\n"
    "    scale_angular: 2.0\n"
    "    scale_linear: 0.5\n");

  EXPECT_DOUBLE_EQ(node->get_parameter("scale_angular").as_double(), 2.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("scale_linear").as_double(), 0.5);
}

TEST_F(ParamsLoaderTest, LoadsRos2WildcardFormat)
{
  TempBundle b;
  auto node = load_single(
    b,
    "/**:\n"
    "  ros__parameters:\n"
    "    score_threshold: 0.35\n"
    "    precision: \"int8\"\n");

  EXPECT_DOUBLE_EQ(node->get_parameter("score_threshold").as_double(), 0.35);
  EXPECT_EQ(node->get_parameter("precision").as_string(), "int8");
}

// A single file may contain multiple node blocks; the loader unwraps each
// and merges (later wins), so authors don't have to split the file.
TEST_F(ParamsLoaderTest, LoadsRos2MultipleNodeBlocksMergeLaterWins)
{
  TempBundle b;
  auto node = load_single(
    b,
    "/first_node:\n"
    "  ros__parameters:\n"
    "    common_key: 1\n"
    "    only_in_first: \"a\"\n"
    "/second_node:\n"
    "  ros__parameters:\n"
    "    common_key: 2\n"
    "    only_in_second: \"b\"\n");

  EXPECT_EQ(node->get_parameter("common_key").as_int(), 2);
  EXPECT_EQ(node->get_parameter("only_in_first").as_string(), "a");
  EXPECT_EQ(node->get_parameter("only_in_second").as_string(), "b");
}

// Nested maps under `ros__parameters:` flatten to dotted keys exactly like
// they do in flat yaml. This is the standard ROS 2 nested-param form.
TEST_F(ParamsLoaderTest, NestedKeysUnderRos2WrapperFlatten)
{
  TempBundle b;
  auto node = load_single(
    b,
    "/**:\n"
    "  ros__parameters:\n"
    "    image_pub:\n"
    "      qos:\n"
    "        depth: 5\n"
    "        reliable: true\n");

  EXPECT_EQ(node->get_parameter("image_pub.qos.depth").as_int(), 5);
  EXPECT_TRUE(node->get_parameter("image_pub.qos.reliable").as_bool());
}

// A ROS-wrapped block and a flat top-level key can coexist. Useful when
// migrating incrementally — the ROS half stays working while a new flat
// key is added alongside.
TEST_F(ParamsLoaderTest, MixedFlatAndRos2FormatInOneFile)
{
  TempBundle b;
  auto node = load_single(
    b,
    "/**:\n"
    "  ros__parameters:\n"
    "    wrapped_key: 42\n"
    "flat_key: \"hello\"\n");

  EXPECT_EQ(node->get_parameter("wrapped_key").as_int(), 42);
  EXPECT_EQ(node->get_parameter("flat_key").as_string(), "hello");
}

// Unwrapping only kicks in at the top level. A `/`-prefixed key nested under
// something else is left alone — it becomes part of a dotted parameter name.
// (Not a name a real ROS param would ever have, but the point is that the
// unwrap heuristic isn't over-eager.)
TEST_F(ParamsLoaderTest, Ros2UnwrapDoesNotApplyToNonTopLevelKeys)
{
  TempBundle b;
  auto node = load_single(
    b,
    "outer:\n"
    "  /nested:\n"
    "    ros__parameters:\n"
    "      inner: 1\n");

  // Values under nested `/x: ros__parameters:` don't get unwrapped, so the
  // full dotted path (with slashes preserved by yaml-cpp) is what shows up.
  EXPECT_TRUE(node->has_parameter("outer./nested.ros__parameters.inner"));
  EXPECT_FALSE(node->has_parameter("inner"));
}

// The wrapper must include a `ros__parameters:` key — otherwise the top-level
// `/foo:` is just a regular map and gets flattened normally. Documenting so
// the heuristic's boundary is obvious.
TEST_F(ParamsLoaderTest, TopLevelSlashKeyWithoutRos2WrapperIsFlattened)
{
  TempBundle b;
  auto node = load_single(b, "/foo:\n  bar: 1\n");

  EXPECT_TRUE(node->has_parameter("/foo.bar"));
  EXPECT_FALSE(node->has_parameter("bar"));
}
