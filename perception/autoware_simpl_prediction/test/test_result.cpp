// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl_prediction/archetype/result.hpp"

#include <gtest/gtest.h>

#include <string>

namespace autoware::simpl_prediction::test
{
using autoware::simpl_prediction::archetype::Err;
using autoware::simpl_prediction::archetype::Ok;
using autoware::simpl_prediction::archetype::SimplError;
using autoware::simpl_prediction::archetype::SimplError_t;
using autoware::simpl_prediction::archetype::SimplException;

TEST(TestResult, OkValueHoldsCorrectly)
{
  auto result = Ok<std::string>("success");

  EXPECT_TRUE(result.is_ok());
  EXPECT_EQ(result.unwrap(), "success");
}

TEST(TestResult, ErrWithSimplErrorThrowsException)
{
  SimplError error(SimplError_t::INVALID_VALUE, "Invalid config");
  auto result = Err<std::string>(error);

  EXPECT_FALSE(result.is_ok());
  EXPECT_THROW(
    {
      try {
        result.unwrap();
      } catch (const SimplException & e) {
        EXPECT_STREQ(e.what(), "[InvalidValue]: Invalid config");
        throw;
      }
    },
    SimplException);
}

TEST(TestResult, ErrWithErrorKindOnly)
{
  auto result = Err<int>(SimplError_t::CUDA);

  EXPECT_FALSE(result.is_ok());
  EXPECT_THROW(
    {
      try {
        result.unwrap();
      } catch (const SimplException & e) {
        EXPECT_STREQ(e.what(), "[CUDA]: ");
        throw;
      }
    },
    SimplException);
}

TEST(TestResult, ErrWithKindAndMessage)
{
  auto result = Err<float>(SimplError_t::TENSORRT, "engine build failed");

  EXPECT_FALSE(result.is_ok());
  EXPECT_THROW(
    {
      try {
        result.unwrap();
      } catch (const SimplException & e) {
        EXPECT_STREQ(e.what(), "[TensorRT]: engine build failed");
        throw;
      }
    },
    SimplException);
}
}  // namespace autoware::simpl_prediction::test
