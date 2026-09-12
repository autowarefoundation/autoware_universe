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

#include "autoware/lidar_centerpoint/ml_package_version.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

using autoware::lidar_centerpoint::check_ml_package_version;

TEST(TestSuite, supportedVersions)
{
  EXPECT_NO_THROW(check_ml_package_version("v4.1", ""));
  EXPECT_NO_THROW(check_ml_package_version("4.1", ""));
  EXPECT_NO_THROW(check_ml_package_version("v4.2", ""));
  EXPECT_NO_THROW(check_ml_package_version("v4.10", ""));
}

TEST(TestSuite, unsupportedVersions)
{
  EXPECT_THROW(check_ml_package_version("v4.0", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("v3.9", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("v5.0", ""), std::invalid_argument);
}

TEST(TestSuite, unreadableVersions)
{
  EXPECT_THROW(check_ml_package_version("", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("v4", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("v4.1.0", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("v4.x", ""), std::invalid_argument);
  EXPECT_THROW(check_ml_package_version("latest", ""), std::invalid_argument);
}

TEST(TestSuite, messageNamesTheBundle)
{
  const std::string model_path = "/data/ml_models/lidar_centerpoint/base";
  try {
    check_ml_package_version("v4.0", model_path);
    FAIL() << "expected std::invalid_argument";
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find(model_path), std::string::npos);
  }
}
