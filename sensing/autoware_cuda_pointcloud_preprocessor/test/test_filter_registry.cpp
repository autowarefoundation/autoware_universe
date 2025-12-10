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

#include "autoware/cuda_pointcloud_preprocessor/dag/filter_registry.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/cropbox_filter.hpp"
#include "autoware/cuda_pointcloud_preprocessor/dag/filters/transform_filter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>

using autoware::cuda_pointcloud_preprocessor::dag::CropBoxFilter;
using autoware::cuda_pointcloud_preprocessor::dag::FilterRegistry;
using autoware::cuda_pointcloud_preprocessor::dag::getFilterRegistry;
using autoware::cuda_pointcloud_preprocessor::dag::IFilter;
using autoware::cuda_pointcloud_preprocessor::dag::registerFilterType;
using autoware::cuda_pointcloud_preprocessor::dag::TransformFilter;

class FilterRegistryTest : public ::testing::Test
{
protected:
  void SetUp() override { registry_ = &getFilterRegistry(); }

  FilterRegistry * registry_;
};

TEST_F(FilterRegistryTest, RegisterAndCreateFilter)
{
  // Register a test filter
  registerFilterType<CropBoxFilter>("TestCropBoxFilter");

  // Check if registered
  EXPECT_TRUE(registry_->isRegistered("TestCropBoxFilter"));

  // Create instance
  auto filter = registry_->createFilter("TestCropBoxFilter");
  ASSERT_NE(filter, nullptr);

  // Verify metadata
  auto metadata = filter->getMetadata();
  EXPECT_EQ(metadata.filter_type, "CropBoxFilter");
  EXPECT_FALSE(metadata.required_inputs.empty());
}

TEST_F(FilterRegistryTest, NonExistentFilter)
{
  // Try to create non-existent filter
  auto filter = registry_->createFilter("NonExistentFilter");
  EXPECT_EQ(filter, nullptr);

  // Check registration status
  EXPECT_FALSE(registry_->isRegistered("NonExistentFilter"));
}

TEST_F(FilterRegistryTest, ListFilters)
{
  // Register multiple filters
  registerFilterType<CropBoxFilter>("ListTestCropBoxFilter");
  registerFilterType<TransformFilter>("ListTestTransformFilter");

  // List all filters
  auto filter_types = registry_->listFilters();

  // Should contain at least our registered filters
  EXPECT_FALSE(filter_types.empty());

  // Verify our filters are in the list
  bool found_organize = false;
  bool found_transform = false;
  for (const auto & type : filter_types) {
    if (type == "ListTestCropBoxFilter") found_organize = true;
    if (type == "ListTestTransformFilter") found_transform = true;
  }

  EXPECT_TRUE(found_organize);
  EXPECT_TRUE(found_transform);
}

TEST_F(FilterRegistryTest, GetMetadata)
{
  registerFilterType<CropBoxFilter>("MetadataTestFilter");

  // Get metadata directly from registry
  auto metadata = registry_->getMetadata("MetadataTestFilter");

  EXPECT_EQ(metadata.filter_type, "CropBoxFilter");
  EXPECT_FALSE(metadata.required_inputs.empty());
  EXPECT_FALSE(metadata.outputs.empty());
}

TEST_F(FilterRegistryTest, MultipleInstances)
{
  registerFilterType<CropBoxFilter>("MultiInstanceTestFilter");

  // Create multiple instances
  auto filter1 = registry_->createFilter("MultiInstanceTestFilter");
  auto filter2 = registry_->createFilter("MultiInstanceTestFilter");

  ASSERT_NE(filter1, nullptr);
  ASSERT_NE(filter2, nullptr);

  // Verify they are different instances
  EXPECT_NE(filter1.get(), filter2.get());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
