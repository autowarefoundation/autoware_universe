// Copyright 2026 The Autoware Foundation
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

#include "top_memory_parser.hpp"

#include <gtest/gtest.h>

namespace autoware::component_monitor
{

TEST(TopMemoryParser, NoSuffixIsKiB)
{
  EXPECT_EQ(parse_top_res_to_bytes("0"), 0U);
  EXPECT_EQ(parse_top_res_to_bytes("18872"), 18872ULL * 1024ULL);
}

TEST(TopMemoryParser, SuffixParsing)
{
  EXPECT_EQ(parse_top_res_to_bytes("1k"), 1024U);
  EXPECT_EQ(parse_top_res_to_bytes("1m"), 1024ULL * 1024ULL);
  EXPECT_EQ(parse_top_res_to_bytes("1g"), 1024ULL * 1024ULL * 1024ULL);
  EXPECT_EQ(parse_top_res_to_bytes("1T"), 1024ULL * 1024ULL * 1024ULL * 1024ULL);
}

}  // namespace autoware::component_monitor
