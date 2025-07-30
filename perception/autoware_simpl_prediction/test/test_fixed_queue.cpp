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

#include "autoware/simpl_prediction/archetype/fixed_queue.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::simpl_prediction::test
{
using autoware::simpl_prediction::archetype::FixedQueue;

TEST(TestFixedQueue, PushBackWithinCapacity)
{
  FixedQueue<int> q(3);
  q.push_back(1);
  q.push_back(2);
  q.push_back(3);

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 1);
  EXPECT_EQ(q.back(), 3);
}

TEST(TestFixedQueue, PushBackExceedCapacity)
{
  FixedQueue<int> q(3);
  q.push_back(1);
  q.push_back(2);
  q.push_back(3);
  q.push_back(4);

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 2);
  EXPECT_EQ(q.back(), 4);
}

TEST(TestFixedQueue, IteratorAccess)
{
  FixedQueue<int> q(3);
  q.push_back(1);
  q.push_back(2);
  q.push_back(3);

  std::vector<int> v(q.begin(), q.end());
  ASSERT_EQ(v.size(), 3u);
  EXPECT_EQ(v[0], 1);
  EXPECT_EQ(v[1], 2);
  EXPECT_EQ(v[2], 3);
}

TEST(TestFixedQueue, PushLvalueAndRvalue)
{
  FixedQueue<std::string> q(2);
  std::string a = "foo";
  q.push_back(a);      // lvalue
  q.push_back("bar");  // rvalue

  EXPECT_EQ(q.front(), "foo");
  EXPECT_EQ(q.back(), "bar");
}
}  // namespace autoware::simpl_prediction::test
