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

#include "autoware/diffusion_planner/utils/fixed_queue.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{

class FixedQueueTest : public ::testing::Test
{
};

TEST_F(FixedQueueTest, PushBackWithinCapacity)
{
  FixedQueue<int> q(3);
  q.push_back(1);
  q.push_back(2);
  q.push_back(3);

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 1);
  EXPECT_EQ(q.back(), 3);
}

TEST_F(FixedQueueTest, PushBackExceedsCapacity)
{
  FixedQueue<int> q(3);
  q.push_back(1);
  q.push_back(2);
  q.push_back(3);
  q.push_back(4);  // Should evict 1

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 2);
  EXPECT_EQ(q.back(), 4);
}

TEST_F(FixedQueueTest, PushFrontWithinCapacity)
{
  FixedQueue<int> q(3);
  q.push_front(1);
  q.push_front(2);
  q.push_front(3);

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 3);
  EXPECT_EQ(q.back(), 1);
}

TEST_F(FixedQueueTest, PushFrontExceedsCapacity)
{
  FixedQueue<int> q(3);
  q.push_front(1);
  q.push_front(2);
  q.push_front(3);
  q.push_front(4);  // Should evict 1 from back

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 4);
  EXPECT_EQ(q.back(), 2);
}

TEST_F(FixedQueueTest, MixedPushFrontBack)
{
  FixedQueue<int> q(3);
  q.push_back(1);   // [1]
  q.push_front(2);  // [2,1]
  q.push_back(3);   // [2,1,3]
  q.push_front(4);  // [4,2,1] (3 evicted)

  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.front(), 4);
  EXPECT_EQ(q.back(), 1);
}

TEST_F(FixedQueueTest, IteratorAccess)
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

  std::vector<int> rv(q.rbegin(), q.rend());
  ASSERT_EQ(rv.size(), 3u);
  EXPECT_EQ(rv[0], 3);
  EXPECT_EQ(rv[2], 1);
}

TEST_F(FixedQueueTest, ConstCorrectness)
{
  FixedQueue<int> q(2);
  q.push_back(10);
  q.push_back(20);

  const auto & cq = q;
  EXPECT_EQ(cq.front(), 10);
  EXPECT_EQ(cq.back(), 20);
  EXPECT_EQ(*cq.begin(), 10);
  EXPECT_EQ(*(cq.end() - 1), 20);
}

TEST_F(FixedQueueTest, PushRvalueAndLvalue)
{
  FixedQueue<std::string> q(2);
  std::string a = "foo";
  q.push_back(a);                   // lvalue
  q.push_back(std::string("bar"));  // rvalue

  EXPECT_EQ(q.front(), "foo");
  EXPECT_EQ(q.back(), "bar");
}

}  // namespace autoware::diffusion_planner::test
