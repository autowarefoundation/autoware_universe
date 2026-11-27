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

#include "autoware/candidate_bank/exact_pair_queue.hpp"

#include <gtest/gtest.h>

using autoware::candidate_bank::ExactPairQueue;

TEST(ExactPairQueue, PairsByExactStampOnlyOnce)
{
  ExactPairQueue<int, int> q(2);
  q.first(100, 1);
  q.second(101, 2);
  EXPECT_FALSE(q.pop());

  q.second(100, 3);
  auto p = q.pop();
  ASSERT_TRUE(p);
  EXPECT_EQ(p->first, 1);
  EXPECT_EQ(p->second, 3);

  // an already consumed stamp is rejected
  q.first(100, 1);
  q.second(100, 3);
  EXPECT_FALSE(q.pop());

  // reversed arrival order
  q.first(101, 4);
  p = q.pop();
  ASSERT_TRUE(p);
  EXPECT_EQ(p->first, 4);
  EXPECT_EQ(p->second, 2);
}

TEST(ExactPairQueue, BoundedCapacityAndStaleRejection)
{
  ExactPairQueue<int, int> q(2);
  q.first(110, 1);
  q.first(120, 2);
  q.first(130, 3);
  q.second(110, 1);
  EXPECT_FALSE(q.pop());
  EXPECT_LE(q.pending(), 4u);

  q.second(130, 7);
  auto p = q.pop();
  ASSERT_TRUE(p);
  EXPECT_EQ(p->first, 3);
  EXPECT_EQ(p->second, 7);

  q.second(120, 2);
  EXPECT_FALSE(q.pop());
}

TEST(ExactPairQueue, ClearResetsLastConsumedStamp)
{
  ExactPairQueue<int, int> q(2);
  q.first(10, 1);
  q.second(10, 1);
  ASSERT_TRUE(q.pop());

  q.clear();
  q.second(1, 8);
  q.first(1, 9);
  auto p = q.pop();
  ASSERT_TRUE(p);
  EXPECT_EQ(p->first, 9);
  EXPECT_EQ(p->second, 8);
}
