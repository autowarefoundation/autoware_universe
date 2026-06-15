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

//
// Created by mgandhi3 on 1/13/20.
//

#ifndef MPPI__UTILS__TEST_HELPER_H_
#define MPPI__UTILS__TEST_HELPER_H_
#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <array>
#include <vector>

inline void array_assert_float_eq(const std::vector<float>& known, const std::vector<float>& compute, int size)
{
  ASSERT_EQ(compute.size(), size) << "The computed vector size is not the given size!";
  ASSERT_EQ(known.size(), compute.size()) << "Two vectors are not the same size!";
  for (int i = 0; i < size; i++)
  {
    ASSERT_FLOAT_EQ(known[i], compute[i]) << "Failed at index: " << i;
  }
}

inline void array_assert_float_eq(const float known, const std::vector<float>& compute, const int size)
{
  ASSERT_EQ(compute.size(), size) << "The computed vector size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    ASSERT_FLOAT_EQ(known, compute[i]) << "Failed at index: " << i;
  }
}

template <int size>
inline void array_assert_float_eq(const std::array<float, size>& known, const std::array<float, size>& compute)
{
  ASSERT_EQ(compute.size(), size) << "The computed array size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    if (isnan(known[i]) || isnan(compute[i]))
    {
      ASSERT_EQ(isnan(known[i]), isnan(compute[i])) << "NaN check failed at index: " << i;
    }
    else if (isinf(known[i]) || isinf(compute[i]))
    {
      ASSERT_EQ(isinf(known[i]), isinf(compute[i])) << "inf check failed at index: " << i;
    }
    else
    {
      ASSERT_FLOAT_EQ(known[i], compute[i]) << "Failed at index: " << i;
    }
  }
}

template <int size>
inline void array_expect_float_eq(const std::array<float, size>& known, const std::array<float, size>& compute)
{
  ASSERT_EQ(compute.size(), size) << "The computed array size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    EXPECT_FLOAT_EQ(known[i], compute[i]) << "Failed at index: " << i;
  }
}

template <int size>
inline void array_expect_near(const std::array<float, size>& known, const std::array<float, size>& compute, float tol)
{
  ASSERT_EQ(compute.size(), size) << "The computed array size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    EXPECT_NEAR(known[i], compute[i], tol) << "Failed at index: " << i;
  }
}

template <int size>
inline void array_assert_float_eq(const float known, const std::array<float, size>& compute)
{
  ASSERT_EQ(compute.size(), size) << "The computed array size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    ASSERT_FLOAT_EQ(known, compute[i]) << "Failed at index: " << i;
  }
}

template <class EIGEN_MAT>
inline void eigen_assert_float_eq(const Eigen::Ref<const EIGEN_MAT>& known, const Eigen::Ref<const EIGEN_MAT>& compute)
{
  for (int i = 0; i < known.size(); i++)
  {
    ASSERT_FLOAT_EQ(known.data()[i], compute.data()[i]) << "Failed at index: " << i;
  }
}

template <class EIGEN_MAT>
inline void eigen_assert_float_near(const Eigen::Ref<const EIGEN_MAT>& known,
                                    const Eigen::Ref<const EIGEN_MAT>& compute, float tol)
{
  for (int i = 0; i < known.size(); i++)
  {
    ASSERT_NEAR(known.data()[i], compute.data()[i], tol * known.data()[i]) << "Failed at index: " << i;
  }
}

template <int size>
inline void array_assert_float_near(const std::array<float, size>& known, const std::array<float, size>& compute,
                                    float tol)
{
  ASSERT_EQ(compute.size(), size) << "The computed array size is not the given size!";
  for (int i = 0; i < size; i++)
  {
    ASSERT_NEAR(known[i], compute[i], tol) << "Failed at index: " << i;
  }
}

#endif  // MPPI__UTILS__TEST_HELPER_H_
