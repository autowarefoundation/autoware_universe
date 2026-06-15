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

/**********************************************
 * @file meta_math.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief template arithmetic for figuring out how
 * much memory to allocate for neural network on GPU
 ***********************************************/

#ifndef MPPI__UTILS__NN_HELPERS__META_MATH_H_
#define MPPI__UTILS__NN_HELPERS__META_MATH_H_

template <typename... Args>
constexpr int input_dim(int first, Args... args)
{
  return first;
}

template <typename... Args>
constexpr int output_dim(int last)
{
  return last;
}

template <typename... Args>
constexpr int output_dim(int first, Args... args)
{
  return output_dim(args...);
}

template <typename... Args>
constexpr int param_counter(int first)
{
  return first;
}

template <typename... Args>
constexpr int param_counter(int first, int next)
{
  return (first + 1) * next;
}

template <typename... Args>
constexpr int param_counter(int first, int next, Args... args)
{
  return (first + 1) * next + param_counter(next, args...);
}

template <typename... Args>
constexpr int layer_counter(int first)
{
  return 1;
}

template <typename... Args>
constexpr int layer_counter(int first, Args... args)
{
  return 1 + layer_counter(args...);
}

template <typename... Args>
constexpr int neuron_counter(int first)
{
  return first;
}

template <typename... Args>
constexpr int neuron_counter(int first, Args... args)
{
  return (first > neuron_counter(args...)) ? first : neuron_counter(args...);
}

#endif  // MPPI__UTILS__NN_HELPERS__META_MATH_H_
