/**********************************************
 * @file meta_math.h
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief template arithmetic for figuring out how
 * much memory to allocate for neural network on GPU
 ***********************************************/

#ifndef MPPIGENERIC_META_MATH_H
#define MPPIGENERIC_META_MATH_H

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

#endif  // MPPIGENERIC_META_MATH_H
