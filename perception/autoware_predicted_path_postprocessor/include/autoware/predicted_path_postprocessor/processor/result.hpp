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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__RESULT_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__RESULT_HPP_

#include <utility>
#include <variant>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief Result class represents the outcome of a computation.
 * It can either hold a successful value or an error value.
 */
template <typename T, typename Err>
class Result
{
public:
  explicit Result(const T & value) : value_(value) {}
  explicit Result(T && value) : value_(std::move(value)) {}
  explicit Result(const Err & error) : value_(error) {}
  explicit Result(Err && error) : value_(std::move(error)) {}

  bool is_ok() const noexcept { return std::holds_alternative<T>(value_); }
  bool is_err() const noexcept { return std::holds_alternative<Err>(value_); }
  explicit operator bool() const noexcept { return is_ok(); }
  bool operator!() const noexcept { return is_err(); }

  const T & ok() const { return std::get<T>(value_); }
  const Err & err() const { return std::get<Err>(value_); }

  template <typename F>
  Result<std::invoke_result_t<F, const T &>, Err> map(F && f) const
  {
    using U = std::invoke_result_t<F, const T &>;
    return is_ok() ? Result<U, Err>(std::forward<F>(f)(ok())) : Result<U, Err>(err());
  }

  template <typename F>
  Result<T, std::invoke_result_t<F, const Err &>> map_err(F && f) const
  {
    using NewErr = std::invoke_result_t<F, const Err &>;
    return is_err() ? Result<T, NewErr>(std::forward<F>(f)(err())) : Result<T, NewErr>(ok());
  }

private:
  std::variant<T, Err> value_;  //!< Container of the expected value or error
};

template <typename Err>
using EmptyResult = Result<std::monostate, Err>;  //!< Alias for common Result patterns
                                                  //!< corresponding to Result<(), Err> in Rust

/**
 * @brief Return an EmptyResult object as successful.
 */
template <typename Err>
EmptyResult<Err> make_ok()
{
  return Result<std::monostate, Err>(std::monostate{});
}

/**
 * @brief Return a Result object with a successful value.
 */
template <typename T, typename Err, typename... Args>
Result<T, Err> make_ok(Args &&... args)
{
  return Result<T, Err>(T{std::forward<Args>(args)...});
}

/**
 * @brief Return an EmptyResult object as failed.
 */
template <typename Err, typename... Args>
EmptyResult<Err> make_err(Args &&... args)
{
  return Result<std::monostate, Err>(Err{std::forward<Args>(args)...});
}

/**
 * @brief Return a Result object with an error value.
 */
template <typename T, typename Err, typename... Args>
Result<T, Err> make_err(Args &&... args)
{
  return Result<T, Err>(Err{std::forward<Args>(args)...});
}
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__RESULT_HPP_
