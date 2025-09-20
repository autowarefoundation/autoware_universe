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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__RESULT_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__RESULT_HPP_

#include "autoware/simpl_prediction/archetype/exception.hpp"

#include <string>
#include <utility>
#include <variant>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief An class to hold expected value or error.
 *
 * @tparam T Data type of expected value.
 */
template <typename T>
class Result
{
public:
  /**
   * @brief Construct a new Result object with an expected value.
   *
   * @param value Expected value.
   */
  explicit Result(const T & value) : value_(value) {}

  /**
   * @brief Construct a new Result object with an error.
   *
   * @param error `SimplError` object.
   */
  explicit Result(const SimplError & error) : value_(error) {}

  /**
   * @brief Check whether holding value is expected type.
   */
  bool is_ok() const noexcept { return std::holds_alternative<T>(value_); }

  /**
   * @brief Return an expected value if it holds, overwise throw `SimpleException`.
   */
  T unwrap() const
  {
    if (is_ok()) {
      return std::move(std::get<T>(value_));
    } else {
      throw SimplException(std::get<SimplError>(value_));
    }
  }

private:
  std::variant<T, SimplError> value_;  //!< Container of expected value or error.
};

/**
 * @brief Returns `Result` with an expected value.
 *
 * @tparam T Data type of the expected value.
 * @param value Expected value.
 */
template <typename T>
Result<T> Ok(const T & value) noexcept
{
  return Result<T>(value);
}

/**
 * @brief Return `Result` with en error.
 *
 * @tparam T Data type of the expected value.
 * @param error `SimplError` object.
 */
template <typename T>
Result<T> Err(const SimplError & error) noexcept
{
  return Result<T>(error);
}

/**
 * @brief Return `Result` with en error.
 *
 * @tparam T Data type of the expected value.
 * @param kind Error kind.
 */
template <typename T>
Result<T> Err(const SimplError_t & kind) noexcept
{
  SimplError error(kind);
  return Result<T>(error);
}

/**
 * @brief Return `Result` with en error.
 *
 * @tparam T Data type of the expected value.
 * @param kind Error kind.
 * @param msg Error message.
 */
template <typename T>
Result<T> Err(const SimplError_t & kind, const std::string & msg) noexcept
{
  SimplError error(kind, msg);
  return Result<T>(error);
}
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__RESULT_HPP_
