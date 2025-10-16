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

#ifndef AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__EXCEPTION_HPP_
#define AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__EXCEPTION_HPP_

#include <exception>
#include <string>

namespace autoware::simpl_prediction::archetype
{
/**
 * @brief An enumerate to represent error kind.
 */
enum class SimplError_t {
  TENSORRT = 0,       //!< TensorRT related error.
  CUDA = 1,           //!< CUDA related error.
  INVALID_VALUE = 2,  //!< Invalid value error.
  UNKNOWN = 3,        //!< Unknown error.
};

/**
 * @brief A class to hold error kind and message.
 */
struct SimplError
{
  /**
   * @brief Construct a new Simpl Error object without any message.
   *
   * @param kind Error kind.
   */
  explicit SimplError(const SimplError_t & kind) : kind(kind), msg("") {}

  /**
   * @brief Construct a new Simpl Error object with message.
   *
   * @param kind Error kind.
   * @param msg Error message.
   */
  explicit SimplError(const SimplError_t & kind, const std::string & msg) : kind(kind), msg(msg) {}

  SimplError_t kind;  //!< Error kind.
  std::string msg;    //!< Error message.
};

/**
 * @brief An exception class for `SimplError`.
 */
class SimplException : public std::exception
{
public:
  /**
   * @brief Construct a new Simpl Exception object.
   *
   * @param error `SimplError` object.
   */
  explicit SimplException(const SimplError & error) : error_(error) { append_message_header(); }

  /**
   * @brief Construct a new Simpl Exception object from the error kind and message.
   *
   * @param kind Error kind.
   * @param msg Error message.
   */
  SimplException(const SimplError_t & kind, const std::string & msg) : error_(kind, msg)
  {
    append_message_header();
  }

  /**
   * @brief Return the error message.
   */
  const char * what() const throw() { return msg_.c_str(); }

private:
  /**
   * @brief Append header to the error message depending on the kind.
   */
  void append_message_header() noexcept
  {
    if (error_.kind == SimplError_t::TENSORRT) {
      msg_ = "[TensorRT]: " + error_.msg;
    } else if (error_.kind == SimplError_t::CUDA) {
      msg_ = "[CUDA]: " + error_.msg;
    } else if (error_.kind == SimplError_t::INVALID_VALUE) {
      msg_ = "[InvalidValue]: " + error_.msg;
    } else {
      msg_ = "[UNKNOWN]: " + error_.msg;
    }
  }

  SimplError error_;  //!< Error object.
  std::string msg_;   //!<  Error message.
};
}  // namespace autoware::simpl_prediction::archetype
#endif  // AUTOWARE__SIMPL_PREDICTION__ARCHETYPE__EXCEPTION_HPP_
