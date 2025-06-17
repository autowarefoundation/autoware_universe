// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__CONFIG__ERRORS_HPP_
#define COMMON__CONFIG__ERRORS_HPP_

#include <stdexcept>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

struct Exception : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct FileNotFound : public Exception
{
  using Exception::Exception;
};

struct SameFileFound : public Exception
{
  using Exception::Exception;
};

struct SameDiagFound : public Exception
{
  using Exception::Exception;
};

struct InvalidType : public Exception
{
  using Exception::Exception;
};

struct FieldNotFound : public Exception
{
  using Exception::Exception;
};

struct UnknownSubstitution : public Exception
{
  using Exception::Exception;
};

struct UnknownLogic : public Exception
{
  using Exception::Exception;
};

struct PathConflict : public Exception
{
  using Exception::Exception;
};

struct PathNotFound : public Exception
{
  using Exception::Exception;
};

struct LinkLoopFound : public Exception
{
  using Exception::Exception;
};

struct UnitLoopFound : public Exception
{
  using Exception::Exception;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__ERRORS_HPP_
