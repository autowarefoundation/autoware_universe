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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__LOG_WRITER_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__LOG_WRITER_HPP_

#include <fmt/format.h>

#include <cstddef>
#include <string_view>
#include <type_traits>

// Nothing in this file is specific to this package, apart from two things: the namespace
// below, and the AW_MPC_ prefix of the macros at the end. To use the file in another
// package, copy it, change those two, and let the include guard hook fix the guard.

namespace autoware::motion::control::mpc_lateral_controller
{
/// Where a line of the log goes. The node builds the one that writes to a logger of ROS; a
/// test reads the lines instead of writing them anywhere.
///
/// Use the macros at the end of this file rather than these two functions. They build the
/// message only when it would be read, and they keep the waiting time of each place in the
/// code apart.
class LogWriter
{
public:
  enum class Level { debug, info, warn, error, fatal };

  /// The place in the code a line comes from. The macros at the end fill it.
  struct Site
  {
    const char * function_name;
    const char * file_name;
    size_t line_number;
  };

  virtual ~LogWriter() = default;

  /// Whether a line of this level would be read now. `after_s` is the shortest time between
  /// two lines from the same place in the code, and zero writes every time. `last_sent_s`
  /// holds when that place wrote last, and a call that returns true sets it to now.
  virtual bool shouldWrite(Level level, double after_s, double & last_sent_s) const = 0;

  /// Write the finished line, naming the place it comes from.
  virtual void write(Level level, const Site & site, std::string_view line) const = 0;
};

/// A writer that drops every line, for a caller that wants no log at all.
class NullLogWriter : public LogWriter
{
public:
  bool shouldWrite(Level, double, double &) const override { return false; }
  void write(Level, const Site &, std::string_view) const override {}
};

/// The writer a caller meets before it is given one of its own. It is one object for the
/// whole program, so that no class has to hold one of its own.
inline const NullLogWriter no_log{};
}  // namespace autoware::motion::control::mpc_lateral_controller

/// Write a line through `writer`, at most once per `after_s` seconds. The message is built
/// only when it would be read, so the arguments are not evaluated otherwise. The macro runs
/// where the line is written, so it can name that place; a writer called as a plain function
/// could only name itself.
///
/// The waiting time lives in a `static` of the line that writes, not in the writer, in the
/// same way the throttling macros of rclcpp keep theirs. Two objects that run the same line
/// therefore share one waiting time, and so do two tests in one test program. Writing this
/// macro inside a function defined in a header shares the waiting time between every shared
/// library that includes the header, because the linker keeps one such `static` for the
/// whole process. Write it in a source file to keep the sharing inside one library.
#define AW_MPC_LOG(writer, level_name, after_s, ...)                                           \
  do {                                                                                         \
    static double aw_log_last_s = 0.0;                                                         \
    const auto & aw_log_writer = (writer);                                                     \
    using AwLogWriter = std::decay_t<decltype(aw_log_writer)>;                                 \
    static const AwLogWriter::Site aw_log_site{__func__, __FILE__, __LINE__};                  \
    if (aw_log_writer.shouldWrite(AwLogWriter::Level::level_name, (after_s), aw_log_last_s)) { \
      aw_log_writer.write(                                                                     \
        AwLogWriter::Level::level_name, aw_log_site, ::fmt::format(__VA_ARGS__));              \
    }                                                                                          \
  } while (0)

// Only the names below carry the name of this package.
#define AW_MPC_DEBUG(writer, ...) AW_MPC_LOG(writer, debug, 0.0, __VA_ARGS__)
#define AW_MPC_INFO(writer, ...) AW_MPC_LOG(writer, info, 0.0, __VA_ARGS__)
#define AW_MPC_WARN(writer, ...) AW_MPC_LOG(writer, warn, 0.0, __VA_ARGS__)
#define AW_MPC_ERROR(writer, ...) AW_MPC_LOG(writer, error, 0.0, __VA_ARGS__)
#define AW_MPC_FATAL(writer, ...) AW_MPC_LOG(writer, fatal, 0.0, __VA_ARGS__)

#define AW_MPC_DEBUG_THROTTLE(writer, after_s, ...) AW_MPC_LOG(writer, debug, after_s, __VA_ARGS__)
#define AW_MPC_INFO_THROTTLE(writer, after_s, ...) AW_MPC_LOG(writer, info, after_s, __VA_ARGS__)
#define AW_MPC_WARN_THROTTLE(writer, after_s, ...) AW_MPC_LOG(writer, warn, after_s, __VA_ARGS__)
#define AW_MPC_ERROR_THROTTLE(writer, after_s, ...) AW_MPC_LOG(writer, error, after_s, __VA_ARGS__)
#define AW_MPC_FATAL_THROTTLE(writer, after_s, ...) AW_MPC_LOG(writer, fatal, after_s, __VA_ARGS__)

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__LOG_WRITER_HPP_
