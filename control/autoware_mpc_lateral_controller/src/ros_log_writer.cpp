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

#include "autoware/mpc_lateral_controller/ros_log_writer.hpp"

#include <rcutils/logging.h>

#include <string_view>

namespace autoware::motion::control::mpc_lateral_controller
{
bool RosLogWriter::shouldWrite(const Level level, const double after_s, double & last_sent_s) const
{
  if (!rcutils_logging_logger_is_enabled_for(logger_.get_name(), severityOf(level))) {
    return false;
  }
  if (after_s <= 0.0) {
    return true;
  }

  const double now = clock_->now().seconds();
  if (now < last_sent_s + after_s) {
    return false;
  }
  last_sent_s = now;
  return true;
}

void RosLogWriter::write(const Level level, const Site & site, const std::string_view line) const
{
  RCUTILS_LOGGING_AUTOINIT;
  const rcutils_log_location_t location{site.function_name, site.file_name, site.line_number};
  rcutils_log(
    &location, severityOf(level), logger_.get_name(), "%.*s", static_cast<int>(line.size()),
    line.data());
}

int RosLogWriter::severityOf(const Level level)
{
  switch (level) {
    case Level::debug:
      return RCUTILS_LOG_SEVERITY_DEBUG;
    case Level::info:
      return RCUTILS_LOG_SEVERITY_INFO;
    case Level::warn:
      return RCUTILS_LOG_SEVERITY_WARN;
    case Level::error:
      return RCUTILS_LOG_SEVERITY_ERROR;
    case Level::fatal:
      return RCUTILS_LOG_SEVERITY_FATAL;
  }
  return RCUTILS_LOG_SEVERITY_DEBUG;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
