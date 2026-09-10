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

#include "autoware/lidar_centerpoint/ml_package_version.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace autoware::lidar_centerpoint
{
namespace
{
std::string supported_range()
{
  return "This node reads model bundles of version v" +
         std::to_string(SUPPORTED_ML_PACKAGE_MAJOR_VERSION) + "." +
         std::to_string(MINIMUM_ML_PACKAGE_MINOR_VERSION) +
         " or later within the same major version. Update the bundle with the ansible artifacts "
         "role of the autoware repository.";
}

std::string manifest_subject(const std::string & model_path)
{
  return model_path.empty() ? "The model manifest" : "The model manifest in '" + model_path + "'";
}

bool parse_version(const std::string & version, int & major, int & minor)
{
  const std::string digits =
    (!version.empty() && (version.front() == 'v' || version.front() == 'V')) ? version.substr(1)
                                                                             : version;
  const auto separator = digits.find('.');
  if (separator == std::string::npos) {
    return false;
  }

  try {
    std::size_t major_end = 0;
    std::size_t minor_end = 0;
    major = std::stoi(digits.substr(0, separator), &major_end);
    minor = std::stoi(digits.substr(separator + 1), &minor_end);
    return major >= 0 && minor >= 0 && major_end == separator &&
           minor_end == digits.size() - separator - 1;
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

void check_ml_package_version(const std::string & version, const std::string & model_path)
{
  const std::string subject = manifest_subject(model_path);

  if (version.empty()) {
    throw std::invalid_argument(
      subject + " declares no version, so it predates the manifest layout this node requires. " +
      supported_range());
  }

  int major = 0;
  int minor = 0;
  if (!parse_version(version, major, minor)) {
    throw std::invalid_argument(
      subject + " declares version '" + version +
      "', which is not of the form '<major>.<minor>'. " + supported_range());
  }

  if (major != SUPPORTED_ML_PACKAGE_MAJOR_VERSION || minor < MINIMUM_ML_PACKAGE_MINOR_VERSION) {
    throw std::invalid_argument(
      subject + " declares version '" + version + "', which is not supported. " +
      supported_range());
  }
}

}  // namespace autoware::lidar_centerpoint
