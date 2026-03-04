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

#include "test_plot_utils.hpp"

#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#ifdef EXPORT_TEST_PLOT_FIGURE
namespace
{
std::string to_snake_case(const std::string & str)
{
  std::string result;
  for (size_t i = 0; i < str.size(); ++i) {
    if (std::isupper(str[i])) {
      if (i > 0 && std::islower(str[i - 1])) {
        result += '_';
      }
      result += std::tolower(str[i]);
    } else {
      result += str[i];
    }
  }
  return result;
}
}  // namespace
#endif

namespace autoware::boundary_departure_checker
{
#ifdef EXPORT_TEST_PLOT_FIGURE
static pybind11::scoped_interpreter guard{};

void save_figure(autoware::pyplot::PyPlot & plt, const std::string & sub_dir)
{
  const std::string file_path = __FILE__;
  const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();

  if (!test_info) return;

  // 1. Get the raw test name (e.g., "test_create_vehicle_footprints/NonZeroMargin")
  std::string test_name = test_info->name();

  // 2. Replace '/' with '_' to prevent FileNotFoundError in Python
  std::replace(test_name.begin(), test_name.end(), '/', '_');

  // 3. Convert to snake_case and add extension
  std::string filename = to_snake_case(test_name) + ".png";

  size_t pos = file_path.rfind(TEST_PACKAGE_NAME);
  if (pos != std::string::npos) {
    std::string output_path = file_path.substr(0, pos) + TEST_PACKAGE_NAME + "/test_results/";

    if (!sub_dir.empty()) {
      output_path += sub_dir + "/";
    }

    std::filesystem::create_directories(output_path);
    // Use the passed plt reference
    plt.savefig(Args(output_path + filename), Kwargs("dpi"_a = 150));
    plt.clf();
  }
}
#endif
}  // namespace autoware::boundary_departure_checker
