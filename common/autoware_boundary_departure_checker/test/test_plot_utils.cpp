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
void save_figure(const std::string & sub_dir)
{
  auto plt = autoware::pyplot::import();
  const std::string file_path = __FILE__;

  const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
  std::string filename =
    test_info ? to_snake_case(std::string(test_info->name())) + ".png" : "unknown_test.png";

  size_t pos = file_path.rfind(TEST_PACKAGE_NAME);
  if (pos != std::string::npos) {
    std::string output_path = file_path.substr(0, pos) + TEST_PACKAGE_NAME + "/test_results/";

    if (!sub_dir.empty()) {
      output_path += sub_dir + "/";
    }

    std::filesystem::create_directories(output_path);
    plt.savefig(Args(output_path + filename), Kwargs("dpi"_a = 150));
    plt.clf();
  }
}
#endif
}  // namespace autoware::boundary_departure_checker
