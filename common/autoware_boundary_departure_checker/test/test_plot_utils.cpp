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

#include <autoware/pyplot/patches.hpp>
#include <autoware/pyplot/pyplot.hpp>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <filesystem>
#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{
void save_figure(
  [[maybe_unused]] const std::string & filename, [[maybe_unused]] const std::string & sub_dir)
{
#ifdef EXPORT_TEST_PLOT_FIGURE
  auto plt = pyplot::import();
  const std::string file_path = __FILE__;

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
#endif
}
}  // namespace autoware::boundary_departure_checker
