// Copyright 2026 The Autoware Foundation
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

#ifndef TOP_MEMORY_PARSER_HPP_
#define TOP_MEMORY_PARSER_HPP_

#include "unit_conversions.hpp"

#include <cctype>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace autoware::component_monitor
{

/**
 * @brief Parse RES memory string produced by procps `top` and convert to bytes.
 *
 * `top` may print memory in two formats:
 * - With a suffix (e.g., "25.3m", "1.2g") according to the task-area scaling (`-e`).
 * - Without a suffix (e.g., "29548"). In that case, the value is in KiB.
 *
 * The current component monitor uses `top -b -n 1 -E k -p <pid>`; note `-E` only affects the
 * summary section, while the task area (where RES is shown) follows `-e` scaling/defaults.
 */
inline std::uint64_t parse_top_res_to_bytes(const std::string & mem_res)
{
  if (mem_res.empty()) {
    throw std::runtime_error("Empty RES string");
  }

  // example 1: 12.3g
  // example 2: 25944 (without suffix, KiB)
  static const std::unordered_map<char, std::function<std::uint64_t(double)>> unit_map{
    {'k', unit_conversions::kib_to_bytes<double>}, {'m', unit_conversions::mib_to_bytes<double>},
    {'g', unit_conversions::gib_to_bytes<double>}, {'t', unit_conversions::tib_to_bytes<double>},
    {'p', unit_conversions::pib_to_bytes<double>}, {'e', unit_conversions::eib_to_bytes<double>},
  };

  const unsigned char last_uc = static_cast<unsigned char>(mem_res.back());
  if (std::isdigit(last_uc)) {
    // procps `top` prints RES without a suffix in KiB (not bytes).
    return unit_conversions::kib_to_bytes(std::stoull(mem_res));
  }

  const double value = std::stod(mem_res.substr(0, mem_res.size() - 1));
  const char suffix = static_cast<char>(std::tolower(last_uc));

  const auto it = unit_map.find(suffix);
  if (it != unit_map.end()) {
    return it->second(value);
  }

  throw std::runtime_error("Unsupported unit suffix: " + std::string(1, mem_res.back()));
}

}  // namespace autoware::component_monitor

#endif  // TOP_MEMORY_PARSER_HPP_
