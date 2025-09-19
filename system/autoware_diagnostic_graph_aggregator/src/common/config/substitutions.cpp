// Copyright 2024 The Autoware Contributors
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

#include "config/substitutions.hpp"

#include "config/entity.hpp"
#include "config/errors.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator::substitutions
{

std::string resolve(const std::string & substitution, const FileContext & context)
{
  std::stringstream ss(substitution);
  std::vector<std::string> words;
  {
    std::string word;
    while (std::getline(ss, word, ' ')) {
      words.push_back(word);
    }
  }

  if (words.size() == 2 && words[0] == "find-pkg-share") {
    return ament_index_cpp::get_package_share_directory(words[1]);
  }
  if (words.size() == 1 && words[0] == "dirname") {
    return std::filesystem::path(context.file).parent_path();
  }
  throw UnknownSubstitution(substitution);
}

std::string evaluate(const std::string & text, const FileContext & context)
{
  static const std::regex pattern(R"(\$\(([^()]*)\))");
  std::smatch m;
  std::string result = text;
  while (std::regex_search(result, m, pattern)) {
    const std::string prefix = m.prefix();
    const std::string suffix = m.suffix();
    result = prefix + resolve(m.str(1), context) + suffix;
  }
  return result;
}

}  // namespace autoware::diagnostic_graph_aggregator::substitutions
