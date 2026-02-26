// Copyright 2026 Tier IV, Inc.
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

#include "autoware/tensorrt_yolox/label.hpp"

#include <experimental/filesystem>

#include <assert.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::tensorrt_yolox
{
void trimLeft(std::string & s)
{
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

void trimRight(std::string & s)
{
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string trim(std::string & s)
{
  trimLeft(s);
  trimRight(s);
  return s;
}

bool fileExists(const std::string & file_name, bool verbose)
{
  if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
    if (verbose) {
      std::cout << "File does not exist : " << file_name << std::endl;
    }
    return false;
  }
  return true;
}

std::vector<std::string> loadListFromTextFile(const std::string & filename)
{
  assert(fileExists(filename, true));
  std::vector<std::string> list;

  std::ifstream f(filename);
  if (!f) {
    std::cout << "failed to open " << filename << std::endl;
    assert(0);
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    } else {
      list.push_back(trim(line));
    }
  }

  return list;
}

std::vector<std::string> loadImageList(const std::string & filename, const std::string & prefix)
{
  std::vector<std::string> fileList = loadListFromTextFile(filename);
  for (auto & file : fileList) {
    if (fileExists(file, false)) {
      continue;
    } else {
      std::string prefixed = prefix + file;
      if (fileExists(prefixed, false))
        file = prefixed;
      else
        std::cerr << "WARNING: couldn't find: " << prefixed << " while loading: " << filename
                  << std::endl;
    }
  }
  return fileList;
}

}  // namespace autoware::tensorrt_yolox
