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

//
// Created by jgibson37 on 1/9/20.
//

#ifndef MPPI__UTILS__FILE_UTILS_H_
#define MPPI__UTILS__FILE_UTILS_H_

#include <string>
#include <unistd.h>

inline bool fileExists(const std::string& name)
{
  return (access(name.c_str(), F_OK) != -1);
}

#endif  // MPPI__UTILS__FILE_UTILS_H_
