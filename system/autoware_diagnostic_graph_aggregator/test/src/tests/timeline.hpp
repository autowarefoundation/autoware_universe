// Copyright 2025 The Autoware Contributors
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

#ifndef TESTS__TIMELINE_HPP_
#define TESTS__TIMELINE_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace autoware::diagnostic_graph_aggregator
{

class TimelineTest
{
public:
  TimelineTest();
  void execute(const std::string & path);
  void set_interval(double interval);
  void set_reset(const std::unordered_set<size_t> & steps);
  void set(const std::string & name, const std::string & levels);
  std::string get(const std::string & path);

private:
  double interval_;
  std::unordered_map<std::string, std::string> input_;
  std::unordered_map<std::string, std::string> output_;
  std::unordered_set<size_t> reset_steps_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // TESTS__TIMELINE_HPP_
