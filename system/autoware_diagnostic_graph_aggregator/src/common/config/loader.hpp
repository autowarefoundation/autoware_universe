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

#ifndef COMMON__CONFIG__LOADER_HPP_
#define COMMON__CONFIG__LOADER_HPP_

#include "types/forward.hpp"
#include "utils/logger.hpp"
#include "utils/memory.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class ConfigLoader
{
public:
  explicit ConfigLoader(std::shared_ptr<Logger> logger);
  ~ConfigLoader();

  // Overall execution for normal use
  void load(const std::string & path);
  GraphData take();
  static GraphData Load(const std::string & path, std::shared_ptr<Logger> logger);

  // Step execution for debug tools.
  void load_file_tree(const std::string & path);
  void make_node_units();
  void make_diag_units();
  void resolve_links();
  void topological_sort();

  void apply_remove_edits();
  void finalize();
  void validate();

  // Getters for debug tools.
  FileData * root() const { return root_file_; }
  std::vector<FileData *> files() const { return raws(files_); }
  std::vector<NodeUnit *> nodes() const { return raws(nodes_); }
  std::vector<DiagUnit *> diags() const { return raws(diags_); }
  std::vector<LinkPort *> ports() const { return raws(ports_); }

private:
  FileData * load_file(const FileContext & context, const std::string & path);
  BaseUnit * load_unit(ConfigYaml yaml);
  BaseUnit * load_diag(ConfigYaml yaml, const std::string & name);

  std::shared_ptr<Logger> logger_;
  std::vector<std::unique_ptr<FileData>> files_;
  std::vector<std::unique_ptr<TempUnit>> temps_;
  std::vector<std::unique_ptr<LinkUnit>> links_;
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<LinkPort>> ports_;
  FileData * root_file_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__LOADER_HPP_
