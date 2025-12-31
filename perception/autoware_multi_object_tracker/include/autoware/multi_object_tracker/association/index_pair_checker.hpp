// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__INDEX_PAIR_CHECKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__INDEX_PAIR_CHECKER_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_set>

namespace autoware::multi_object_tracker
{

/// Manages and checks registration of index pairs using a hash-based approach.
/// Efficiently stores and queries pairs of indices (e.g., tracker-measurement pairs).
class IndexPairChecker
{
private:
  std::unordered_set<uint64_t> pair_set_;

  /// Computes a hash key from two indices.
  /// Uses bit shifting to combine the two 32-bit indices into a single 64-bit key.
  static uint64_t computeHashKey(size_t first_idx, size_t second_idx)
  {
    return (static_cast<uint64_t>(first_idx) << 32) | second_idx;
  }

public:
  IndexPairChecker() = default;

  /// Registers an index pair in the set.
  /// @param first_idx The first index
  /// @param second_idx The second index
  void addPair(size_t first_idx, size_t second_idx)
  {
    pair_set_.insert(computeHashKey(first_idx, second_idx));
  }

  /// Checks if an index pair is registered in the set.
  /// @param first_idx The first index
  /// @param second_idx The second index
  /// @return true if the pair is registered, false otherwise
  bool hasPair(size_t first_idx, size_t second_idx) const
  {
    return pair_set_.count(computeHashKey(first_idx, second_idx)) > 0;
  }

  /// Clears all pairs from the set.
  void clear() { pair_set_.clear(); }

  /// Returns the number of pairs in the set.
  /// @return The size of the set
  size_t size() const { return pair_set_.size(); }

  /// Checks if the set is empty.
  /// @return true if the set is empty, false otherwise
  bool empty() const { return pair_set_.empty(); }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__INDEX_PAIR_CHECKER_HPP_
