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

#ifndef AUTOWARE__CANDIDATE_BANK__EXACT_PAIR_QUEUE_HPP_
#define AUTOWARE__CANDIDATE_BANK__EXACT_PAIR_QUEUE_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
namespace autoware::candidate_bank
{
// Generation stamps, never receive time. A pair can be consumed at most once.
template <class A, class B>
class ExactPairQueue
{
public:
  explicit ExactPairQueue(std::size_t capacity) : capacity_(capacity)
  {
    if (!capacity) throw std::invalid_argument("zero pair capacity");
  }
  void first(int64_t stamp, A value)
  {
    if (stamp > last_) {
      a_[stamp] = std::move(value);
      trim(a_);
    }
  }
  void second(int64_t stamp, B value)
  {
    if (stamp > last_) {
      b_[stamp] = std::move(value);
      trim(b_);
    }
  }
  std::optional<std::pair<A, B>> pop()
  {
    for (auto it = a_.begin(); it != a_.end(); ++it) {
      auto jt = b_.find(it->first);
      if (jt == b_.end()) continue;
      last_ = it->first;
      auto out = std::make_pair(std::move(it->second), std::move(jt->second));
      a_.erase(a_.begin(), a_.upper_bound(last_));
      b_.erase(b_.begin(), b_.upper_bound(last_));
      return out;
    }
    return std::nullopt;
  }
  void clear()
  {
    a_.clear();
    b_.clear();
    last_ = -1;
  }
  std::size_t pending() const { return a_.size() + b_.size(); }

private:
  template <class M>
  void trim(M & m)
  {
    while (m.size() > capacity_) m.erase(m.begin());
  }
  std::size_t capacity_;
  int64_t last_{-1};
  std::map<int64_t, A> a_;
  std::map<int64_t, B> b_;
};
}  // namespace autoware::candidate_bank

#endif  // AUTOWARE__CANDIDATE_BANK__EXACT_PAIR_QUEUE_HPP_
