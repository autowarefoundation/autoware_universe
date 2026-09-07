//  Copyright 2025 The Autoware Contributors
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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__DETAIL__OVERLOADED_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__DETAIL__OVERLOADED_HPP_

namespace autoware::redundancy_switcher
{

// C++17 overloaded helper for std::visit with multiple callables.
template <class... Ts>
struct overloaded : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__DETAIL__OVERLOADED_HPP_
