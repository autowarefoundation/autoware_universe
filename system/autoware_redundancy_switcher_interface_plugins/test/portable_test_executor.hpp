//  Copyright 2026 The Autoware Contributors
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
#ifndef PORTABLE_TEST_EXECUTOR_HPP_
#define PORTABLE_TEST_EXECUTOR_HPP_

// Test-only helpers for gtest fixtures that construct an autoware::agnocast_wrapper::Node and
// spin it with a plain rclcpp::Executor.
//
// Two independent runtime gaps make this impossible when the node is actually running in
// Agnocast mode (ENABLE_AGNOCAST=1 at runtime), and both are unconditional — no amount of
// LD_PRELOAD/heaphook setup fixes either one, so tests that hit them must skip rather than run:
//
// 1. Constructing an Agnocast endpoint at all requires the heaphook preloaded
//    (validate_ld_preload() in agnocast_utils.cpp exits the whole process otherwise). Neither
//    colcon test nor ctest sets LD_PRELOAD for a gtest binary.
// 2. Even with the heaphook loaded, spinning an autoware::agnocast_wrapper::Node that picked the
//    Agnocast backend via any *generic* rclcpp::Executor (SingleThreadedExecutor, MultiThreaded,
//    or even agnocast::SingleThreadedAgnocastExecutor added via get_node_base_interface()) throws
//    "get_shared_notify_guard_condition is not yet implemented": that NodeBaseInterface only
//    supports the AgnocastOnly* executors via get_agnocast_node(), which is exactly what
//    autoware_agnocast_wrapper_register_node()'s generated main uses in production for a node
//    built with an AgnocastOnly* AGNOCAST_EXECUTOR (as both RedundancySwitcherInterface and
//    SimpleSwitcherNode are). Reproducing that combination in an ad-hoc gtest fixture is not
//    attempted here; real Agnocast-transport coverage for these nodes comes from launching the
//    actual registered executable (see the dummy-topic/UDS integration tests), not from spinning
//    a hand-built node+executor pair in-process.
//
// Fixtures that only construct a Node without ever adding it to an executor (e.g.
// NonRedundantSwitcherAdapter, which owns no pub/sub) are unaffected by gap 2 and only need to
// guard against gap 1.

#include <cstdlib>
#include <string>

#ifdef USE_AGNOCAST_ENABLED
#include <autoware/agnocast_wrapper/runtime.hpp>
#endif

inline bool agnocast_heaphook_loaded()
{
  const char * ld_preload = std::getenv("LD_PRELOAD");
  return ld_preload != nullptr &&
         std::string(ld_preload).find("libagnocast_heaphook.so") != std::string::npos;
}

// Use for fixtures that construct a Node but never add it to an executor (gap 1 only).
inline bool agnocast_node_construction_untestable()
{
#ifdef USE_AGNOCAST_ENABLED
  return autoware::agnocast_wrapper::use_agnocast() && !agnocast_heaphook_loaded();
#else
  return false;
#endif
}

// Use for fixtures that add a Node to an executor and spin it (gap 2 — unconditional on
// heaphook/LD_PRELOAD, see the file comment above).
inline bool agnocast_executor_spin_untestable()
{
#ifdef USE_AGNOCAST_ENABLED
  return autoware::agnocast_wrapper::use_agnocast();
#else
  return false;
#endif
}

#endif  // PORTABLE_TEST_EXECUTOR_HPP_
