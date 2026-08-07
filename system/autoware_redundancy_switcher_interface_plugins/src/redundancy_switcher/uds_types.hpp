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
#ifndef REDUNDANCY_SWITCHER__UDS_TYPES_HPP_
#define REDUNDANCY_SWITCHER__UDS_TYPES_HPP_

#include <nlohmann/json.hpp>

#include <cstdint>
#include <vector>

namespace autoware::redundancy_switcher
{

// Interface → Switcher
struct ElectionRequest
{
  bool self_fault_request{false};
  bool reset_request{false};
  uint16_t priority{0};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ElectionRequest, self_fault_request, reset_request, priority)

// Switcher → Interface
//
// node_state values and their SwitcherSignals mapping:
//   0 INITIALIZING       all false  (transitional)
//   1 ELECTABLE          is_stable  = true
//   2 WAIT_FOR_AUTOWARE  all false  (transitional)
//   3 IN_ELECTION        all false  (transitional)
//   4 ELECTION_COMPLETED is_stable  = true
//   5 ELECTION_UNCLOSED  is_faulted = true
//   6 PATH_NOT_FOUND     is_faulted = true
//   7 SELF_INTERRUPTION  is_self_interrupted = true
//
// path_info: bit-field (bit0=main_ecu, bit1=sub_ecu, bit2=main_vcu, bit3=sub_vcu)
struct ElectionStatus
{
  uint8_t node_state{0};
  uint8_t node_id{0};
  uint8_t leader_id{0};
  uint8_t path_info{0};
  bool main_ecu_to_main_ecu_connected{false};
  bool main_ecu_to_sub_ecu_connected{false};
  bool main_ecu_to_main_vcu_connected{false};
  bool main_ecu_to_sub_vcu_connected{false};
  bool sub_ecu_to_main_ecu_connected{false};
  bool sub_ecu_to_sub_ecu_connected{false};
  bool sub_ecu_to_main_vcu_connected{false};
  bool sub_ecu_to_sub_vcu_connected{false};
  bool main_vcu_to_main_ecu_connected{false};
  bool main_vcu_to_sub_ecu_connected{false};
  bool main_vcu_to_main_vcu_connected{false};
  bool main_vcu_to_sub_vcu_connected{false};
  bool sub_vcu_to_main_ecu_connected{false};
  bool sub_vcu_to_sub_ecu_connected{false};
  bool sub_vcu_to_main_vcu_connected{false};
  bool sub_vcu_to_sub_vcu_connected{false};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
  ElectionStatus, node_state, node_id, leader_id, path_info, main_ecu_to_main_ecu_connected,
  main_ecu_to_sub_ecu_connected, main_ecu_to_main_vcu_connected, main_ecu_to_sub_vcu_connected,
  sub_ecu_to_main_ecu_connected, sub_ecu_to_sub_ecu_connected, sub_ecu_to_main_vcu_connected,
  sub_ecu_to_sub_vcu_connected, main_vcu_to_main_ecu_connected, main_vcu_to_sub_ecu_connected,
  main_vcu_to_main_vcu_connected, main_vcu_to_sub_vcu_connected, sub_vcu_to_main_ecu_connected,
  sub_vcu_to_sub_ecu_connected, sub_vcu_to_main_vcu_connected, sub_vcu_to_sub_vcu_connected)

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER__UDS_TYPES_HPP_
