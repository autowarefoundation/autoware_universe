// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE_DRIVING_MODE_MANAGER__TYPES_HPP_
#define AUTOWARE_DRIVING_MODE_MANAGER__TYPES_HPP_

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::driving_mode_manager
{

struct TrajectorySource
{
  uint32_t id;
  bool operator==(const TrajectorySource & another) const { return id == another.id; }
  bool operator!=(const TrajectorySource & another) const { return id != another.id; }
};

struct CommandSource
{
  uint32_t id;
  bool operator==(const CommandSource & another) const { return id == another.id; }
  bool operator!=(const CommandSource & another) const { return id != another.id; }
};

struct CommandFilter
{
  bool flag;
  bool operator==(const CommandFilter & another) const { return flag == another.flag; }
  bool operator!=(const CommandFilter & another) const { return flag != another.flag; }
};

struct AutowareMode
{
  uint32_t id;
  bool operator==(const AutowareMode & another) const { return id == another.id; }
  bool operator!=(const AutowareMode & another) const { return id != another.id; }
};

struct MrmBehavior
{
  uint16_t id;
  bool operator==(const MrmBehavior & another) const { return id == another.id; }
  bool operator!=(const MrmBehavior & another) const { return id != another.id; }
};

enum class PlatformMode {
  kUnknown,
  kAutoware,
  kAutowareSteering,
  kAutowareVelocity,
  kManual,
};

enum class OperationMode {
  kUnknown,
  kStop,
  kAutonomous,
  kLocal,
  kRemote,
};

enum class AutowareControl {
  kUnknown,
  kEnable,
  kDisable,
};

enum class MrmStrategy {
  kUnknown,
  kNone,
  kDelegate,
  kBehavior,
};

struct RequestModes
{
  AutowareMode operation_mode;  // Operation mode API
  PlatformMode platform_mode;   // Operation mode API
  MrmStrategy mrm_strategy;     // MRM request API
  AutowareMode mrm_behavior;    // MRM request API
  AutowareMode autoware_mode;   // Decision logic plugin.
};

using AutowareModeSet = std::unordered_set<AutowareMode>;

}  // namespace autoware::driving_mode_manager

namespace std
{

template <>
struct hash<autoware::driving_mode_manager::AutowareMode>
{
  size_t operator()(const autoware::driving_mode_manager::AutowareMode & mode) const
  {
    return hash<uint32_t>{}(mode.id);
  }
};

template <>
struct hash<autoware::driving_mode_manager::MrmBehavior>
{
  size_t operator()(const autoware::driving_mode_manager::MrmBehavior & behavior) const
  {
    return hash<uint16_t>{}(behavior.id);
  }
};

template <>
struct hash<autoware::driving_mode_manager::TrajectorySource>
{
  size_t operator()(const autoware::driving_mode_manager::TrajectorySource & source) const
  {
    return hash<uint32_t>{}(source.id);
  }
};

template <>
struct hash<autoware::driving_mode_manager::CommandSource>
{
  size_t operator()(const autoware::driving_mode_manager::CommandSource & source) const
  {
    return hash<uint32_t>{}(source.id);
  }
};

}  // namespace std

#endif  // AUTOWARE_DRIVING_MODE_MANAGER__TYPES_HPP_
