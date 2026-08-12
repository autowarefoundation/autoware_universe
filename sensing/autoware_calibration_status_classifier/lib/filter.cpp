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

#include "autoware/calibration_status_classifier/filter.hpp"

#include <string>
#include <vector>

namespace autoware::calibration_status_classifier
{

LinearVelocityFilter::LinearVelocityFilter(double threshold, double timeout_sec)
: ThresholdFilter<double>(threshold, timeout_sec), name_("Linear velocity"), unit_("m/s")
{
}

const std::string & LinearVelocityFilter::name() const
{
  return name_;
}

const std::string & LinearVelocityFilter::unit() const
{
  return unit_;
}

void LinearVelocityFilter::update(const LinearVelocityInfo & info, const double stamp)
{
  const double linear_velocity = std::sqrt(info.x * info.x + info.y * info.y + info.z * info.z);
  ThresholdFilter<double>::update(linear_velocity, stamp);
}

AngularVelocityFilter::AngularVelocityFilter(double threshold, double timeout_sec)
: ThresholdFilter<double>(threshold, timeout_sec), name_("Angular velocity"), unit_("rad/s")
{
}

const std::string & AngularVelocityFilter::name() const
{
  return name_;
}

const std::string & AngularVelocityFilter::unit() const
{
  return unit_;
}

void AngularVelocityFilter::update(const AngularVelocityInfo & info, const double stamp)
{
  const double angular_velocity = std::sqrt(info.x * info.x + info.y * info.y + info.z * info.z);
  ThresholdFilter<double>::update(angular_velocity, stamp);
}

ObjectsFilter::ObjectsFilter(size_t threshold, double timeout_sec)
: ThresholdFilter<size_t>(threshold, timeout_sec), name_("Object count"), unit_()
{
}

const std::string & ObjectsFilter::name() const
{
  return name_;
}

const std::string & ObjectsFilter::unit() const
{
  return unit_;
}

void ObjectsFilter::update(const std::vector<ObjectInfo> & objects, const double stamp)
{
  objects_ = objects;
  ThresholdFilter<size_t>::update(objects_.size(), stamp);
}

const std::vector<ObjectInfo> & ObjectsFilter::objects() const
{
  return objects_;
}

}  // namespace autoware::calibration_status_classifier
