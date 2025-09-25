// Copyright 2025 Instituto de Telecomunições-Porto Branch, Inc. All rights reserved.
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

#ifndef SPHERIC_COLLISION_DETECTOR__SPHERE3_HPP_
#define SPHERIC_COLLISION_DETECTOR__SPHERE3_HPP_

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <vector>

namespace sphere3
{
    class Sphere3{
        public:
            Eigen::Vector3d center_;
            double radius_;
            int tag_;

            std::vector<Eigen::Vector3d> vPts_;
            Sphere3(Eigen::Vector3d center, double radius, int tag);
            void computeSphereCoords(std::vector<Eigen::Vector3d> &pts);

            ~Sphere3();
            void setVelocity(double x, double y, double z);
            void setCenter(double x, double y, double z);

    };
} //namespace sphere3

#endif  // SPHERIC_COLLISION_DETECTOR_SPHERE3_HPP_
