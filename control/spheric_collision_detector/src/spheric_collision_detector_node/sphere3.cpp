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

#include "spheric_collision_detector/sphere3.hpp"

namespace sphere3
{
    Sphere3::Sphere3(Eigen::Vector3d center, double radius, int tag): center_(center), radius_(radius), tag_(tag)
    {
        computeSphereCoords(vPts_);
    }

    void Sphere3::computeSphereCoords(std::vector<Eigen::Vector3d> &pts){
		pts.clear();
        const auto PIOVER = M_PI/25.;
		for (double phi = 0.; phi < 2*M_PI; phi += PIOVER){ // Azimuth [0, 2*PI]
            for (double theta = 0.; theta < M_PI; theta += PIOVER){ // Elevation [0, PI]
                Eigen::Vector3d point;
                point.x() = radius_ * cos(phi) * sin(theta) + center_.x();
                point.y() = radius_ * sin(phi) * sin(theta) + center_.y();
                point.z() = radius_            * cos(theta) + center_.z();
                pts.push_back(point);
            }
    	}
	}
}
