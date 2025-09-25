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

#ifndef SPHERIC_COLLISION_DETECTOR__VEC3_HPP_
#define SPHERIC_COLLISION_DETECTOR__VEC3_HPP_

#include <cmath>
#include <iostream>

using std::sqrt;

class vec3 {
  public:
    double x_, y_, z_;

    vec3() : x_(0), y_(0), z_(0) {}
    vec3(double x, double y, double z) : x_(x), y_(y), z_(z) {}

    double x() const { return x_;}
    double y() const { return y_;}
    double z() const { return z_;}
};

#endif


