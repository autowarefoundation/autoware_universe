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

//
// Created by jason on 4/14/20.
//

#ifndef MPPI_TEST__MOCK_CLASSES__MOCK_COSTS_H_
#define MPPI_TEST__MOCK_CLASSES__MOCK_COSTS_H_

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mppi/cost_functions/cost.cuh>

// ===== mock cost ====
typedef struct
{
  int test = 1;
} mockCostParams;

class MockCost : public Cost<MockCost, mockCostParams, DynamicsParams>
{
public:
  MOCK_METHOD1(bindToStream, void(cudaStream_t stream));
  MOCK_METHOD1(setParams, void(mockCostParams params));
  MOCK_METHOD0(getParams, mockCostParams());
  MOCK_METHOD0(GPUSetup, void());
  MOCK_METHOD0(freeCudaMem, void());
};
#endif  // MPPI_TEST__MOCK_CLASSES__MOCK_COSTS_H_
