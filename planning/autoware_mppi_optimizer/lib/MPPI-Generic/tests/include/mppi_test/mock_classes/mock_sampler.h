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
// Created by jason on 2/21/24.
//

#ifndef MPPI_TEST__MOCK_CLASSES__MOCK_SAMPLER_H_
#define MPPI_TEST__MOCK_CLASSES__MOCK_SAMPLER_H_

#include <mppi/sampling_distributions/sampling_distribution.cuh>
#include <mppi_test/mock_classes/mock_dynamics.h>

class MockSamplingDistribution
  : public mppi::sampling_distributions::SamplingDistribution<
        MockSamplingDistribution, mppi::sampling_distributions::SamplingParams, mockDynamicsParams>
{
public:
  MOCK_METHOD1(bindToStream, void(cudaStream_t stream));
  MOCK_METHOD0(GPUSetup, void());
  MOCK_METHOD1(resizeVisualizationCotnrolTrajectories, void(bool synchronize));
  MOCK_METHOD1(allocateCUDAMemory, void(bool synchronize));
  MOCK_METHOD0(allocateCUDAMemoryHelper, void());
  MOCK_METHOD0(freeCudaMem, void());
  MOCK_METHOD4(generateSamples,
               void(const int& opt_stride, const int& iteration_num, curandGenerator_t& gen, bool synchronize));
  MOCK_METHOD3(setHostOptimalControlSequence,
               void(float* optimal_control_trajectory, const int& distribution_idx, bool synchronize));
  MOCK_METHOD4(updateDistributionParamsFromDevice,
               void(const float* trajectory_weights_d, float normalizer, const int& distribution_i, bool synchronize));
};
#endif  // MPPI_TEST__MOCK_CLASSES__MOCK_SAMPLER_H_
