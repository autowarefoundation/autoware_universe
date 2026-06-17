//
// Created by jason on 2/21/24.
//

#ifndef MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_SAMPLER_H_
#define MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_SAMPLER_H_

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
#endif  // MPPIGENERIC_TESTS_INCLUDE_MPPI_TEST_MOCK_CLASSES_MOCK_SAMPLER_H_
