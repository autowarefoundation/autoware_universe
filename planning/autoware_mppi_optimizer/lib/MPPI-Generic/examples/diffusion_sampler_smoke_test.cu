/** Compile-time smoke test for DiffusionDistribution (links MPPI + diffusion headers). */
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/sampling_distributions/diffusion/diffusion_distribution.cuh>

int main(int argc, char** argv)
{
  using Sampler = mppi::sampling_distributions::DiffusionDistribution<FirstOrderDubinsBicycleParams>;
  Sampler::SAMPLING_PARAMS_T params(64, 80);
  params.num_rollouts = 64;
  params.num_timesteps = 80;
  Sampler sampler(params);
  if (argc > 1)
  {
    sampler.setWeightsPath(argv[1]);
  }
  sampler.GPUSetup();
  return 0;
}
