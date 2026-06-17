#include <mppi/instantiations/quadrotor_mppi/quadrotor_mppi.cuh>

typedef mppi::sampling_distributions::GaussianDistribution<QuadrotorDynamics::DYN_PARAMS_T> Sampler;

template class VanillaMPPIController<QuadrotorDynamics, QuadrotorQuadraticCost, DDPFeedback<QuadrotorDynamics, 100>,
                                     100, 512, Sampler>;
template class VanillaMPPIController<QuadrotorDynamics, QuadrotorMapCost, DDPFeedback<QuadrotorDynamics, 100>, 100, 512,
                                     Sampler>;
