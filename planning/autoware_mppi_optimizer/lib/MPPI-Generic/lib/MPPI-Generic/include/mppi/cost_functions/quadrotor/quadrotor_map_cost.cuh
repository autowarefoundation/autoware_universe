/*
 * Created on Wed Jul 22 2020 by Bogdan
 */
#ifndef MPPI_COST_FUNCTIONS_QUADROTOR_MAP_COST_CUH_
#define MPPI_COST_FUNCTIONS_QUADROTOR_MAP_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/quadrotor/quadrotor_dynamics.cuh>
#include <mppi/utils/math_utils.h>
#include <mppi/utils/texture_helpers/two_d_texture_helper.cuh>

#include <string>

struct QuadrotorMapCostParams : public CostParams<4>
{
  float3 r_c1;
  float3 r_c2;
  float3 trs;

  float attitude_coeff = 10;
  float crash_coeff = 1000;
  float dist_to_waypoint_coeff = 0.0;
  float heading_coeff = 5;
  float heading_power = 1.0;  // power to take the difference in headings to
  float height_coeff = 5;
  float track_coeff = 10;
  float speed_coeff = 5;
  float track_slop = 0.0;
  float gate_pass_cost = -150;

  float4 curr_waypoint;
  float4 prev_waypoint;
  float3 curr_gate_left;
  float3 curr_gate_right;
  float3 prev_gate_left;
  float3 prev_gate_right;
  float4 end_waypoint;

  // if the costmap cost is above this, we are no longer on the track
  float desired_speed = 5;            // [m/s]
  float gate_margin = 0.5;            // [m]
  float min_dist_to_gate_side = 0.5;  // [m] TODO find correct value for this
  float track_boundary_cost = 2.5;
  float gate_width = 2.15;  // [m]

  QuadrotorMapCostParams()
  {
    control_cost_coeff[0] = 1;  // roll
    control_cost_coeff[1] = 1;  // pitch
    control_cost_coeff[2] = 1;  // yaw
    control_cost_coeff[3] = 1;  // throttle
    curr_waypoint = make_float4(0, 0, 0, 0);
    prev_waypoint = make_float4(0, 0, 0, 0);

    curr_gate_left = make_float3(0, 0, 0);
    curr_gate_right = make_float3(0, 0, 0);
    prev_gate_left = make_float3(0, 0, 0);
    prev_gate_right = make_float3(0, 0, 0);
    end_waypoint = make_float4(NAN, NAN, NAN, NAN);
  }

  bool updateWaypoint(float x, float y, float z, float heading = 0)
  {
    bool changed = false;
    if (curr_waypoint.x != x || curr_waypoint.y != y || curr_waypoint.z != z || curr_waypoint.w != heading)
    {
      prev_waypoint = curr_waypoint;
      curr_waypoint = make_float4(x, y, z, heading);
      float3 curr_left = make_float3(x + cosf(heading) * gate_width, y + sinf(heading) * gate_width, z);
      float3 curr_right = make_float3(x - cosf(heading) * gate_width, y - sinf(heading) * gate_width, z);
      updateGateBoundaries(curr_left.x, curr_left.y, curr_left.z, curr_right.x, curr_right.y, curr_right.z);
      changed = true;
    }
    return changed;
  }

  bool updateGateBoundaries(float left_x, float left_y, float left_z, float right_x, float right_y, float right_z)
  {
    bool changed = false;
    if (curr_gate_left.x != left_x || curr_gate_left.y != left_y || curr_gate_left.z != left_z ||
        curr_gate_right.x != right_x || curr_gate_right.y != right_y || curr_gate_right.z != right_z)
    {
      prev_gate_left = curr_gate_left;
      prev_gate_right = curr_gate_right;
      curr_gate_left = make_float3(left_x, left_y, left_z);
      curr_gate_right = make_float3(right_x, right_y, right_z);
      changed = true;
    }
    return changed;
  }
};

template <class CLASS_T, class PARAMS_T = QuadrotorMapCostParams>
class QuadrotorMapCostImpl : public Cost<CLASS_T, PARAMS_T, QuadrotorDynamicsParams>
{
public:
  // I think these typedefs are needed because this class is itself templated?
  typedef Cost<CLASS_T, PARAMS_T, QuadrotorDynamicsParams> PARENT_CLASS;
  using output_array = typename PARENT_CLASS::output_array;
  using control_array = typename PARENT_CLASS::control_array;
  using OutputIndex = typename PARENT_CLASS::OutputIndex;
  using ControlIndex = typename PARENT_CLASS::ControlIndex;
  using DYN_P = QuadrotorDynamicsParams;

  QuadrotorMapCostImpl(cudaStream_t stream = 0);

  ~QuadrotorMapCostImpl();

  void bindToStream(cudaStream_t stream);

  void GPUSetup();

  void freeCudaMem();

  void paramsToDevice();

  std::string getCostFunctionName() const override
  {
    return std::string("Quadrotor Map Cost");
  }

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep, int* crash_status);

  __device__ float computeStateCost(float* s, int timestep, float* theta_c, int* crash_status);

  float terminalCost(const Eigen::Ref<const output_array> s);

  __device__ float terminalCost(float* s, float* theta_c = nullptr);

  __host__ __device__ float computeGateSideCost(float* s);

  __host__ __device__ float computeHeadingCost(float* s);

  __host__ __device__ float computeHeightCost(float* s);

  __host__ __device__ float computeSpeedCost(float* s);

  __host__ __device__ float computeStabilizingCost(float* s);

  __host__ __device__ float computeWaypointCost(float* s);

  __host__ __device__ float distToWaypoint(float* s, float4 waypoint);

  void updateWaypoint(float4 new_waypoint);
  void updateWaypoint(float x, float y, float z, float heading = 0);

  void updateGateBoundaries(float3 left_side, float3 right_side);
  void updateGateBoundaries(std::vector<float> boundaries);
  void updateGateBoundaries(float left_x, float left_y, float left_z, float right_x, float right_y, float right_z);

  /** =================== Cost Map Related Functions ================== **/
  __device__ float computeCostmapCost(float* s);

  std::vector<float4> loadTrackData(std::string map_path);

  __device__ float4 queryTexture(float x, float y) const;

  /**
   * alters the costmap size in CPU storage and GPU texture
   * @param width
   * @param height
   * @return
   */
  bool changeCostmapSize(int width, int height);

  /**
   * @brief Binds the member variable costmap to a CUDA texture.
   */
  void costmapToTexture();

  /**
   * @brief Updates the current costmap coordinate transform.
   * @param h Matrix representing a transform from world to (offset) costmap coordinates.
   * @param trs Array representing the offset.
   */
  void updateTransform(const Eigen::Ref<const Eigen::Matrix3f>& m, const Eigen::Ref<const Eigen::Array3f>& trs);

  /**
   * @brief Compute a coordinate transform going from world to costmap coordinates.
   */
  __host__ __device__ void coorTransform(float x, float y, float* u, float* v, float* w);

  /**
   * Queries the texture using coorTransform beforehand
   */
  __device__ float4 queryTextureTransformed(float x, float y);

  TwoDTextureHelper<float>* tex_helper_ = nullptr;

protected:
  std::string map_path_;

  // Costmap Related Variables
  std::vector<float4> track_costs_;
  int width_ = -1;
  int height_ = -1;

  cudaArray* costmapArray_d_;          ///< Cuda array for texture binding.
  cudaChannelFormatDesc channelDesc_;  ///< Cuda texture channel description.
  cudaTextureObject_t costmap_tex_d_;  ///< Cuda texture object.
};

class QuadrotorMapCost : public QuadrotorMapCostImpl<QuadrotorMapCost>
{
public:
  QuadrotorMapCost(cudaStream_t stream = 0) : QuadrotorMapCostImpl<QuadrotorMapCost>(stream){};
};

#if __CUDACC__
#include "quadrotor_map_cost.cu"
#endif
#endif  // MPPI_COST_FUNCTIONS_QUADROTOR_MAP_COST_CUH_
