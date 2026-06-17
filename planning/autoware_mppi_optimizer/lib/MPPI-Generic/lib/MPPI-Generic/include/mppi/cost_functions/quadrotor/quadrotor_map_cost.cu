#include <cnpy.h>
#include <mppi/utils/file_utils.h>
#include <mppi/cost_functions/quadrotor/quadrotor_map_cost.cuh>
#include <mppi/utils/cuda_math_utils.cuh>

template <class CLASS_T, class PARAMS_T>
QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::QuadrotorMapCostImpl(cudaStream_t stream)
{
  tex_helper_ = new TwoDTextureHelper<float>(1, stream);
  this->bindToStream(stream);
}
template <class CLASS_T, class PARAMS_T>
QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::~QuadrotorMapCostImpl()
{
  delete tex_helper_;
}
template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::bindToStream(cudaStream_t stream)
{
  PARENT_CLASS::bindToStream(stream);
  tex_helper_->bindToStream(stream);
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    // HANDLE_ERROR(cudaFreeArray(costmapArray_d_));
    // HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_d_));
    tex_helper_->freeCudaMem();
  }
  PARENT_CLASS::freeCudaMem();
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::GPUSetup()
{
  PARENT_CLASS* derived = static_cast<PARENT_CLASS*>(this);
  tex_helper_->GPUSetup();
  derived->GPUSetup();
  HANDLE_ERROR(cudaMemcpyAsync(&(this->cost_d_->tex_helper_), &(tex_helper_->ptr_d_), sizeof(TwoDTextureHelper<float>*),
                               cudaMemcpyHostToDevice, this->stream_));
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::paramsToDevice()
{
  if (this->GPUMemStatus_)
  {
    // HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->params_, &this->params_, sizeof(PARAMS_T), cudaMemcpyHostToDevice,
    //                              this->stream_));
    // HANDLE_ERROR(
    //     cudaMemcpyAsync(&this->cost_d_->width_, &width_, sizeof(float), cudaMemcpyHostToDevice, this->stream_));
    // HANDLE_ERROR(
    //     cudaMemcpyAsync(&this->cost_d_->height_, &height_, sizeof(float), cudaMemcpyHostToDevice, this->stream_));
    // HANDLE_ERROR(cudaStreamSynchronize(this->stream_));
    tex_helper_->copyToDevice();
  }
  PARENT_CLASS::paramsToDevice();
}

template <class CLASS_T, class PARAMS_T>
float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                                                int* crash_status)
{
  // TODO query texture on CPU
  float cost = 0;
  cost += computeGateSideCost(s.data());
  cost += computeHeightCost(s.data());
  cost += computeHeadingCost(s.data());
  cost += computeSpeedCost(s.data());
  cost += computeStabilizingCost(s.data());
  cost += computeWaypointCost(s.data());

  // Decrease cost if we pass a gate
  float dist_to_gate = distToWaypoint(s.data(), this->params_.curr_waypoint);

  if (dist_to_gate < this->params_.gate_margin)
  {
    // if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 &&
    //     blockIdx.z == 0)
    // {
    //   printf("Passing through a gate: state_vec: (%f, %f, %f)\n", s[O_IND_CLASS(DYN_T::PARAMS_T, POS_X)],
    //   s[O_IND_CLASS(typename DYN_T::PARAMS_T, POS_Y), s[O_IND_CLASS(DYN_T::PARAMS_T, POS_Z)]]);
    // }
    cost += this->params_.gate_pass_cost;
  }
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeStateCost(float* s, int timestep, float* theta_c,
                                                                           int* crash_status)
{
  float cost = 0;
  float costmap_cost, gate_cost, height_cost, heading_cost, speed_cost, stable_cost;
  float waypoint_cost;
  costmap_cost = computeCostmapCost(s);
  gate_cost = computeGateSideCost(s);
  height_cost = computeHeightCost(s);
  heading_cost = computeHeadingCost(s);
  speed_cost = computeSpeedCost(s);
  stable_cost = computeStabilizingCost(s);
  waypoint_cost = computeWaypointCost(s);
  if (gate_cost != 0)
  {
    *crash_status = 1;
    if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 &&
        blockIdx.z == 0)
    {
      printf("hitting the gate?\n");
    }
  }

  // if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 && blockIdx.z ==
  // 0 && timestep == 1)
  // {
  //   // if (isnan(costmap_cost) || isnan(gate_cost) || isnan(height_cost) || isnan(heading_cost) || isnan(speed_cost)
  //   //     ||
  //   //     isnan(stable_cost) || isnan(waypoint_cost))
  //   // {
  //     printf(
  //         "Costs rollout %d: Costmap %5.2f, Gate %5.2f, Height %5.2f,"
  //         " Heading %5.2f, Speed %5.2f, Stabilization %5.2f, Waypoint %5.2f\n", timestep,
  //         costmap_cost, gate_cost, height_cost, heading_cost, speed_cost, stable_cost, waypoint_cost);
  //   // }
  //   // printf("Costs %d: height - %5.2f, stable - %5.2f prev_height %5.2f, cur_height: %5.2f\n", timestep,
  //   //        height_cost,
  //   //        stable_cost, this->params_.prev_waypoint.z, this->params_.curr_waypoint.z);
  // }

  cost += costmap_cost + gate_cost + height_cost + heading_cost + speed_cost + stable_cost;

  // Decrease cost if we pass a gate
  float dist_to_gate = distToWaypoint(s, this->params_.curr_waypoint);

  if (dist_to_gate < this->params_.gate_margin)
  {
    cost += this->params_.gate_pass_cost;
  }
  cost += *crash_status * this->params_.crash_coeff;
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::distToWaypoint(float* s, float4 waypoint)
{
  float dist = sqrt(SQ(s[E_INDEX(OutputIndex, POS_X)] - waypoint.x) + SQ(s[E_INDEX(OutputIndex, POS_Y)] - waypoint.y) +
                    SQ(s[E_INDEX(OutputIndex, POS_Z)] - waypoint.z));

  return dist;
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateWaypoint(float4 new_waypoint)
{
  updateWaypoint(new_waypoint.x, new_waypoint.y, new_waypoint.z, new_waypoint.w);
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateWaypoint(float x, float y, float z, float heading)
{
  if (this->params_.updateWaypoint(x, y, z, heading))
  {
    paramsToDevice();
  }
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateGateBoundaries(float3 left_side, float3 right_side)
{
  updateGateBoundaries(left_side.x, left_side.y, left_side.z, right_side.x, right_side.y, right_side.z);
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateGateBoundaries(std::vector<float> boundaries)
{
  if (boundaries.size() < 6)
  {
    std::cerr << "You need " << 6 - boundaries.size() << " more floats in the"
              << " call to updateGateBoundaries" << std::endl;
    return;
  }
  updateGateBoundaries(boundaries[0], boundaries[1], boundaries[2], boundaries[3], boundaries[4], boundaries[5]);
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateGateBoundaries(float left_x, float left_y, float left_z,
                                                                   float right_x, float right_y, float right_z)
{
  if (this->params_.updateGateBoundaries(left_x, left_y, left_z, right_x, right_y, right_z))
  {
    paramsToDevice();
  }
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeStabilizingCost(float* s)
{
  float cost = 0;
  float roll, pitch, yaw;
  mppi::math::Quat2EulerNWU(&s[6], roll, pitch, yaw);

  float quat_dist_from_level = SQ(roll) + SQ(pitch);
  cost += this->params_.attitude_coeff * quat_dist_from_level;
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeHeadingCost(float* s)
{
  float cost = 0;
  // float roll, pitch, yaw;
  // mppi::math::Quat2EulerNWU(&s[6], roll, pitch, yaw);
  float R[3][3];
  mppi::math::Quat2DCM(&s[E_INDEX(OutputIndex, QUAT_W)], R);
  const float& vx = s[E_INDEX(OutputIndex, VEL_X)];
  const float& vy = s[E_INDEX(OutputIndex, VEL_Y)];
  const float& vz = s[E_INDEX(OutputIndex, VEL_Z)];
  float3 w_v = make_float3(R[0][0] * vx + R[0][1] * vy + R[0][2] * vz, R[1][0] * vx + R[1][1] * vy + R[1][2] * vz,
                           R[2][0] * vx + R[2][1] * vy + R[2][2] * vz);
  float yaw = atan2f(w_v.y, w_v.x);

  // Calculate heading to gate
  float wx = this->params_.curr_waypoint.x - s[E_INDEX(OutputIndex, POS_X)];
  float wy = this->params_.curr_waypoint.y - s[E_INDEX(OutputIndex, POS_Y)];
  float w_heading = atan2f(wy, wx);

  float dist_to_gate = distToWaypoint(s, this->params_.curr_waypoint);
  // Far away from the gate, we want to be pointing at the gate
  if (dist_to_gate > this->params_.gate_margin)
  {
    cost += this->params_.heading_coeff *
            powf(fabsf(angle_utils::shortestAngularDistance(w_heading, yaw)), this->params_.heading_power);
  }
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeSpeedCost(float* s)
{
  float cost = 0;
  float speed = sqrt(s[E_INDEX(OutputIndex, VEL_X)] * s[E_INDEX(OutputIndex, VEL_X)] +
                     s[E_INDEX(OutputIndex, VEL_Y)] * s[E_INDEX(OutputIndex, VEL_Y)]);
  float desired_speed = this->params_.desired_speed;
  // if (this->params_.curr_waypoint == this->params_.end_waypoint)
  // {
  //   desired_speed = distToWaypoint(s, this->params_.curr_waypoint);
  // }
  cost = this->params_.speed_coeff * SQ(speed - desired_speed);
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeWaypointCost(float* s)
{
  float cost = 0;
  float dist_to_gate = distToWaypoint(s, this->params_.curr_waypoint);
  cost = this->params_.dist_to_waypoint_coeff * SQ(dist_to_gate);
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeGateSideCost(float* s)
{
  float cost = 0;
  float2 curr_left = make_float2(this->params_.curr_gate_left.x, this->params_.curr_gate_left.y);
  float2 curr_right = make_float2(this->params_.curr_gate_right.x, this->params_.curr_gate_right.y);
  // Calculate the side border cost
  float dist_to_left_side =
      sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - curr_left.x) + SQ(s[E_INDEX(OutputIndex, POS_Y)] - curr_left.y));
  float dist_to_right_side =
      sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - curr_right.x) + SQ(s[E_INDEX(OutputIndex, POS_Y)] - curr_right.y));

  float prev_dist_to_left_side = sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - this->params_.prev_gate_left.x) +
                                       SQ(s[E_INDEX(OutputIndex, POS_Y)] - this->params_.prev_gate_left.y));
  float prev_dist_to_right_side = sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - this->params_.prev_gate_right.x) +
                                        SQ(s[E_INDEX(OutputIndex, POS_Y)] - this->params_.prev_gate_right.y));

  float2 gate_vec = curr_left - curr_right;
  float2 state_vec_right = make_float2(s[E_INDEX(OutputIndex, POS_X)], s[E_INDEX(OutputIndex, POS_Y)]) - curr_right;
  float2 state_vec_left = make_float2(s[E_INDEX(OutputIndex, POS_X)], s[E_INDEX(OutputIndex, POS_Y)]) - curr_left;
  const float perp_dist = cross(state_vec_right, gate_vec);
  const float comp_state_along_gate_right = dot(state_vec_right, gate_vec) / dot(gate_vec, gate_vec);
  const float threshold = 0.5;
  const float comp_state_along_gate_left = dot(state_vec_left, -gate_vec) / dot(gate_vec, gate_vec);
  const float outside_gate = fmaxf(comp_state_along_gate_left, comp_state_along_gate_right);
  // Find the side closest to the current state
  // const float closest_side_dist =
  //     fminf(dist_to_left_side, fminf(dist_to_right_side, fminf(prev_dist_to_left_side, prev_dist_to_right_side)));
  // if (fabsf(perp_dist) < this->params_.min_dist_to_gate_side &&
  //     fmaxf(comp_state_along_gate_left, comp_state_along_gate_right) > 1.0f)
  // {  // Within perpendicular distance of min_dist and outside of gate
  //   cost += this->params_.crash_coeff * fmaxf(comp_state_along_gate_left, comp_state_along_gate_right);
  if (fabsf(perp_dist) < this->params_.min_dist_to_gate_side &&
      ((comp_state_along_gate_right < 0.0f && comp_state_along_gate_right >= 0.0f - threshold) ||
       (comp_state_along_gate_right > 1.0f && comp_state_along_gate_right <= 1.0f + threshold)))
  {
    cost += this->params_.crash_coeff * fabsf(comp_state_along_gate_right);
    if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 &&
        blockIdx.z == 0)
    {
      printf("Hitting a gate: state_vec: (%f, %f), gate_vec: (%f, %f), projection onto gate: %f\n", state_vec_right.x,
             state_vec_right.y, gate_vec.x, gate_vec.y, comp_state_along_gate_right);
    }
  }
  // if (closest_side_dist < this->params_.min_dist_to_gate_side)
  // {
  //   cost += this->params_.crash_coeff * (this->params_.min_dist_to_gate_side - closest_side_dist);
  //   if (fmaxf(comp_state_along_gate_left, comp_state_along_gate_right) > 1.0f)
  //   {
  //     cost += this->params_.crash_coeff * fmaxf(comp_state_along_gate_left, comp_state_along_gate_right);
  //     if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 &&
  //     blockIdx.z == 0
  //         )
  //     {
  //       printf("Hitting a gate\n");
  //     }
  //   }
  // }
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeHeightCost(float* s)
{
  // Calculate height cost
  float cost = 0;
  // if (s[E_INDEX(OutputIndex, POS_Z)] < 1) {
  //   return this->params_.crash_coeff;
  // }
  float d1 = sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - this->params_.prev_waypoint.x) +
                   SQ(s[E_INDEX(OutputIndex, POS_Y)] - this->params_.prev_waypoint.y));
  float d2 = sqrtf(SQ(s[E_INDEX(OutputIndex, POS_X)] - this->params_.curr_waypoint.x) +
                   SQ(s[E_INDEX(OutputIndex, POS_Y)] - this->params_.curr_waypoint.y));

  float w1 = d1 / (d1 + d2 + 0.001);
  float w2 = d2 / (d1 + d2 + 0.001);
  float interpolated_height = (1.0 - w1) * this->params_.prev_waypoint.z + (1.0 - w2) * this->params_.curr_waypoint.z;

  float height_diff = SQ(fabsf(s[E_INDEX(OutputIndex, POS_Z)] - interpolated_height));
  if (height_diff < 0)
  {
    cost += this->params_.crash_coeff * (1 - height_diff);
  }
  else
  {
    cost += this->params_.height_coeff * height_diff;
  }
  // cost += this->params_.height_coeff * height_diff;
  if (height_diff > this->params_.gate_width)
  {
    cost += 400;
  }
  return cost;
}

template <class CLASS_T, class PARAMS_T>
__device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::computeCostmapCost(float* s)
{
  float cost = 0;
  if (!this->tex_helper_->checkTextureUse(0))
  {
    return cost;
  }
  float3 query_point =
      make_float3(s[E_INDEX(OutputIndex, POS_X)], s[E_INDEX(OutputIndex, POS_Y)], s[E_INDEX(OutputIndex, POS_Z)]);
  float3 tex_coords;
  this->tex_helper_->worldPoseToTexCoord(0, query_point, tex_coords);
  if (tex_coords.x < 0.0f || tex_coords.x > 1.0f || tex_coords.y < 0.0f || tex_coords.y > 1.0f)
  {  // The vehicle is not in the map anymore
    cost += this->params_.crash_coeff;
    if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && blockIdx.x == 0 && blockIdx.y == 0 &&
        blockIdx.z == 0)
    {
      printf("left the map\n");
    }
  }
  float track_cost = this->tex_helper_->queryTexture(0, tex_coords);

  // Calculate cost based on distance from centerline of the track
  if (track_cost > this->params_.track_slop)
  {
    cost += this->params_.track_coeff * track_cost;
  }

  if (track_cost > this->params_.track_boundary_cost)
  {
    // the cost at this point on the costmap indicates no longer being on the track
    cost += this->params_.crash_coeff;
  }

  return cost;
}

template <class CLASS_T, class PARAMS_T>
float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::terminalCost(const Eigen::Ref<const output_array> s)
{
  return 0;
}

template <class CLASS_T, class PARAMS_T>
__device__ float QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::terminalCost(float* s, float* theta_c)
{
  return 0;
}

template <class CLASS_T, class PARAMS_T>
std::vector<float4> QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::loadTrackData(std::string map_path)
{
  // check if file exists
  if (!fileExists(map_path))
  {
    std::cerr << "ERROR: map path invalid, " << map_path << std::endl;
    return std::vector<float4>();
  }

  // load the npz file
  cnpy::npz_t map_dict = cnpy::npz_load(map_path);
  float x_min, x_max, y_min, y_max, ppm;
  float* xBounds = map_dict["xBounds"].data<float>();
  float* yBounds = map_dict["yBounds"].data<float>();
  float* pixelsPerMeter = map_dict["pixelsPerMeter"].data<float>();
  x_min = xBounds[0];
  x_max = xBounds[1];
  y_min = yBounds[0];
  y_max = yBounds[1];
  ppm = pixelsPerMeter[0];

  int width = int((x_max - x_min) * ppm);
  int height = int((y_max - y_min) * ppm);

  if (!changeCostmapSize(width, height))
  {
    std::cerr << "ERROR: load track has invalid sizes" << std::endl;
    return std::vector<float4>();
  }

  float* channel0 = map_dict["channel0"].data<float>();
  float* channel1 = map_dict["channel1"].data<float>();
  float* channel2 = map_dict["channel2"].data<float>();
  float* channel3 = map_dict["channel3"].data<float>();

  // copy the track data into CPU side storage
  for (int i = 0; i < width_ * height_; i++)
  {
    // std::cout << i << " = " << channel0[i] << ", " << channel1[i] << ", " << channel2[i] << ", " << channel3[i] <<
    // std::endl;
    track_costs_[i].x = channel0[i];
    track_costs_[i].y = channel1[i];
    track_costs_[i].z = channel2[i];
    track_costs_[i].w = channel3[i];
  }

  Eigen::Matrix3f R;
  Eigen::Array3f trs;

  // Save the scaling and offset
  R << 1. / (x_max - x_min), 0, 0, 0, 1. / (y_max - y_min), 0, 0, 0, 1;
  trs << -x_min / (x_max - x_min), -y_min / (y_max - y_min), 1;

  updateTransform(R, trs);
  costmapToTexture();

  return track_costs_;
}

template <class CLASS_T, class PARAMS_T>
bool QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::changeCostmapSize(int width, int height)
{
  // TODO set flag at top that indicates memory allocation changes
  if (height < 0 && width < 0)
  {
    std::cerr << "ERROR: cannot resize costmap to size less than 1" << std::endl;
    return false;
  }
  if (height != height_ || width != width_)
  {
    track_costs_.resize(width * height);

    // Allocate memory for the cuda array which is bound the costmap_tex_
    // has been allocated in the past, must be freed
    if (height_ > 0 && width_ > 0)
    {
      HANDLE_ERROR(cudaFreeArray(costmapArray_d_));
    }
    // 4 floats of size 32 bits
    channelDesc_ = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
    HANDLE_ERROR(cudaMallocArray(&costmapArray_d_, &channelDesc_, width, height));

    // set all of the elements in the array to be zero
    std::vector<float4> zero_array(width_ * height_);
    zero_array.resize(width * height, make_float4(0, 0, 0, 0));
    HANDLE_ERROR(cudaMemcpyToArray(costmapArray_d_, 0, 0, zero_array.data(), width * height * sizeof(float4),
                                   cudaMemcpyHostToDevice));
  }

  width_ = width;
  height_ = height;
  return true;
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::updateTransform(const Eigen::Ref<const Eigen::Matrix3f>& m,
                                                              const Eigen::Ref<const Eigen::Array3f>& trs)
{
  this->params_.r_c1.x = m(0, 0);
  this->params_.r_c1.y = m(1, 0);
  this->params_.r_c1.z = m(2, 0);
  this->params_.r_c2.x = m(0, 1);
  this->params_.r_c2.y = m(1, 1);
  this->params_.r_c2.z = m(2, 1);
  this->params_.trs.x = trs(0);
  this->params_.trs.y = trs(1);
  this->params_.trs.z = trs(2);
  // Move the updated parameters to gpu memory
  if (this->GPUMemStatus_)
  {
    paramsToDevice();
  }
}

template <class CLASS_T, class PARAMS_T>
void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::costmapToTexture()
{
  if (width_ < 0 || height_ < 0)
  {
    std::cerr << "ERROR: cannot allocate texture with zero size" << std::endl;
    return;
  }

  // transfer CPU version of costmap to GPU
  float4* costmap_ptr = track_costs_.data();
  HANDLE_ERROR(
      cudaMemcpyToArray(costmapArray_d_, 0, 0, costmap_ptr, width_ * height_ * sizeof(float4), cudaMemcpyHostToDevice));
  cudaStreamSynchronize(this->stream_);

  // Specify texture
  struct cudaResourceDesc resDesc;
  memset(&resDesc, 0, sizeof(resDesc));
  resDesc.resType = cudaResourceTypeArray;
  resDesc.res.array.array = costmapArray_d_;

  // Specify texture object parameters
  struct cudaTextureDesc texDesc;
  memset(&texDesc, 0, sizeof(texDesc));
  texDesc.addressMode[0] = cudaAddressModeClamp;
  texDesc.addressMode[1] = cudaAddressModeClamp;
  texDesc.filterMode = cudaFilterModePoint;
  texDesc.readMode = cudaReadModeElementType;
  texDesc.normalizedCoords = 1;

  // Destroy current texture and create new texture object
  HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_d_));
  HANDLE_ERROR(cudaCreateTextureObject(&costmap_tex_d_, &resDesc, &texDesc, NULL));

  // copy over pointers setup up on CPU code to GPU
  HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->costmapArray_d_, &costmapArray_d_, sizeof(cudaArray*),
                               cudaMemcpyHostToDevice, this->stream_));
  HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->costmap_tex_d_, &costmap_tex_d_, sizeof(cudaTextureObject_t),
                               cudaMemcpyHostToDevice, this->stream_));
  cudaStreamSynchronize(this->stream_);
}

template <class CLASS_T, class PARAMS_T>
__device__ float4 QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::queryTexture(float x, float y) const
{
  return tex2D<float4>(costmap_tex_d_, x, y);
}

template <class CLASS_T, class PARAMS_T>
__device__ float4 QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::queryTextureTransformed(float x, float y)
{
  float u, v, w;
  coorTransform(x, y, &u, &v, &w);
  return queryTexture(u / w, v / w);
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ void QuadrotorMapCostImpl<CLASS_T, PARAMS_T>::coorTransform(float x, float y, float* u, float* v,
                                                                                float* w)
{
  ////Compute a projective transform of (x, y, 0, 1)
  // converts to the texture [0-1] coordinate system
  u[0] = this->params_.r_c1.x * x + this->params_.r_c2.x * y + this->params_.trs.x;
  v[0] = this->params_.r_c1.y * x + this->params_.r_c2.y * y + this->params_.trs.y;
  w[0] = this->params_.r_c1.z * x + this->params_.r_c2.z * y + this->params_.trs.z;
}
