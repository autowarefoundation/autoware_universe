
template <class CLASS_T, class PARAMS_T>
ARRobustCostImpl<CLASS_T, PARAMS_T>::ARRobustCostImpl(cudaStream_t stream)
  : ARStandardCostImpl<CLASS_T, PARAMS_T>(stream)
{
}

template <class CLASS_T, class PARAMS_T>
ARRobustCostImpl<CLASS_T, PARAMS_T>::~ARRobustCostImpl()
{
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float ARRobustCostImpl<CLASS_T, PARAMS_T>::getStabilizingCost(const float* s)
{
  float penalty_val = 0;
  float slip;
  if (fabs(s[4]) < 0.001)
  {
    slip = 0;
  }
  else
  {
    slip = fabs(-atan(s[5] / fabs(s[4])));
  }
  if (slip >= 0.75 * this->params_.max_slip_ang)
  {
    float slip_val = fminf(1.0, slip / this->params_.max_slip_ang);
    float alpha = (slip_val - 0.75) / (1.0 - 0.75);
    penalty_val = alpha * this->params_.crash_coeff;
  }
  // crash if roll is too large
  if (fabs(s[3]) >= M_PI_2)
  {
    penalty_val = this->params_.crash_coeff;
  }
  return this->params_.slip_coeff * slip + penalty_val;
}

template <class CLASS_T, class PARAMS_T>
__host__ __device__ float ARRobustCostImpl<CLASS_T, PARAMS_T>::getCostmapCost(const float* s)
{
  float cost = 0;

  // Compute a transformation to get the (x,y) positions of the front and back of the car.
#ifdef __CUDA_ARCH__
  float x_front = s[0] + this->FRONT_D * __cosf(s[2]);
  float y_front = s[1] + this->FRONT_D * __sinf(s[2]);
  float x_back = s[0] + this->BACK_D * __cosf(s[2]);
  float y_back = s[1] + this->BACK_D * __sinf(s[2]);
#else
  float x_front = s[0] + this->FRONT_D * cosf(s[2]);
  float y_front = s[1] + this->FRONT_D * sinf(s[2]);
  float x_back = s[0] + this->BACK_D * cosf(s[2]);
  float y_back = s[1] + this->BACK_D * sinf(s[2]);
#endif

  float u, v, w;  // Transformed coordinates

  // parameters for the front and back of car
  this->coorTransform(x_front, y_front, &u, &v, &w);
#ifdef __CUDA_ARCH__
  float4 track_params_front = tex2D<float4>(this->costmap_tex_d_, u / w, v / w);
#else
  float2 query = make_float2(u / w * this->width_, v / w * this->height_);
  query.x = query.x - 0.5f;
  query.y = query.y - 0.5f;
  query.x = fmaxf(0.0f, fminf(this->width_ - 1, query.x));
  query.y = fmaxf(0.0f, fminf(this->height_ - 1, query.y));
  float4 track_params_front = this->track_costs_[std::round(query.y) * this->width_ + std::round(query.x)];
#endif

  // Calculate back texture query
  this->coorTransform(x_back, y_back, &u, &v, &w);
#ifdef __CUDA_ARCH__
  float4 track_params_back = tex2D<float4>(this->costmap_tex_d_, u / w, v / w);
#else
  query = make_float2(u / w * this->width_, v / w * this->height_);
  query.x = query.x - 0.5f;
  query.y = query.y - 0.5f;
  query.x = fmaxf(0.0f, fminf(this->width_ - 1, query.x));
  query.y = fmaxf(0.0f, fminf(this->height_ - 1, query.y));
  float4 track_params_back = this->track_costs_[std::round(query.y) * this->width_ + std::round(query.x)];
#endif
  // printf("thread (%d %d %d) front val (%f, %f) %f back_val (%f, %f) %f\n", threadIdx.x, threadIdx.y, threadIdx.z,
  // x_front, y_front, track_params_front.x, x_back, y_back, track_params_back.x);

  // Calculate the constraint penalty
  float constraint_val = fminf(1.0, fmaxf(track_params_front.x, track_params_back.x));

  if (constraint_val >= this->params_.boundary_threshold)
  {
    float alpha = (constraint_val - this->params_.boundary_threshold) / (1.0 - this->params_.boundary_threshold);
    cost += alpha * this->params_.crash_coeff;
  }

  // Calculate the track positioning cost
  if (track_params_front.y > this->params_.track_slop)
  {
    cost += this->params_.track_coeff * track_params_front.y;
  }

  // Calculate the speed cost
  if (this->params_.desired_speed == -1)
  {
    cost += this->params_.speed_coeff * fabs(s[4] - track_params_front.z);
  }
  else
  {
    cost += this->params_.speed_coeff * fabs(s[4] - this->params_.desired_speed);
  }

  // Calculate the heading cost
  cost += this->params_.heading_coeff * fabs(sinf(s[2]) + track_params_front.w);

  return cost;
}

template <class CLASS_T, class PARAMS_T>
inline __host__ __device__ float ARRobustCostImpl<CLASS_T, PARAMS_T>::computeStateCost(const float* s, int timestep,
                                                                                       float* theta_c,
                                                                                       int* crash_status)
{
  float stabilizing_cost = getStabilizingCost(s);
  float costmap_cost = getCostmapCost(s);
  float cost = stabilizing_cost + costmap_cost;
  if (cost > this->MAX_COST_VALUE || isnan(cost))
  {  // TODO Handle max cost value in a generic way
    cost = this->MAX_COST_VALUE;
  }
  return cost;
}

template <class CLASS_T, class PARAMS_T>
inline float ARRobustCostImpl<CLASS_T, PARAMS_T>::computeStateCost(const Eigen::Ref<const output_array> y, int timestep,
                                                                   int* crash_status)
{
  return computeStateCost(y.data(), timestep, nullptr, crash_status);
}
