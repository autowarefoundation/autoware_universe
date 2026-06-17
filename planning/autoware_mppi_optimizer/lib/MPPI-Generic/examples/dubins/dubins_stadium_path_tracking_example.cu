/**
 * @file racer_dubins_stadium_path_tracking_example.cu
 * @brief Example of MPPI-based path tracking and obstacle avoidance for a Racer Dubins model on a stadium track.
 */

 #include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
 #include <mppi/cost_functions/cost.cuh>
 #include <mppi/controllers/MPPI/mppi_controller.cuh>
 #include <mppi/feedback_controllers/feedback.cuh>
 #include <mppi/path/path_projection.hpp>
 #include <mppi/path/path_reference_generator.hpp>
 #include <mppi/path/path2d.hpp>
 #include <mppi/sampling_distributions/gaussian/gaussian.cuh>
 
 #include <opencv2/opencv.hpp>
 
 #include <algorithm>
 #include <cmath>
 #include <string>
 #include <random>
 
 namespace
 {
   // --- Simulation Parameters ---
   constexpr int kMppiHorizon = 80;
   constexpr int kRefHorizon = kMppiHorizon;
   constexpr float kDt = 0.1F;
   constexpr int kNumRollouts = 4*1024;
   constexpr float kTargetSpeed = 2.5F;
   constexpr float kVMax = 3.0F;
   constexpr size_t kSimLaps = 1;
   
   constexpr float kStraightLength = 40.0F;
   constexpr float kTurnRadius = 10.0F;
   constexpr int kSamplesPerArc = 48;
   
   constexpr float kInitArcLength = kStraightLength - 2.0F;
   constexpr float kInitLateralOffset = 0.1F;
   
   constexpr float kLambda = 100.0F;
 
   /**
    * @struct RacerCostParams
    * @brief Parameters for the Racer cost function.
    */
   struct RacerCostParams : public ::CostParams<2>
   {
     float desired_speed = kTargetSpeed;
     float speed_coeff = 20.0F;
     float track_coeff = 500.0F;
     float crash_coeff = 10000.0F;
     float boundary_threshold = 0.8F;
     
     // Coefficient to penalize large steering commands
     float steer_coeff = 50.0F; 
     
     float3 r_c1 = make_float3(1, 0, 0);
     float3 r_c2 = make_float3(0, 1, 0);
     float3 trs = make_float3(0, 0, 1);
   };
 
   /**
    * @class RacerCostImpl
    * @brief Implements cost calculations for the RacerDubins model, including GPU-accelerated costmap lookup.
    */
   template <class CLASS_T, class PARAMS_T = RacerCostParams, class DYN_PARAMS_T = RacerDubinsParams>
   class RacerCostImpl : public ::Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>
   {
   public:
     using PARENT_CLASS = ::Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
     using output_array = typename PARENT_CLASS::output_array;
     using control_array = typename PARENT_CLASS::control_array;
 
     RacerCostImpl(cudaStream_t stream = 0) {
         this->bindToStream(stream);
     }
 
     void freeCudaMem() {
         if (this->GPUMemStatus_) {
             HANDLE_ERROR(cudaFreeArray(costmapArray_d_));
             HANDLE_ERROR(cudaDestroyTextureObject(costmap_tex_d_));
         }
         PARENT_CLASS::freeCudaMem();
     }
 
     void paramsToDevice() {
         HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->params_, &this->params_, sizeof(PARAMS_T), cudaMemcpyHostToDevice, this->stream_));
     }
 
     __host__ __device__ void coorTransform(float x, float y, float* u, float* v, float* w) {
         *u = this->params_.r_c1.x * x + this->params_.r_c2.x * y + this->params_.trs.x;
         *v = this->params_.r_c1.y * x + this->params_.r_c2.y * y + this->params_.trs.y;
         *w = this->params_.r_c1.z * x + this->params_.r_c2.z * y + this->params_.trs.z;
     }
 
     __device__ float4 queryTextureTransformed(float x, float y) {
         float u, v, w;
         coorTransform(x, y, &u, &v, &w);
         return tex2D<float4>(costmap_tex_d_, u / w, v / w);
     }
 
     // GPU-side state cost computation
     __device__ float computeStateCost(float* y, int timestep, float* theta_c, int* crash_status) {
         float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
         float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
         float vel = y[static_cast<int>(RacerDubinsParams::OutputIndex::TOTAL_VELOCITY)];
 
         float4 tex_val = queryTextureTransformed(x, y_pos);
         float track_val = tex_val.x;
 
         float vel_diff = vel - this->params_.desired_speed;
         float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
         float track_cost = this->params_.track_coeff * track_val;
         float crash_cost = 0;
         if (track_val >= this->params_.boundary_threshold) {
             crash_cost = this->params_.crash_coeff;
         }
 
         return speed_cost + track_cost + crash_cost;
     }
 
     cv::Mat cpu_costmap_;
     void setCpuCostmap(const cv::Mat& costmap) { cpu_costmap_ = costmap; }
 
     // CPU-side state cost computation for nominal trajectory evaluation
     float computeStateCost(const Eigen::Ref<const output_array> y, int timestep, int* crash_status) {
         if (cpu_costmap_.empty()) return 0.0f;
 
         float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
         float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
         float vel = y[static_cast<int>(RacerDubinsParams::OutputIndex::TOTAL_VELOCITY)];
 
         float u, v, w;
         coorTransform(x, y_pos, &u, &v, &w);
         
         float norm_u = u / w;
         float norm_v = v / w;
 
         int pixel_u = std::max(0, std::min(static_cast<int>(norm_u * cpu_costmap_.cols), cpu_costmap_.cols - 1));
         int pixel_v = std::max(0, std::min(static_cast<int>(norm_v * cpu_costmap_.rows), cpu_costmap_.rows - 1));
         
         float track_val = cpu_costmap_.at<float>(pixel_v, pixel_u);
 
         float vel_diff = vel - this->params_.desired_speed;
         float speed_cost = this->params_.speed_coeff * (vel_diff * vel_diff);
         float track_cost = this->params_.track_coeff * track_val;
         float crash_cost = 0;
         if (track_val >= this->params_.boundary_threshold) {
             crash_cost = this->params_.crash_coeff;
             *crash_status = 1;
         }
 
         return speed_cost + track_cost + crash_cost;
     }
 
     // GPU Implementation
     __device__ float computeControlCost(float* u, int timestep, float* theta_c, int* crash) {
         // Extract the steering command from the control array
         float steer = u[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)];
         
         // Return a quadratic penalty based on the magnitude of the steering command
         return this->params_.steer_coeff * (steer * steer);
     }
 
     // CPU Implementation (Required by CRTP to match the base class signature)
     float computeControlCost(const Eigen::Ref<const control_array> u, int timestep, int* crash) {
         float steer = u(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
         return this->params_.steer_coeff * (steer * steer);
     }
 
     __device__ float terminalCost(float* y, float* theta_c) {
       float x = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X)];
       float y_pos = y[static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y)];
     
       float track_val = queryTextureTransformed(x, y_pos).x;
     
       return this->params_.track_coeff * track_val * 10.0f; 
     }
 
     void costmapToTexture(int width, int height, float4* host_data) {
         width_ = width;
         height_ = height;
         channelDesc_ = cudaCreateChannelDesc(32, 32, 32, 32, cudaChannelFormatKindFloat);
         HANDLE_ERROR(cudaMallocArray(&costmapArray_d_, &channelDesc_, width, height));
         HANDLE_ERROR(cudaMemcpyToArray(costmapArray_d_, 0, 0, host_data, width * height * sizeof(float4), cudaMemcpyHostToDevice));
 
         struct cudaResourceDesc resDesc;
         memset(&resDesc, 0, sizeof(resDesc));
         resDesc.resType = cudaResourceTypeArray;
         resDesc.res.array.array = costmapArray_d_;
 
         struct cudaTextureDesc texDesc;
         memset(&texDesc, 0, sizeof(texDesc));
         texDesc.addressMode[0] = cudaAddressModeClamp;
         texDesc.addressMode[1] = cudaAddressModeClamp;
         texDesc.filterMode = cudaFilterModeLinear;
         texDesc.readMode = cudaReadModeElementType;
         texDesc.normalizedCoords = 1;
 
         HANDLE_ERROR(cudaCreateTextureObject(&costmap_tex_d_, &resDesc, &texDesc, NULL));
         
         if (this->cost_d_) {
              HANDLE_ERROR(cudaMemcpyAsync(&this->cost_d_->costmap_tex_d_, &costmap_tex_d_, sizeof(cudaTextureObject_t), cudaMemcpyHostToDevice, this->stream_));
         }
     }
 
     void updateCostmapTexture(float4* host_data) {
         if (costmapArray_d_ == nullptr) {
             printf("Error: Costmap array not initialized! Call costmapToTexture first.\n");
             return;
         }
 
         // Copy the new data into the EXISTING GPU array. 
         HANDLE_ERROR(cudaMemcpyToArray(costmapArray_d_, 0, 0, host_data, 
                                        width_ * height_ * sizeof(float4), 
                                        cudaMemcpyHostToDevice));
     }
 
     cudaArray* costmapArray_d_ = nullptr;
     cudaTextureObject_t costmap_tex_d_;
     cudaChannelFormatDesc channelDesc_;
     int width_, height_;
   };
 
   class RacerCost : public RacerCostImpl<RacerCost>
   {
   public:
     RacerCost(cudaStream_t stream = 0) : RacerCostImpl<RacerCost>(stream) {}
   };
 
   /**
    * @class ZeroFeedbackImpl
    * @brief GPU-side implementation for a feedback controller that provides no feedback.
    */
   template <class DYN_T, int NUM_TIMESTEPS = kMppiHorizon>
   class ZeroFeedbackImpl : public GPUFeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T, GPUState> {
   public:
     ZeroFeedbackImpl(cudaStream_t stream = 0) : GPUFeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, DYN_T, GPUState>(stream) {}
     __device__ void k(const float* __restrict__ x_act, const float* __restrict__ x_goal, const int t, float* __restrict__ theta, float* __restrict__ control_output) {}
   };
 
   /**
    * @class ZeroFeedback
    * @brief CPU-side wrapper for the zero feedback controller.
    */
   template <class DYN_T, int NUM_TIMESTEPS = kMppiHorizon>
   class ZeroFeedback : public FeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, int, NUM_TIMESTEPS> {
   public:
       using PARENT_CLASS = FeedbackController<ZeroFeedbackImpl<DYN_T, NUM_TIMESTEPS>, int, NUM_TIMESTEPS>;
       using control_array = typename PARENT_CLASS::control_array;
       using state_array = typename PARENT_CLASS::state_array;
       using TEMPLATED_FEEDBACK_STATE = typename PARENT_CLASS::TEMPLATED_FEEDBACK_STATE;
 
       ZeroFeedback(DYN_T* dyn = nullptr, float dt = 0.01) : PARENT_CLASS(dt, NUM_TIMESTEPS) {}
 
       void initTrackingController() override {}
       control_array k_(const Eigen::Ref<const state_array>& x_act, const Eigen::Ref<const state_array>& x_goal, int t, TEMPLATED_FEEDBACK_STATE& fb_state) override {
           return control_array::Zero();
       }
       void computeFeedback(const Eigen::Ref<const state_array>& init_state, const Eigen::Ref<const typename PARENT_CLASS::state_trajectory>& goal_traj, const Eigen::Ref<const typename PARENT_CLASS::control_trajectory>& control_traj) override {}
   };
 
   // Obstacle structure
   struct Obstacle { float ox; float oy; float r; };
 
   // Helper to update the costmap
   void updateCostmap(cv::Mat& costmap_img, const std::vector<mppi::path::PathReferenceSample>& ref, 
                      const std::vector<Obstacle>& obstacles, int width, int height, float ppm, 
                      float x_min, float y_min, std::vector<float4>& cost_map_gpu_data)
   {
     auto worldToMap = [&](float x, float y) {
         return cv::Point((x - x_min) * ppm, (y - y_min) * ppm);
     };
 
     // 1. Reset costmap and Draw road based on reference path
     costmap_img = cv::Mat::ones(height, width, CV_32FC1);
     cv::Mat center_img = cv::Mat::ones(height, width, CV_8UC1) * 255;
     
     if (ref.size() >= 2) {
         for (size_t i = 0; i < ref.size() - 1; ++i) {
             cv::line(center_img, worldToMap(ref[i].x, ref[i].y), 
                      worldToMap(ref[i+1].x, ref[i+1].y), cv::Scalar(0), 1);
         }
     }
 
     // 2. Compute Distance Transform
     cv::Mat dist_img;
     cv::distanceTransform(center_img, dist_img, cv::DIST_L2, 3);
     dist_img.convertTo(costmap_img, CV_32FC1, 1.0 / 25.0);
     cv::threshold(costmap_img, costmap_img, 0.75, 0.75, cv::THRESH_TRUNC);
     
     // 3. Draw obstacles
     for (const auto& obs : obstacles) {
         cv::circle(costmap_img, worldToMap(obs.ox, obs.oy), obs.r * ppm, cv::Scalar(1.0), -1);
     }
     
     cv::GaussianBlur(costmap_img, costmap_img, cv::Size(5, 5), 1.0);
     
     // 4. Prepare cost data for GPU
     for (int i = 0; i < height; ++i) {
         for (int j = 0; j < width; ++j) {
             cost_map_gpu_data[i * width + j] = make_float4(costmap_img.at<float>(i, j), 0, 0, 0);
         }
     }
   }
 
   // --- MPPI Controller Setup ---
   using DYN = RacerDubins;
   using COST = RacerCost;
   using FB = ZeroFeedback<DYN, kMppiHorizon>;
   using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
   using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;
 
   /**
    * @brief Calculates simulation steps needed for a given number of laps.
    */
   int simStepsForLaps(const mppi::path::Path2D& path, const float laps)
   {
     const float lap_time = path.length() / kVMax;
     return static_cast<int>(std::ceil(laps * lap_time / kDt));
   }
 
   /**
    * @brief Converts world coordinates to pixel coordinates for visualization.
    */
   cv::Point2f worldToPixel(float x, float y, int img_w, int img_h)
   {
     const float scale = 15.0F;
     const float u = static_cast<float>(img_w) / 2.0F + x * scale;
     const float v = static_cast<float>(img_h) / 2.0F - y * scale;
     return cv::Point2f(u, v);
   }
 
   // --- Visualization Utilities ---
   
   void draw_centerline(cv::Mat& img, const mppi::path::Path2D& path)
   {
     cv::Mat overlay = img.clone();
     const auto& anchors = path.anchors();
     for (size_t i = 0; i < anchors.size() - 1; ++i)
     {
       cv::line(overlay, worldToPixel(anchors[i].x, anchors[i].y, img.cols, img.rows),
                worldToPixel(anchors[i + 1].x, anchors[i + 1].y, img.cols, img.rows),
                cv::Scalar(128, 128, 128), 2);
     }
     cv::addWeighted(overlay, 0.5, img, 0.5, 0, img);
   }
 
   void draw_reference_path(cv::Mat& img, const std::vector<mppi::path::PathReferenceSample>& ref)
   {
     if (ref.size() < 2) return;
     cv::Mat overlay = img.clone();
     for (size_t i = 0; i < ref.size() - 1; ++i)
     {
       cv::line(overlay, worldToPixel(ref[i].x, ref[i].y, img.cols, img.rows),
                worldToPixel(ref[i + 1].x, ref[i + 1].y, img.cols, img.rows),
                cv::Scalar(255, 0, 0), 2);
     }
     cv::addWeighted(overlay, 0.5, img, 0.5, 0, img);
   }
 
   void draw_trajectory(cv::Mat& img, const Mppi::state_trajectory& traj)
   {
     cv::Mat overlay = img.clone();
     const int x_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_X);
     const int y_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_Y);
     for (int i = 0; i < traj.cols() - 1; ++i)
     {
       cv::line(overlay, worldToPixel(traj(x_idx, i), traj(y_idx, i), img.cols, img.rows),
                worldToPixel(traj(x_idx, i + 1), traj(y_idx, i + 1), img.cols, img.rows),
                cv::Scalar(0, 255, 0), 2);
     }
     cv::addWeighted(overlay, 0.75, img, 0.25, 0, img);
   }
 
   void draw_sampled_trajectories(cv::Mat& img, const std::vector<Mppi::output_trajectory>& sampled_trajectories)
   {
     if (sampled_trajectories.empty()) return;
     cv::Mat overlay = img.clone();
     const int x_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_X);
     const int y_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_Y);
     for (const auto& traj : sampled_trajectories)
     {
       for (int i = 0; i + 1 < traj.cols(); ++i)
       {
         cv::line(overlay, worldToPixel(traj(x_idx, i), traj(y_idx, i), img.cols, img.rows),
                  worldToPixel(traj(x_idx, i + 1), traj(y_idx, i + 1), img.cols, img.rows),
                  cv::Scalar(180, 180, 180), 1);
       }
     }
     cv::addWeighted(overlay, 0.4, img, 0.6, 0, img);
   }
 }  // namespace
 
 /**
  * @brief Main simulation loop for the Racer Dubins path tracking example.
  *
  * @param argc Argument count.
  * @param argv Argument vector (optional: seed for RNG).
  * @return int Exit code.
  */
 int main(int argc, char** argv)
 {
     // 1. Initialize random seed for reproducibility
     unsigned int seed = 42;
     if (argc > 1) {
         seed = std::stoul(argv[1]);
     }
     std::cout << "Using random seed: " << seed << std::endl;
     std::string video_path = "racer_dubins_stadium_path_tracking.mp4";
 
     /* Environment */
     // 2. Generate track path (stadium shape)
     const mppi::path::Path2D path = mppi::path::Path2D::stadium(kStraightLength, kTurnRadius, kSamplesPerArc);
     
     float x_min = -40, x_max = 60, y_min = -30, y_max = 30;
     float ppm = 10.0f;
     int width = (x_max - x_min) * ppm;
     int height = (y_max - y_min) * ppm;
     
     cv::Mat costmap_img = cv::Mat::ones(height, width, CV_32FC1);
     
     auto worldToMap = [&](float x, float y) {
         return cv::Point((x - x_min) * ppm, (y - y_min) * ppm);
     };
     
 
     // TODO: calculate road costs for the reference at each iteration, update with the updateCostmapTexture
     // Draw road (cost 0.0)
     // 1. Draw a 1-pixel thin centerline on a blank binary image (White background, Black line)
     cv::Mat center_img = cv::Mat::ones(height, width, CV_8UC1) * 255;
     const auto& anchors = path.anchors();
     for (size_t i = 0; i < anchors.size() - 1; ++i) {
         cv::line(center_img, worldToMap(anchors[i].x, anchors[i].y), 
                  worldToMap(anchors[i+1].x, anchors[i+1].y), cv::Scalar(0), 1);
     }
 
     // 2. Compute Distance Transform (Distance from every pixel to the black centerline)
     cv::Mat dist_img;
     cv::distanceTransform(center_img, dist_img, cv::DIST_L2, 3);
 
     // 3. Convert distance to a continuous cost gradient
     // A multiplier of (1.0 / 25.0) means the cost becomes 1.0 when the car is 25 pixels (2.5m) off-center.
     dist_img.convertTo(costmap_img, CV_32FC1, 1.0 / 25.0);
 
     // 4. Cap the off-track cost at 0.75. 
     // (This ensures it is strictly lower than your 0.8F crash threshold, so going off track 
     // is heavily penalized, but hitting a 1.0 obstacle is correctly recognized as worse).
     cv::threshold(costmap_img, costmap_img, 0.75, 0.75, cv::THRESH_TRUNC);
     
     // 3. Generate obstacles once
     std::vector<Obstacle> obstacles;
     std::mt19937 gen(seed);
     std::uniform_real_distribution<float> dist_s(0, path.length());
     std::uniform_real_distribution<float> dist_side(-1.0, 1.0);
     std::uniform_real_distribution<float> dist_r(2.0, 4.5);
     
     for (int i = 0; i < 15; ++i) {
         float s = dist_s(gen);
         float side = (dist_side(gen) > 0 ? 1.0 : -1.0) * 2.5; 
         float r = dist_r(gen);
         auto p = path.poseAt(s);
         float tx, ty;
         path.tangentAt(s, tx, ty);
         obstacles.push_back({p.x - side * ty, p.y + side * tx, r});
     }
 
     COST cost;
     cost.GPUSetup();
     std::vector<float4> cost_map_gpu_data(width * height, make_float4(0,0,0,0));
     cost.costmapToTexture(width, height, cost_map_gpu_data.data());
     
     RacerCostParams cost_params;
     cost_params.r_c1 = make_float3(1.0f / (x_max - x_min), 0, 0);
     cost_params.r_c2 = make_float3(0, 1.0f / (y_max - y_min), 0);
     cost_params.trs = make_float3(-x_min / (x_max - x_min), -y_min / (y_max - y_min), 1);
     cost.setParams(cost_params);
 
     const size_t num_sim_steps = simStepsForLaps(path, kSimLaps);
 
     mppi::path::PathReferenceGenerator ref_gen(kDt);
     ref_gen.setSpeedCap(kVMax);
 
     /* Model parameters */
     // 5. Setup model and sampling distributions
     DYN model;
     RacerDubinsParams dyn;
     dyn.wheel_base = 0.3f;
     model.setParams(dyn);
     std::array<float2, DYN::CONTROL_DIM> u_rng{};
     u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = { -1.0f, 1.0f };
     u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = { -1.0f, 1.0f };
     model.setControlRanges(u_rng);
 
     SAMPLER::SAMPLING_PARAMS_T sp{};
     sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = 0.2F;
     sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = 0.3F;
     sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
     SAMPLER sampler(sp);
 
     // 6. Setup MPPI Controller
     FB feedback(&model, kDt);
     Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
     Mppi controller(&model, &cost, &feedback, &sampler, kDt, 1, kLambda, 0.0F, kMppiHorizon, u_nom);
     {
       auto cp = controller.getParams();
       cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
       cp.cost_rollout_dim_ = dim3(32, 2, 1);
       cp.seed_ = 1U;
       controller.setParams(cp);
       controller.setPercentageSampledControlTrajectories(0.01F);
     }
     model.GPUSetup();
 
     // 7. Initialize simulation state
     DYN::state_array x = model.getZeroState();
     const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
     x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)) = p0.x;
     x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)) = p0.y;
     x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)) = p0.yaw;
     x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)) = kTargetSpeed;
 
     float arcLength = kInitArcLength;
 
     // 8. Prepare visualization
     cv::Mat base_frame = cv::Mat::zeros(1024, 1024, CV_8UC3);
     draw_centerline(base_frame, path);
     // Draw obstacles on base frame
     for (const auto& obs : obstacles) {
         cv::circle(base_frame, worldToPixel(obs.ox, obs.oy, 1024, 1024), obs.r * 15.0f, cv::Scalar(0, 0, 255), -1);
     }
 
     cv::VideoWriter video(video_path, 
                       cv::VideoWriter::fourcc('m','p','4','v'), 
                       static_cast<int>(1.0F/kDt), base_frame.size());
     cv::VideoWriter costmap_video("racer_dubins_stadium_costmap.mp4", 
                                cv::VideoWriter::fourcc('m','p','4','v'), 
                                static_cast<int>(1.0F/kDt), costmap_img.size());
 
     cv::namedWindow("MPPI Tracking", cv::WINDOW_NORMAL);
     cv::resizeWindow("MPPI Tracking", base_frame.cols, base_frame.rows);
 
     // 9. Main simulation loop
     for (size_t k = 0; k < num_sim_steps; ++k) {
       const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(path, arcLength, kRefHorizon);
       
       // Update the costmap
       updateCostmap(costmap_img, ref, obstacles, width, height, ppm, x_min, y_min, cost_map_gpu_data);
       
       // Save costmap frame
       cv::Mat costmap_vis;
       costmap_img.convertTo(costmap_vis, CV_8UC1, 255.0);
       cv::cvtColor(costmap_vis, costmap_vis, cv::COLOR_GRAY2BGR);
       costmap_video.write(costmap_vis);
 
       cost.updateCostmapTexture(cost_map_gpu_data.data());
       cost.setCpuCostmap(costmap_img);
       
       // Update importance sampling based on current nominal control
       controller.updateImportanceSampler(u_nom);
 
       // Compute control sequence
       controller.computeControl(x, 1);
       cudaStreamSynchronize(controller.stream_);
       controller.calculateSampledStateTrajectories();
       
       Mppi::control_trajectory u_opt = controller.getControlSeq();
 
       /* Video frame generation */
       const auto state_trajectory = controller.getActualStateSeq();
       const auto sampled_trajectories = controller.getSampledOutputTrajectories();
       
       auto frame = base_frame.clone();
       
       draw_reference_path(frame, ref);
       draw_sampled_trajectories(frame, sampled_trajectories);
       draw_trajectory(frame, state_trajectory);
       video.write(frame);
 
       cv::imshow("MPPI Tracking", frame);
       if (cv::waitKey(1) == 27) break; // Exit on ESC
 
       // Step simulation model
       DYN::state_array x_next = model.getZeroState();
       DYN::state_array xdot = model.getZeroState();
       DYN::output_array y = DYN::output_array::Zero();
 
       model.enforceConstraints(x, u_opt.col(0));
       model.step(x, x_next, xdot, u_opt.col(0), y, static_cast<float>(k), kDt);
 
       // Shift nominal control trajectory for the next step
       u_nom.leftCols(kMppiHorizon - 1) = u_opt.rightCols(kMppiHorizon - 1);
       u_nom.rightCols(1) = u_opt.rightCols(1); 
 
       x = x_next;
 
       // Project state onto path to update progress
       const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(path, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)), x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)), arcLength);
       arcLength = proj.arc_length_s;
     }
 
     cost.freeCudaMem();
     return 0;
 }