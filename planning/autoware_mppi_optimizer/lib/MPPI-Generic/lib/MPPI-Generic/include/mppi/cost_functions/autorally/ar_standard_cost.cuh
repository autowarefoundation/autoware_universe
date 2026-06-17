#pragma once

#ifndef AR_STANDARD_COST_CUH_
#define AR_STANDARD_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/autorally/ar_nn_model.cuh>
#include <mppi/utils/file_utils.h>
#include <vector>
#include <Eigen/Dense>
#include <cuda_runtime.h>
#include <cnpy.h>

struct ARStandardCostParams : public CostParams<2>
{
  float desired_speed = 6.0;
  float speed_coeff = 4.25;
  float track_coeff = 200.0;
  float max_slip_ang = 1.25;
  float slip_coeff = 10.0;
  float track_slop = 0;
  float crash_coeff = 10000;
  float boundary_threshold = 0.65;
  // TODO remove from struct
  int grid_res = 10;
  /*
   * Prospective transform matrix
   * r_c1.x, r_c2.x, trs.x
   * r_c1.y, r_c2.y, trs.y
   * r_c1.z, r_c2.z, trs.z
   */
  float3 r_c1;  // R matrix col 1
  float3 r_c2;  // R matrix col 2
  float3 trs;   // translation vector

  ARStandardCostParams()
  {
    control_cost_coeff[0] = 0.0;  // steering_coeff
    control_cost_coeff[1] = 0.0;  // throttle_coeff
  }
};

template <class CLASS_T, class PARAMS_T = ARStandardCostParams, class DYN_PARAMS_T = NNDynamicsParams>
class ARStandardCostImpl : public Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>
{
public:
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PARENT_CLASS = Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>;
  using output_array = typename PARENT_CLASS::output_array;
  static constexpr float MAX_COST_VALUE = 1e16;

  /**
   * Constructor
   * @param width
   * @param height
   */
  ARStandardCostImpl(cudaStream_t stream = 0);

  std::string getCostFunctionName() const override
  {
    return "AutoRally standard cost function";
  }

  /**
   * Deallocates the allocated cuda memory for an object
   */
  void freeCudaMem();

  inline __host__ __device__ int getHeight() const
  {
    return height_;
  }
  inline __host__ __device__ int getWidth() const
  {
    return width_;
  }
  inline std::vector<float4> getTrackCostCPU() const
  {
    return track_costs_;
  }
  inline Eigen::Matrix3f getRotation();
  inline Eigen::Array3f getTranslation();
  // inline __host__ __device__ cudaArray* getCudaArray() {return costmapArray_d_;}
  // inline __host__ __device__ cudaArray_t* getCudaArrayPointer() {return &costmapArray_d_;}
  // inline __host__ __device__ cudaTextureObject_t* getCostmapTex(){return &costmap_tex_d_;};

  /**
   * Copies the parameters to the GPU object
   */
  void paramsToDevice();

  /**
   * alters the costmap size in CPU storage and GPU texture
   * @param width
   * @param height
   * @return
   */
  bool changeCostmapSize(int width, int height);

  /**
   * @brief Initializes the costmap to all zeros.
   *
   * Initializes a float4 vector to the correct width and height and sets every value to zero on the CPU.
   * default leaves the sizes alone
   */
  void clearCostmapCPU(int width = -1, int height = -1);

  /**
   * @brief Binds the member variable costmap to a CUDA texture.
   */
  void costmapToTexture();

  __device__ float4 queryTexture(float x, float y) const;

  /**
   * @brief Loads track data from a file.
   * @param C-string representing the path to the costmap data file.
   * @param h Matrix representing a transform from world to (offset) costmap coordinates.
   * @param trs Array representing the offset.
   */
  std::vector<float4> loadTrackData(std::string map_path);

  /**
   * @brief Updates the current costmap coordinate transform.
   * @param h Matrix representing a transform from world to (offset) costmap coordinates.
   * @param trs Array representing the offset.
   */
  void updateTransform(Eigen::MatrixXf m, Eigen::ArrayXf trs);

  /**
   * @brief Compute a coordinate transform going from world to costmap coordinates.
   */
  __host__ __device__ void coorTransform(float x, float y, float* u, float* v, float* w);

  /**
   * Queries the texture using coorTransform beforehand
   */
  __host__ __device__ float4 queryTextureTransformed(float x, float y);

  /**
   *@brief Initializes the debug window for a default 20x20 meter window.
   */
  // void debugDisplayInit();

  /**
   * @brief Initialize and allocate memory for debug window display
   */
  // void debugDisplayInit(int width_m, int height_m, int ppm);

  bool getDebugDisplayEnabled()
  {
    return false;
  }

  /**
   * @brief Display the debug view centered around x and y.
   * @param x float representing the current x-coordinate
   * @param y float representing the current y-coordinate
   */
  // cv::Mat getDebugDisplay(float x, float y, float heading);

  /**
   *
   * @param description
   * @param data
   */
  // void updateCostmap(std::vector<int> description, std::vector<float> data);

  /**
   *
   * @param description
   * @param data
   */
  // void updateObstacles(std::vector<int> description, std::vector<float> data);

  /**
   * @brief Returns whether or not the vehicle has crashed or not
   */
  //__host__ __device__ void getCrash(float* state, int* crash);

  /**
   * @brief Compute the cost for achieving a desired speed
   */
  __host__ __device__ float getSpeedCost(float* s, int* crash);

  /**
   * @brief Compute a penalty term for crashing
   */
  __host__ __device__ float getCrashCost(float* s, int* crash, int num_timestep);

  /**
   * @brief Compute some cost terms that help stabilize the car.
   */
  __host__ __device__ float getStabilizingCost(float* s, int* crash);

  /**
   * @brief Compute the current track cost based on the costmap.
   * Requires using CUDA texture memory so can only be run on the GPU
   */
  __host__ __device__ float getTrackCost(float* s, int* crash);

  /**
   * @brief Compute all of the individual cost terms and adds them together.
   */
  inline __device__ float computeStateCost(float* s, int timestep, float* theta_c, int* crash_status);

  /**
   * @brief Computes the terminal cost from a state
   */
  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> y);

  // Constant variables
  const float FRONT_D = 0.5;  ///< Distance from GPS receiver to front of car.
  const float BACK_D = -0.5;  ///< Distance from GPS receiver to back of car.

protected:
  bool l1_cost_ = false;  // Whether to use L1 speed cost (if false it is L2)

  // Primary variables
  int width_ = -1;                     ///< width of costmap
  int height_ = -1;                    ///< height of costmap.
  cudaArray* costmapArray_d_;          ///< Cuda array for texture binding.
  cudaChannelFormatDesc channelDesc_;  ///< Cuda texture channel description.
  cudaTextureObject_t costmap_tex_d_;  ///< Cuda texture object.
  // TODO what does this look like on GPU side
  std::vector<float4> track_costs_;

  // Debugging variables
  float* debug_data_;  ///< Host array for holding debug info.
  // float* debug_data_d_; ///< Device array for holding debug info.
  int debug_img_width_;   /// Width (in meters) of area imaged by debug view.
  int debug_img_height_;  ///< Height (in meters) of area imaged by debug view.
  int debug_img_ppm_;     ///< Pixels per meter for resolution of debug view.
  int debug_img_size_;    ///< Number of pixels in the debug image.
  bool debugging_;        ///< Indicator for if we're in debugging mode
};

#if __CUDACC__
#include "ar_standard_cost.cu"
#endif

class ARStandardCost : public ARStandardCostImpl<ARStandardCost>
{
public:
  ARStandardCost(cudaStream_t stream = 0) : ARStandardCostImpl<ARStandardCost>(stream){};
};

#endif  // AR_STANDARD_COST_CUH_
