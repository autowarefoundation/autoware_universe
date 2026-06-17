#include <array>

#include <gtest/gtest.h>
#include <mppi/utils/test_helper.h>
#include <mppi/utils/math_utils.h>
#include <mppi/utils/gpu_err_chk.cuh>
#include <mppi/utils/cuda_math_utils.cuh>

TEST(MATH_UTILS, SQ)
{
  EXPECT_FLOAT_EQ(0.25, SQ(0.5));
  EXPECT_FLOAT_EQ(0.25, SQ(0.5 - 1.0));
  EXPECT_FLOAT_EQ(25, SQ(5));
  EXPECT_FLOAT_EQ(25, SQ(5 - 10));
  EXPECT_FLOAT_EQ(625, SQ(5 * 5));
}

TEST(MATH_UTILS, QuatInv)
{
  // Create an unnormalized quaternion.
  std::array<float, 4> q{ 1, 2, 3, 2 };
  std::array<float, 4> q_inv{};

  mppi::math::QuatInv(q.data(), q_inv.data());

  // Compute the inverse using eigen.
  // Don't use the array constructor because eigen stores it as [x, y, z, w] internally.
  const Eigen::Quaternionf eigen_q{ q[0], q[1], q[2], q[3] };
  const Eigen::Quaternionf eigen_q_inv = eigen_q.inverse().normalized();
  std::array<float, 4> correct_q_inv{ eigen_q_inv.w(), eigen_q_inv.x(), eigen_q_inv.y(), eigen_q_inv.z() };

  // q_inv should be normalized.
  constexpr float tol = 1e-7;
  array_assert_float_near<4>(q_inv, correct_q_inv, tol);
}

TEST(MATH_UTILS, RotatePointByDCM)
{
  const float tol = 1e-6;
  Eigen::Quaternionf q_eig = Eigen::Quaternionf::UnitRandom();
  Eigen::Vector3f point_eig = Eigen::Vector3f::Random();

  std::array<float, 4> q{ q_eig.w(), q_eig.x(), q_eig.y(), q_eig.z() };
  float3 point = make_float3(point_eig.x(), point_eig.y(), point_eig.z());

  float M[3][3];
  Eigen::Matrix3f M_eig;
  mppi::math::Quat2DCM(q.data(), M);
  mppi::math::Quat2DCM(q_eig, M_eig);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      ASSERT_NEAR(M[row][col], M_eig(row, col), tol) << " failed at row: " << row << ", col: " << col;
    }
  }

  float3 result, result_eig_rotation_matrix;
  Eigen::Vector3f result_eig;
  mppi::math::RotatePointByDCM(M, point, result);
  mppi::math::RotatePointByDCM(M_eig, point, result_eig_rotation_matrix);
  mppi::math::RotatePointByDCM(M_eig, point_eig, result_eig);
  ASSERT_NEAR(result.x, result_eig_rotation_matrix.x, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.y, result_eig_rotation_matrix.y, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.z, result_eig_rotation_matrix.z, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.x, result_eig.x(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
  ASSERT_NEAR(result.y, result_eig.y(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
  ASSERT_NEAR(result.z, result_eig.z(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
}

TEST(MATH_UTILS, RotatePointByDCMTranspose)
{
  const float tol = 1e-6;
  Eigen::Quaternionf q_eig = Eigen::Quaternionf::UnitRandom();
  Eigen::Vector3f point_eig = Eigen::Vector3f::Random();

  std::array<float, 4> q{ q_eig.w(), q_eig.x(), q_eig.y(), q_eig.z() };
  float3 point = make_float3(point_eig.x(), point_eig.y(), point_eig.z());

  float M[3][3];
  Eigen::Matrix3f M_eig;
  mppi::math::Quat2DCM(q.data(), M);
  mppi::math::Quat2DCM(q_eig, M_eig);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      ASSERT_NEAR(M[row][col], M_eig(row, col), tol) << " failed at row: " << row << ", col: " << col;
    }
  }

  float3 result, result_eig_rotation_matrix;
  Eigen::Vector3f result_eig;
  mppi::math::RotatePointByDCM(M, point, result, mppi::matrix_multiplication::MAT_OP::TRANSPOSE);
  mppi::math::RotatePointByDCM(M_eig, point, result_eig_rotation_matrix,
                               mppi::matrix_multiplication::MAT_OP::TRANSPOSE);
  mppi::math::RotatePointByDCM(M_eig, point_eig, result_eig, mppi::matrix_multiplication::MAT_OP::TRANSPOSE);
  ASSERT_NEAR(result.x, result_eig_rotation_matrix.x, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.y, result_eig_rotation_matrix.y, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.z, result_eig_rotation_matrix.z, tol) << " failed comparing R stored in 2d float array and R in "
                                                              "Eigen matrix";
  ASSERT_NEAR(result.x, result_eig.x(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
  ASSERT_NEAR(result.y, result_eig.y(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
  ASSERT_NEAR(result.z, result_eig.z(), tol) << " failed comparing GPU and Eigen methods of rotating by DCM";
}

TEST(MATH_UTILS, QuatMultiply)
{
  // Create an unnormalized quaternion.
  std::array<float, 4> q1{ 1, 2, 3, 4 };
  std::array<float, 4> q2{ 8, 7, 6, 5 };
  std::array<float, 4> q3{};
  std::array<float, 4> q3_norm{};
  Eigen::Quaternionf eigen_result, eigen_result_norm;

  // Compare the multiplication using eigen.
  // Don't use the array constructor because eigen stores it as [x, y, z, w] internally.
  const Eigen::Quaternionf eigen_q1{ q1[0], q1[1], q1[2], q1[3] };
  const Eigen::Quaternionf eigen_q2{ q2[0], q2[1], q2[2], q2[3] };

  // Ground Truth
  const Eigen::Quaternionf eigen_q3 = (eigen_q1 * eigen_q2);
  const Eigen::Quaternionf eigen_q3_norm = eigen_q3.normalized();

  // Our Methods
  mppi::math::QuatMultiply(q1.data(), q2.data(), q3.data(), false);
  mppi::math::QuatMultiply(q1.data(), q2.data(), q3_norm.data());
  mppi::math::QuatMultiply(eigen_q1, eigen_q2, eigen_result, false);
  mppi::math::QuatMultiply(eigen_q1, eigen_q2, eigen_result_norm);

  std::array<float, 4> correct_q3{ eigen_q3.w(), eigen_q3.x(), eigen_q3.y(), eigen_q3.z() };
  std::array<float, 4> eigen_q3_array{ eigen_result.w(), eigen_result.x(), eigen_result.y(), eigen_result.z() };
  std::array<float, 4> correct_q3_norm{ eigen_q3_norm.w(), eigen_q3_norm.x(), eigen_q3_norm.y(), eigen_q3_norm.z() };
  std::array<float, 4> eigen_q3_norm_array{ eigen_result_norm.w(), eigen_result_norm.x(), eigen_result_norm.y(),
                                            eigen_result_norm.z() };

  // q_inv should be normalized.
  constexpr auto tol = 1e-7;
  array_assert_float_near<4>(q3, correct_q3, tol);
  array_assert_float_near<4>(eigen_q3_array, correct_q3, tol);
  array_assert_float_near<4>(q3_norm, correct_q3_norm, tol);
  array_assert_float_near<4>(eigen_q3_norm_array, correct_q3_norm, tol);
}

TEST(MATH_UTILS, RotatePointByQuat)
{
  for (int i = 0; i < 100; i++)
  {
    Eigen::Quaternionf rotation = Eigen::Quaternionf::UnitRandom();
    Eigen::Vector3f translation = Eigen::Vector3f::Random();

    float3 point = make_float3(translation.x(), translation.y(), translation.z());
    float q[4] = { rotation.w(), rotation.x(), rotation.y(), rotation.z() };

    float3 output, output_eig;
    Eigen::Vector3f output_all_eig;

    // Test method with 3 parameters, output going into the last
    mppi::math::RotatePointByQuat(rotation, point, output_eig);
    mppi::math::RotatePointByQuat(q, point, output);
    mppi::math::RotatePointByQuat(rotation, translation, output_all_eig);

    // Test method with 2 parameters, modifying the second
    float3 output2 = point;
    float3 output2_eig = point;
    Eigen::Vector3f output2_all_eig = translation;
    mppi::math::RotatePointByQuat(rotation, output2_eig);
    mppi::math::RotatePointByQuat(q, output2);
    mppi::math::RotatePointByQuat(rotation, output2_all_eig);

    // Ground Truth
    auto result = rotation * translation;

    EXPECT_NEAR(output.x, result.x(), 1.0e-5);
    EXPECT_NEAR(output.y, result.y(), 1.0e-5);
    EXPECT_NEAR(output.z, result.z(), 1.0e-5);
    EXPECT_NEAR(output_eig.x, result.x(), 1.0e-5);
    EXPECT_NEAR(output_eig.y, result.y(), 1.0e-5);
    EXPECT_NEAR(output_eig.z, result.z(), 1.0e-5);
    EXPECT_NEAR(output_all_eig.x(), result.x(), 1.0e-5);
    EXPECT_NEAR(output_all_eig.y(), result.y(), 1.0e-5);
    EXPECT_NEAR(output_all_eig.z(), result.z(), 1.0e-5);
    EXPECT_NEAR(output2.x, result.x(), 1.0e-5);
    EXPECT_NEAR(output2.y, result.y(), 1.0e-5);
    EXPECT_NEAR(output2.z, result.z(), 1.0e-5);
    EXPECT_NEAR(output2_eig.x, result.x(), 1.0e-5);
    EXPECT_NEAR(output2_eig.y, result.y(), 1.0e-5);
    EXPECT_NEAR(output2_eig.z, result.z(), 1.0e-5);
    EXPECT_NEAR(output2_all_eig.x(), result.x(), 1.0e-5);
    EXPECT_NEAR(output2_all_eig.y(), result.y(), 1.0e-5);
    EXPECT_NEAR(output2_all_eig.z(), result.z(), 1.0e-5);
  }
}

TEST(MATH_UTILS, QuatDCM)
{
  for (int iteration = 0; iteration < 100; iteration++)
  {
    Eigen::Quaternionf q_eig = Eigen::Quaternionf::UnitRandom();
    float q[4] = { q_eig.w(), q_eig.x(), q_eig.y(), q_eig.z() };
    float M[3][3];
    Eigen::Matrix3f M_eig;
    M_eig = q_eig.toRotationMatrix();
    mppi::math::Quat2DCM(q, M);
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        EXPECT_NEAR(M_eig(i, j), M[i][j], 1.0e-5);
      }
    }
  }
}

TEST(MATH_UTILS, EulerDCM)
{
  for (int iteration = 0; iteration < 100; iteration++)
  {
    Eigen::Quaternionf q_eig = Eigen::Quaternionf::UnitRandom();
    float r, p, y;
    float M[3][3];
    Eigen::Matrix3f M_eig, M_result;
    M_result = q_eig.toRotationMatrix();

    mppi::math::Quat2EulerNWU(q_eig, r, p, y);
    mppi::math::Euler2DCM_NWU(r, p, y, M);
    mppi::math::Euler2DCM_NWU(r, p, y, M_eig);
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        EXPECT_NEAR(M_result(i, j), M[i][j], 1.0e-5);
        EXPECT_NEAR(M_result(i, j), M_eig(i, j), 1.0e-5);
      }
    }
  }
}

TEST(MATH_UTILS, TranslateVectorByQuat)
{
  const float tol = 1e-6;
  Eigen::Vector3f origin_eig = Eigen::Vector3f::Random();
  Eigen::Vector3f body_pose_eig = Eigen::Vector3f::Random();
  Eigen::Quaternionf rotation_eig = Eigen::Quaternionf::UnitRandom();

  float3 origin_f3 = make_float3(origin_eig.x(), origin_eig.y(), origin_eig.z());
  float3 body_pose_f3 = make_float3(body_pose_eig.x(), body_pose_eig.y(), body_pose_eig.z());
  float rotation_array[4] = { rotation_eig.w(), rotation_eig.x(), rotation_eig.y(), rotation_eig.z() };
  // Create output variables
  float3 output_eig, output_array;
  Eigen::Vector3f output_all_eig;

  // Ground Truth
  Eigen::Matrix3f R_eig = rotation_eig.toRotationMatrix();
  Eigen::Vector3f correct_value = origin_eig + R_eig * body_pose_eig;

  mppi::math::bodyOffsetToWorldPoseQuat(body_pose_f3, origin_f3, rotation_eig, output_eig);
  mppi::math::bodyOffsetToWorldPoseQuat(body_pose_f3, origin_f3, rotation_array, output_array);
  mppi::math::bodyOffsetToWorldPoseQuat(body_pose_eig, origin_eig, rotation_eig, output_all_eig);

  EXPECT_NEAR(output_eig.x, correct_value.x(), tol);
  EXPECT_NEAR(output_eig.y, correct_value.y(), tol);
  EXPECT_NEAR(output_eig.z, correct_value.z(), tol);
  EXPECT_NEAR(output_array.x, correct_value.x(), tol);
  EXPECT_NEAR(output_array.y, correct_value.y(), tol);
  EXPECT_NEAR(output_array.z, correct_value.z(), tol);
  EXPECT_NEAR(output_all_eig.x(), correct_value.x(), tol);
  EXPECT_NEAR(output_all_eig.y(), correct_value.y(), tol);
  EXPECT_NEAR(output_all_eig.z(), correct_value.z(), tol);
}

TEST(MATH_UTILS, TranslateVectorByDCM)
{
  const float tol = 1e-6;
  Eigen::Vector3f origin_eig = Eigen::Vector3f::Random();
  Eigen::Vector3f body_pose_eig = Eigen::Vector3f::Random();
  Eigen::Quaternionf rotation_eig = Eigen::Quaternionf::UnitRandom();

  float3 origin_f3 = make_float3(origin_eig.x(), origin_eig.y(), origin_eig.z());
  float3 body_pose_f3 = make_float3(body_pose_eig.x(), body_pose_eig.y(), body_pose_eig.z());
  Eigen::Matrix3f R_eig = rotation_eig.toRotationMatrix();
  float R_float[3][3];
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      R_float[row][col] = R_eig(row, col);
    }
  }
  // Create output variables
  float3 output_eig, output_array;
  Eigen::Vector3f output_all_eig;

  // Ground Truth
  Eigen::Vector3f correct_value = origin_eig + R_eig * body_pose_eig;

  // Our Methods
  mppi::math::bodyOffsetToWorldPoseDCM(body_pose_f3, origin_f3, R_eig, output_eig);
  mppi::math::bodyOffsetToWorldPoseDCM(body_pose_f3, origin_f3, R_float, output_array);
  mppi::math::bodyOffsetToWorldPoseDCM(body_pose_eig, origin_eig, R_eig, output_all_eig);

  EXPECT_NEAR(output_eig.x, correct_value.x(), tol);
  EXPECT_NEAR(output_eig.y, correct_value.y(), tol);
  EXPECT_NEAR(output_eig.z, correct_value.z(), tol);
  EXPECT_NEAR(output_array.x, correct_value.x(), tol);
  EXPECT_NEAR(output_array.y, correct_value.y(), tol);
  EXPECT_NEAR(output_array.z, correct_value.z(), tol);
  EXPECT_NEAR(output_all_eig.x(), correct_value.x(), tol);
  EXPECT_NEAR(output_all_eig.y(), correct_value.y(), tol);
  EXPECT_NEAR(output_all_eig.z(), correct_value.z(), tol);
}

TEST(MATH_UTILS, TranslateVectorByEuler)
{
  const float tol = 1e-6;
  Eigen::Vector3f origin_eig = Eigen::Vector3f::Random();
  Eigen::Vector3f body_pose_eig = Eigen::Vector3f::Random();
  Eigen::Quaternionf q = Eigen::Quaternionf::UnitRandom();
  Eigen::Matrix3f R_eig = q.toRotationMatrix();

  float3 origin_f3 = make_float3(origin_eig.x(), origin_eig.y(), origin_eig.z());
  float3 body_pose_f3 = make_float3(body_pose_eig.x(), body_pose_eig.y(), body_pose_eig.z());

  float3 rotation;
  Eigen::Vector3f rotation_eig;
  mppi::math::Quat2EulerNWU(q, rotation.x, rotation.y, rotation.z);
  rotation_eig << rotation.x, rotation.y, rotation.z;

  // Create output variables
  float3 output_eig, output_array;
  Eigen::Vector3f output_all_eig;

  // Ground Truth
  Eigen::Vector3f correct_value = origin_eig + R_eig * body_pose_eig;

  // Our Methods
  mppi::math::bodyOffsetToWorldPoseEuler(body_pose_f3, origin_f3, rotation_eig, output_eig);
  mppi::math::bodyOffsetToWorldPoseEuler(body_pose_f3, origin_f3, rotation, output_array);
  mppi::math::bodyOffsetToWorldPoseEuler(body_pose_eig, origin_eig, rotation_eig, output_all_eig);

  EXPECT_NEAR(output_eig.x, correct_value.x(), tol);
  EXPECT_NEAR(output_eig.y, correct_value.y(), tol);
  EXPECT_NEAR(output_eig.z, correct_value.z(), tol);
  EXPECT_NEAR(output_array.x, correct_value.x(), tol);
  EXPECT_NEAR(output_array.y, correct_value.y(), tol);
  EXPECT_NEAR(output_array.z, correct_value.z(), tol);
  EXPECT_NEAR(output_all_eig.x(), correct_value.x(), tol);
  EXPECT_NEAR(output_all_eig.y(), correct_value.y(), tol);
  EXPECT_NEAR(output_all_eig.z(), correct_value.z(), tol);
}

TEST(MATH_UTILS, SkewSymmetricMatrixSameAsCrossProd)
{
  Eigen::Vector3f a(1, 2, 3);
  Eigen::Vector3f b(8, 3, 9);
  eigen_assert_float_eq<Eigen::Vector3f>(a.cross(b), mppi::math::skewSymmetricMatrix(a) * b);
}

namespace mm = mppi::matrix_multiplication;
namespace mp1 = mppi::p1;
namespace mp2 = mppi::p2;

template <int M, int K, int N, mp1::Parallel1Dir PARALLELIZATION_DIR, mm::MAT_OP A_OP = mm::MAT_OP::NONE,
          mm::MAT_OP B_OP = mm::MAT_OP::NONE, class T = float>
__global__ void gemm1d(T* A, T* B, T* C, T alpha = 1, T beta = 0, int shared_mem_size = 0)
{
  extern __shared__ float buff[];
  T* A_p;
  T* B_p;
  if (shared_mem_size != 0)
  {
    A_p = (T*)&buff[0];
    B_p = &A_p[M * K];
    mp1::loadArrayParallel<M * K, PARALLELIZATION_DIR, T>(A_p, 0, A, 0);
    mp1::loadArrayParallel<K * N, PARALLELIZATION_DIR, T>((T*)&buff[0], M * K, B, 0);
    __syncthreads();
  }
  else
  {
    A_p = A;
    B_p = B;
  }

  mm::gemm1<M, K, N, PARALLELIZATION_DIR, T>(A_p, B_p, C, alpha, beta, A_OP, B_OP);
}

template <int M, int K, int N>
__global__ void gemm1dBatchKernel(const float* A, const float* B, float* C)
{
  const float* A_local = &A[M * K * threadIdx.x];
  const float* B_local = &B[K * N * threadIdx.x];
  float* C_local = &C[M * N * threadIdx.x];
  mm::gemm1<M, K, N, mp1::Parallel1Dir::THREAD_Y>(A_local, B_local, C_local);
}

template <int M, int K, int N, mm::MAT_OP A_OP, mm::MAT_OP B_OP, mp1::Parallel1Dir P_DIR, class T = float>
float2 testMatMulHost(int size, T alpha = 1, T beta = 0)
{
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A_MAT;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> B_MAT;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C_MAT;

  A_MAT A;
  B_MAT B;
  C_MAT C_Eigen = C_MAT::Ones(M, N);
  C_MAT C_ours = C_Eigen;
  C_Eigen *= beta;
  float eigen_time_ms = 0;

  if (A_OP == mm::MAT_OP::NONE && B_OP == mm::MAT_OP::NONE)
  {
    A = A_MAT::Random(M, K);
    B = B_MAT::Random(K, N);
    auto start = std::chrono::steady_clock::now();
    C_Eigen += A * B * alpha;
    auto stop = std::chrono::steady_clock::now();
    eigen_time_ms = mppi::math::timeDiffms(stop, start);
  }
  else if (A_OP == mm::MAT_OP::TRANSPOSE && B_OP == mm::MAT_OP::NONE)
  {
    A = A_MAT::Random(K, M);
    B = B_MAT::Random(K, N);
    auto start = std::chrono::steady_clock::now();
    C_Eigen += A.transpose() * B * alpha;
    auto stop = std::chrono::steady_clock::now();
    eigen_time_ms = mppi::math::timeDiffms(stop, start);
  }
  if (A_OP == mm::MAT_OP::NONE && B_OP == mm::MAT_OP::TRANSPOSE)
  {
    A = A_MAT::Random(M, K);
    B = B_MAT::Random(N, K);
    auto start = std::chrono::steady_clock::now();
    C_Eigen += A * B.transpose() * alpha;
    auto stop = std::chrono::steady_clock::now();
    eigen_time_ms = mppi::math::timeDiffms(stop, start);
  }
  else if (A_OP == mm::MAT_OP::TRANSPOSE && B_OP == mm::MAT_OP::TRANSPOSE)
  {
    A = A_MAT::Random(K, M);
    B = B_MAT::Random(N, K);
    auto start = std::chrono::steady_clock::now();
    C_Eigen += A.transpose() * B.transpose() * alpha;
    auto stop = std::chrono::steady_clock::now();
    eigen_time_ms = mppi::math::timeDiffms(stop, start);
  }

  auto our_start = std::chrono::steady_clock::now();
  mm::gemm1<M, K, N, P_DIR, T>(A.data(), B.data(), C_ours.data(), alpha, beta, A_OP, B_OP);
  auto our_stop = std::chrono::steady_clock::now();
  float ours_time_ms = mppi::math::timeDiffms(our_stop, our_start);
  double avg_err = (C_Eigen - C_ours).norm() / C_Eigen.size();
  EXPECT_TRUE(avg_err < 1e-7) << "C_Eigen:\n" << C_Eigen << "\nC_ours:\n" << C_ours;
  return make_float2(eigen_time_ms, ours_time_ms);
}

template <int M, int K, int N, mm::MAT_OP A_OP, mm::MAT_OP B_OP, mp1::Parallel1Dir P_DIR, class T = float>
float2 testMatMulp1(int size, T alpha = 1, T beta = 0)
{
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A_MAT;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> B_MAT;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C_MAT;

  cudaStream_t stream;
  cudaStreamCreate(&stream);
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  A_MAT A;
  B_MAT B;
  C_MAT C_CPU = C_MAT::Ones(M, N);
  C_MAT C_GPU = C_CPU;
  C_CPU *= beta;

  // Device Memory Locations
  T* A_d;
  T* B_d;
  T* C_d;
  dim3 block(1, 1, 1);
  dim3 grid(1, 1, 1);

  HANDLE_ERROR(cudaMalloc((void**)&A_d, sizeof(T) * M * K));
  HANDLE_ERROR(cudaMalloc((void**)&B_d, sizeof(T) * K * N));
  HANDLE_ERROR(cudaMalloc((void**)&C_d, sizeof(T) * M * N));
  float cpu_time_ms = 0;

  if (A_OP == mm::MAT_OP::NONE && B_OP == mm::MAT_OP::NONE)
  {
    A = A_MAT::Random(M, K);
    B = B_MAT::Random(K, N);
    auto start = std::chrono::steady_clock::now();
    C_CPU += A * B * alpha;
    auto stop = std::chrono::steady_clock::now();
    cpu_time_ms = mppi::math::timeDiffms(stop, start);
  }
  else if (A_OP == mm::MAT_OP::TRANSPOSE && B_OP == mm::MAT_OP::NONE)
  {
    A = A_MAT::Random(K, M);
    B = B_MAT::Random(K, N);
    auto start = std::chrono::steady_clock::now();
    C_CPU += A.transpose() * B * alpha;
    auto stop = std::chrono::steady_clock::now();
    cpu_time_ms = mppi::math::timeDiffms(stop, start);
  }
  if (A_OP == mm::MAT_OP::NONE && B_OP == mm::MAT_OP::TRANSPOSE)
  {
    A = A_MAT::Random(M, K);
    B = B_MAT::Random(N, K);
    auto start = std::chrono::steady_clock::now();
    C_CPU += A * B.transpose() * alpha;
    auto stop = std::chrono::steady_clock::now();
    cpu_time_ms = mppi::math::timeDiffms(stop, start);
  }
  else if (A_OP == mm::MAT_OP::TRANSPOSE && B_OP == mm::MAT_OP::TRANSPOSE)
  {
    A = A_MAT::Random(K, M);
    B = B_MAT::Random(N, K);
    auto start = std::chrono::steady_clock::now();
    C_CPU += A.transpose() * B.transpose() * alpha;
    auto stop = std::chrono::steady_clock::now();
    cpu_time_ms = mppi::math::timeDiffms(stop, start);
  }

  int grid_dim = 1;

  if (P_DIR == mp1::Parallel1Dir::THREAD_X)
  {
    block.x = size;
  }
  else if (P_DIR == mp1::Parallel1Dir::THREAD_Y)
  {
    block.y = size;
  }
  else if (P_DIR == mp1::Parallel1Dir::THREAD_Z)
  {
    block.z = size;
  }
  else if (P_DIR == mp1::Parallel1Dir::GLOBAL_X)
  {
    grid_dim = (M * N - 1) / (size) + 1;
    block.x = size;
    grid.x = grid_dim;
  }
  else if (P_DIR == mp1::Parallel1Dir::GLOBAL_Y)
  {
    grid_dim = (M * N - 1) / (size) + 1;
    block.y = size;
    grid.y = grid_dim;
  }
  else if (P_DIR == mp1::Parallel1Dir::GLOBAL_Z)
  {
    grid_dim = (M * N - 1) / (size) + 1;
    block.y = size;
    grid.y = grid_dim;
  }
  int shared_mem_num = ((M * K + K * N) * sizeof(T) <= 1 << 16) ? (M * K + K * N) : 0;

  HANDLE_ERROR(cudaMemcpyAsync(A_d, A.data(), sizeof(T) * M * K, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(B_d, B.data(), sizeof(T) * K * N, cudaMemcpyHostToDevice, stream));
  HANDLE_ERROR(cudaMemcpyAsync(C_d, C_GPU.data(), sizeof(T) * M * N, cudaMemcpyHostToDevice, stream));
  cudaEventRecord(start, stream);
  gemm1d<M, K, N, P_DIR, A_OP, B_OP, T>
      <<<grid, block, shared_mem_num * sizeof(T), stream>>>(A_d, B_d, C_d, alpha, beta, shared_mem_num);
  cudaEventRecord(stop, stream);
  HANDLE_ERROR(cudaMemcpyAsync(C_GPU.data(), C_d, sizeof(T) * M * N, cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
  HANDLE_ERROR(cudaEventSynchronize(stop));

  float gpu_time_ms = 0;
  HANDLE_ERROR(cudaEventElapsedTime(&gpu_time_ms, start, stop));

  double avg_err = (C_CPU - C_GPU).norm() / C_CPU.size();
  EXPECT_TRUE(avg_err < 1e-7) << "C_CPU:\n" << C_CPU << "\nC_GPU:\n" << C_GPU;
  cudaFree(A_d);
  cudaFree(B_d);
  cudaFree(C_d);
  return make_float2(cpu_time_ms, gpu_time_ms);
};

TEST(MATH_UTILS, float_BasicMatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<10, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<10, 2, 50, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(256, 1, 0);
}

TEST(MATH_UTILS, float_BasicMatrixMult_y)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Y, float>(16, 1, 0);
  testMatMulp1<10, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Y, float>(16, 1, 0);
  testMatMulp1<10, 2, 50, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Y, float>(256, 1, 0);
}

TEST(MATH_UTILS, float_BasicMatrixMult_z)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, float>(16, 1, 0);
  testMatMulp1<10, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, float>(16, 1, 0);
  testMatMulp1<10, 2, 50, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, float>(64, 1, 0);
}

TEST(MATH_UTILS, float_BasicMatrixMult_none)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::NONE, float>(16, 1, 0);
  testMatMulp1<10, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::NONE, float>(16, 1, 0);
  testMatMulp1<10, 2, 50, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::NONE, float>(256, 1, 0);
}

TEST(MATH_UTILS, float_TransposeA_MatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, float_TransposeA_TransposeB_MatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, float_TransposeB_MatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, int_BasicMatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, int>(16, 1, 0);
}

TEST(MATH_UTILS, double_BasicMatrixMult_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, double>(16, 1, 0);
}

TEST(MATH_UTILS, float_MatrixMult_UnalignedK_x)
{
  testMatMulp1<3, 17, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 17, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 17, 5, mm::MAT_OP::NONE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 1, 4, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, float_MatrixMult_alignedK2_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 2, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 2, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, float_MatrixMult_alignedK4_x)
{
  testMatMulp1<3, 4, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 4, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 4, 5, mm::MAT_OP::NONE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
  testMatMulp1<3, 4, 5, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::TRANSPOSE, mp1::Parallel1Dir::THREAD_X, float>(16, 1, 0);
}

TEST(MATH_UTILS, float_BasicMatrixMult_alpha_z)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, float>(16, 3.0, 0);
}

TEST(MATH_UTILS, float_BasicMatrixMult_alpha_beta_z)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_Z, float>(16, 3.0, 0.2);
}

TEST(MATH_UTILS, float_BasicMatrixMult_alpha_beta_x)
{
  testMatMulp1<3, 2, 5, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(5, 3.0, 0.2);
}

TEST(MATH_UTILS, HostMatrixMult)
{
  const int M = 10;
  const int K = 5;
  const int N = 6;
  const mm::MAT_OP A_OP = mm::MAT_OP::NONE;
  const mm::MAT_OP B_OP = mm::MAT_OP::NONE;
  const mp1::Parallel1Dir P_DIR = mp1::Parallel1Dir::THREAD_X;
  using T = float;
  T alpha = 1.0;
  T beta = 0.0;
  testMatMulHost<M, K, N, A_OP, B_OP, P_DIR, T>(alpha, beta);
}
TEST(MATH_UTILS, SpeedSharedMemMatrixMult)
{
  const int NUM_ITERATIONS = 100;
  float2 total_ms = make_float2(0, 0);
  const int thread_dim = 1024;
  for (int i = 0; i < NUM_ITERATIONS; i++)
  {
    total_ms +=
        testMatMulp1<128, 256, 128, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(
            thread_dim, 3.0, 0.2);
  }
  std::cout << NUM_ITERATIONS << " GPU Shared Mem Matrix Multiplications took " << total_ms.y / NUM_ITERATIONS
            << " ms on average" << std::endl;
  std::cout << NUM_ITERATIONS << " CPU Shared Mem Matrix Multiplications took " << total_ms.x / NUM_ITERATIONS
            << " ms on average" << std::endl;
}

TEST(MATH_UTILS, SpeedLargeMatrixMult)
{
  const int NUM_ITERATIONS = 5;
  float2 total_ms = make_float2(0, 0);
  const int thread_dim = 1024;
  for (int i = 0; i < NUM_ITERATIONS; i++)
  {
    total_ms +=
        testMatMulp1<1024, 2048, 1024, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(
            thread_dim, 3.0, 0.2);
  }
  std::cout << NUM_ITERATIONS << " GPU Matrix Multiplications took " << total_ms.y / NUM_ITERATIONS << " ms on average"
            << std::endl;
  std::cout << NUM_ITERATIONS << " CPU Matrix Multiplications took " << total_ms.x / NUM_ITERATIONS << " ms on average"
            << std::endl;
}

TEST(MATH_UTILS, SpeedLargeMatrixMultGlobal)
{
  const int NUM_ITERATIONS = 10;
  float2 total_ms = make_float2(0, 0);
  const int thread_dim = 1024;
  for (int i = 0; i < NUM_ITERATIONS; i++)
  {
    total_ms +=
        testMatMulp1<1024, 2048, 1024, mm::MAT_OP::TRANSPOSE, mm::MAT_OP::NONE, mp1::Parallel1Dir::GLOBAL_X, float>(
            thread_dim, 3.0, 0.2);
  }
  std::cout << NUM_ITERATIONS << " GPU Matrix Multiplications took " << total_ms.y / NUM_ITERATIONS << " ms on average"
            << std::endl;
  std::cout << NUM_ITERATIONS << " CPU Matrix Multiplications took " << total_ms.x / NUM_ITERATIONS << " ms on average"
            << std::endl;
}

TEST(MATH_UTILS, SpeedSmallMatrixMult)
{
  const int NUM_ITERATIONS = 10;
  float2 total_ms = make_float2(0, 0);
  const int thread_dim = 16;
  for (int i = 0; i < NUM_ITERATIONS; i++)
  {
    total_ms += testMatMulp1<5, 4, 3, mm::MAT_OP::NONE, mm::MAT_OP::NONE, mp1::Parallel1Dir::THREAD_X, float>(
        thread_dim, 3.0, 0.2);
  }
  std::cout << NUM_ITERATIONS << " GPU Matrix Multiplications took " << total_ms.y / NUM_ITERATIONS << " ms on average"
            << std::endl;
  std::cout << NUM_ITERATIONS << " CPU Matrix Multiplications took " << total_ms.x / NUM_ITERATIONS << " ms on average"
            << std::endl;
}

template <int M, int N, mp1::Parallel1Dir PARALLELIZATION_DIR = mp1::Parallel1Dir::THREAD_Y, class T = float>
__global__ void GaussJordanKernel(T* A_d)
{
  int batch_idx;
  if (PARALLELIZATION_DIR == mp1::Parallel1Dir::THREAD_X)
  {
    batch_idx = blockIdx.x;
  }
  else if (PARALLELIZATION_DIR == mp1::Parallel1Dir::THREAD_Y)
  {
    batch_idx = blockIdx.y;
  }
  else if (PARALLELIZATION_DIR == mp1::Parallel1Dir::THREAD_Z)
  {
    batch_idx = blockIdx.z;
  }
  extern __shared__ float A_shared_s[];
  T* A_shared = (T*)A_shared_s;
  mp1::loadArrayParallel<M * N, PARALLELIZATION_DIR, T>(A_shared, 0, A_d + batch_idx * M * N, 0);
  __syncthreads();
  mm::GaussJordanElimination<M, N, PARALLELIZATION_DIR, T>(A_shared);
  __syncthreads();
  mp1::loadArrayParallel<M * N, PARALLELIZATION_DIR, T>(A_d + batch_idx * M * N, 0, A_shared, 0);
}

TEST(MATH_UTILS, GaussJordanFactorizationAgainstInverse)
{
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  const int M = 3;
  const int N = 2 * M;
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(M, N);
  Eigen::MatrixXf A_result = A;
  Eigen::MatrixXf first_A;
  Eigen::MatrixXf A_inv;
  dim3 block_size(1, 1, 1);
  dim3 grid_size(1, 1, 1);
  int shared_mem_size = sizeof(float) * M * N;
  float* A_d;
  cudaMalloc((void**)&A_d, sizeof(float) * M * N);

  float max_error = 0.0f;
  Eigen::MatrixXf diff;
  first_A = Eigen::MatrixXf::Random(M, M);
  A_inv = first_A.inverse();
  for (int tdx = 1; tdx < 64; tdx++)
  {
    A << first_A, Eigen::MatrixXf::Identity(M, M);
    cudaMemcpyAsync(A_d, A.data(), sizeof(float) * M * N, cudaMemcpyHostToDevice, stream);
    block_size.x = tdx;
    GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_X><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
    cudaMemcpyAsync(A_result.data(), A_d, sizeof(float) * M * N, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    diff = A_result.block(0, M, M, N - M) - A_inv;
    max_error = diff.norm();
    ASSERT_NEAR(max_error, 0.0f, sqrtf(1e-4 * M * N)) << "failed for " << tdx << " parallel x threads.\nDiff:\n"
                                                      << diff << "\nCPU:\n"
                                                      << A_inv << "\nGPU:\n"
                                                      << A_result.block(0, M, M, N - M);
  }
  block_size = dim3(1, 1, 1);
  for (int tdy = 1; tdy < 64; tdy++)
  {
    // first_A = Eigen::MatrixXf::Random(M, M);
    A << first_A, Eigen::MatrixXf::Identity(M, M);
    // A_inv = first_A.inverse();
    cudaMemcpyAsync(A_d, A.data(), sizeof(float) * M * N, cudaMemcpyHostToDevice, stream);
    block_size.y = tdy;
    GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_Y><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
    cudaMemcpyAsync(A_result.data(), A_d, sizeof(float) * M * N, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    diff = A_result.block(0, M, M, N - M) - A_inv;
    max_error = diff.lpNorm<Eigen::Infinity>();
    ASSERT_NEAR(max_error, 0.0f, 1e-4 * A_inv.norm()) << "failed for " << tdy << " parallel y threads.\nDiff:\n"
                                                      << diff << "\nCPU:\n"
                                                      << A_inv << "\nGPU:\n"
                                                      << A_result.block(0, M, M, N - M);
  }
  block_size = dim3(1, 1, 1);
  for (int tdz = 1; tdz < 64; tdz++)
  {
    // first_A = Eigen::MatrixXf::Random(M, M);
    A << first_A, Eigen::MatrixXf::Identity(M, M);
    // A_inv = first_A.inverse();
    cudaMemcpyAsync(A_d, A.data(), sizeof(float) * M * N, cudaMemcpyHostToDevice, stream);
    block_size.z = tdz;
    GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_Z><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
    cudaMemcpyAsync(A_result.data(), A_d, sizeof(float) * M * N, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    diff = A_result.block(0, M, M, N - M) - A_inv;
    max_error = diff.lpNorm<Eigen::Infinity>();
    ASSERT_NEAR(max_error, 0.0f, 1e-4 * A_inv.norm()) << "failed for " << tdz << " parallel z threads.\nDiff:\n"
                                                      << diff << "\nCPU:\n"
                                                      << A_inv << "\nGPU:\n"
                                                      << A_result.block(0, M, M, N - M);
  }
  cudaFree(A_d);
}

TEST(MATH_UTILS, GaussJordanEliminationColumnSkip)
{
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  const int M = 2;
  const int N = 5;
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(M, N);
  Eigen::MatrixXf A_result = A;
  // A << 0, 2, 3, 1, 1, 4;
  A << 1, 2, 2, 3, 4, 2, 4, 4, 1, 8;

  Eigen::MatrixXf b_d = Eigen::MatrixXf::Zero(M, N - M);
  // b_d << 2.5, 1.5;
  b_d << 2, 0, 4, 0, 1, 0;
  dim3 block_size(1, 1, 1);
  dim3 grid_size(1, 1, 1);
  int shared_mem_size = sizeof(float) * M * N;
  float* A_d;
  cudaMalloc((void**)&A_d, sizeof(float) * M * N);

  float max_error = 0.0f;
  Eigen::MatrixXf diff;
  for (int tdx = 1; tdx < 65; tdx++)
  {
    cudaMemcpyAsync(A_d, A.data(), sizeof(float) * M * N, cudaMemcpyHostToDevice, stream);
    block_size.x = tdx;
    GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_X><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
    cudaMemcpyAsync(A_result.data(), A_d, sizeof(float) * M * N, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    diff = A_result.block(0, M, M, N - M) - b_d;
    max_error = diff.norm();
    ASSERT_NEAR(max_error, 0.0f, 1e-4 * M * N) << "failed for " << tdx << " parallel x threads.\nDiff:\n"
                                               << diff << "\nCPU:\n"
                                               << b_d << "\nGPU:\n"
                                               << A_result.block(0, M, M, N - M);
  }
  cudaFree(A_d);
}

TEST(MATH_UTILS, GaussJordanEliminationRowSwap)
{
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  const int M = 2;
  const int N = 3;
  Eigen::MatrixXf A = Eigen::MatrixXf::Zero(M, N);
  Eigen::MatrixXf A_result = A;
  A << 0, 2, 3, 1, 1, 4;

  Eigen::MatrixXf b_d = Eigen::MatrixXf::Zero(M, N - M);
  b_d << 2.5, 1.5;
  dim3 block_size(1, 1, 1);
  dim3 grid_size(1, 1, 1);
  int shared_mem_size = sizeof(float) * M * N;
  float* A_d;
  cudaMalloc((void**)&A_d, sizeof(float) * M * N);

  float max_error = 0.0f;
  Eigen::MatrixXf diff;
  for (int tdx = 1; tdx < 65; tdx++)
  {
    cudaMemcpyAsync(A_d, A.data(), sizeof(float) * M * N, cudaMemcpyHostToDevice, stream);
    block_size.x = tdx;
    GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_X><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
    cudaMemcpyAsync(A_result.data(), A_d, sizeof(float) * M * N, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    diff = A_result.block(0, M, M, N - M) - b_d;
    max_error = diff.norm();
    ASSERT_NEAR(max_error, 0.0f, 1e-4 * M * N) << "failed for " << tdx << " parallel x threads.\nDiff:\n"
                                               << diff << "\nCPU:\n"
                                               << b_d << "\nGPU:\n"
                                               << A_result.block(0, M, M, N - M);
  }
  cudaFree(A_d);
}

TEST(MATH_UTILS, GaussJordanFactorizationBatched)
{
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  const int M = 3;
  const int N = 2 * M;
  dim3 block_size(1, 1, 1);
  dim3 grid_size(1, 1, 1);
  int shared_mem_size = sizeof(double) * M * N;
  double* A_d = nullptr;
  // cudaMalloc((void**)&A_d, sizeof(double) * M * N);
  for (int batches = 1; batches < 1000; batches++)
  {
    std::vector<double> A_batch(batches * M * N);
    std::vector<double> A_cpu(batches * M * N);
    std::vector<double> A_gpu(batches * M * N);
    if (A_d)
    {
      cudaFree(A_d);
    }
    cudaMalloc((void**)&A_d, sizeof(double) * batches * M * N);

    grid_size.x = batches;
    // Fill in A_batch
    for (int i = 0; i < batches; i++)
    {
      Eigen::Map<Eigen::MatrixXd> A_batch_i(&A_batch.data()[i * M * N], M, N);
      Eigen::Map<Eigen::MatrixXd> A_cpu_i(&A_cpu.data()[i * M * N], M, N);
      A_batch_i << Eigen::MatrixXd::Random(M, M) + Eigen::MatrixXd::Identity(M, M), Eigen::MatrixXd::Identity(M, M);
      A_cpu_i << Eigen::MatrixXd::Identity(M, M), A_batch_i.block(0, 0, M, M).inverse();
    }

    for (int tdx = 1; tdx < 128; tdx++)
    {
      // Copy over to GPU and run GJ Elimination
      cudaMemcpyAsync(A_d, A_batch.data(), sizeof(double) * batches * M * N, cudaMemcpyHostToDevice, stream);
      block_size.x = tdx;
      GaussJordanKernel<M, N, mp1::Parallel1Dir::THREAD_X><<<grid_size, block_size, shared_mem_size, stream>>>(A_d);
      cudaMemcpyAsync(A_gpu.data(), A_d, sizeof(double) * batches * M * N, cudaMemcpyDeviceToHost, stream);
      cudaStreamSynchronize(stream);
      for (int i = 0; i < A_gpu.size(); i++)
      {
        // double tolerance = fabsf(A_cpu[i]) < 1 ? 1e-3 : 1e-3 * fabsf(A_cpu[i]);
        double tolerance = 0.1f;
        ASSERT_LT(fabsf(A_gpu[i] - A_cpu[i]), tolerance)
            << i % (M * N) << "th item in "
            << " batch: " << i / (M * N) << " out of " << batches << ", CPU: " << A_cpu[i] << ", GPU: " << A_gpu[i];
      }
    }
  }
  cudaFree(A_d);
}

/** 2 Dimensional Parallelization Tests **/

template <int M, int K, int N, mp2::Parallel2Dir PARALLELIZATION_DIR>
__global__ void gemm2d(const float* A, const float* B, float* C)
{
  mm::gemm2<M, K, N, PARALLELIZATION_DIR>(A, B, C);
}

TEST(MATH_UTILS, DISABLED_Matrix3fMultiplicationP2)
{
  const int M = 300;
  const int K = 30;
  const int N = 3;
  const int max_thread_size = 1;
  typedef Eigen::Matrix<float, M, K> A_mat;
  typedef Eigen::Matrix<float, K, N> B_mat;
  typedef Eigen::Matrix<float, M, N> C_mat;

  cudaEvent_t start;
  cudaEvent_t stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaStream_t stream;
  cudaStreamCreate(&stream);

  A_mat A = A_mat::Random();
  B_mat B = B_mat::Random();
  C_mat C_CPU = A * B;
  C_mat C_GPU;
  // std::cout << "C_CPU:\n" << C_CPU << std::endl;
  float* A_d;
  float* B_d;
  float* C_d;
  dim3 block(1, 1, 1);
  dim3 grid(1, 1, 1);

  HANDLE_ERROR(cudaMalloc((void**)&A_d, sizeof(float) * A.size()));
  HANDLE_ERROR(cudaMalloc((void**)&B_d, sizeof(float) * B.size()));
  HANDLE_ERROR(cudaMalloc((void**)&C_d, sizeof(float) * C_CPU.size()));

  auto inner_loop_before = [&](int size1, int size2, mp2::Parallel2Dir P_DIR) {
    A = A_mat::Random();
    B = B_mat::Random();
    C_CPU = A * B;
    block = dim3(1, 1, 1);
    if (P_DIR == mp2::Parallel2Dir::THREAD_XY)
    {
      block.x = size1;
      block.y = max(size2, 1);
    }
    else if (P_DIR == mp2::Parallel2Dir::THREAD_XZ)
    {
      block.x = size1;
      block.z = max(min(size2, 64), 1);
    }
    else if (P_DIR == mp2::Parallel2Dir::THREAD_YX)
    {
      block.y = size1;
      block.x = max(size2, 1);
    }
    else if (P_DIR == mp2::Parallel2Dir::THREAD_YZ)
    {
      block.y = size1;
      block.z = max(min(size2, 64), 1);
    }
    else if (P_DIR == mp2::Parallel2Dir::THREAD_ZY)
    {
      block.z = max(min(size1, 64), 1);
      block.y = max(size2, 1);
    }
    else if (P_DIR == mp2::Parallel2Dir::THREAD_ZX)
    {
      block.z = max(min(size1, 64), 1);
      block.x = max(size2, 1);
    }
    cudaEventRecord(start, stream);
    HANDLE_ERROR(cudaMemcpyAsync(A_d, A.data(), sizeof(float) * A.size(), cudaMemcpyHostToDevice, stream));
    HANDLE_ERROR(cudaMemcpyAsync(B_d, B.data(), sizeof(float) * B.size(), cudaMemcpyHostToDevice, stream));
  };

  auto inner_loop_after = [&](int size1, int size2, mp2::Parallel2Dir P_DIR) {
    HANDLE_ERROR(cudaMemcpyAsync(C_GPU.data(), C_d, sizeof(float) * C_GPU.size(), cudaMemcpyDeviceToHost, stream));
    HANDLE_ERROR(cudaStreamSynchronize(stream));
    cudaEventRecord(stop, stream);
    float duration = 0;
    cudaEventElapsedTime(&duration, start, stop);
    // std::cout << "C_GPU " << " with " << block_x_size << " parallelization:\n" << C_GPU << std::endl;
    // std::cout << block_x_size << " parallelization diff " << duration << " ms:\n" << C_CPU - C_GPU << std::endl;
    double avg_err = (C_CPU - C_GPU).norm() / C_CPU.size();
    // std::cout << block_x_size << " parallelization diff " << duration << " ms: " << avg_err << std::endl;
    // std::string dir_s = "none";
    // if (P_DIR == mp2::Parallel2Dir::THREAD_XY)
    // {
    //   dir_s = "xy";
    // } else if (P_DIR == mp2::Parallel2Dir::THREAD_XZ)
    // {
    //   dir_s = "xz";
    // } else if (P_DIR == mp2::Parallel2Dir::THREAD_YX)
    // {
    //   dir_s = "yx";
    // } else if (P_DIR == mp2::Parallel2Dir::THREAD_YZ)
    // {
    //   dir_s = "yz";
    // } else if (P_DIR == mp2::Parallel2Dir::THREAD_ZY)
    // {
    //   dir_s = "zy";
    // } else if (P_DIR == mp2::Parallel2Dir::THREAD_ZX)
    // {
    //   dir_s = "zx";
    // }
    // printf("%d-%d-%d parallelization in %s-dir, time elapsed %f ms, diff: %f\n", block.x, block.y, block.z,
    // dir_s.c_str(), duration, avg_err);
    printf("%d-%d-%d parallelization, time elapsed %f ms, diff: %f\n", block.x, block.y, block.z, duration, avg_err);
    ASSERT_TRUE(avg_err < 1e-07);
  };

  for (int block_size_1 = 1; block_size_1 <= max_thread_size; block_size_1++)
  {
    int block_size_2 = max_thread_size - block_size_1;
    inner_loop_before(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_XY);
    gemm2d<M, K, N, mp2::Parallel2Dir::THREAD_XY><<<grid, block, 0, stream>>>(A_d, B_d, C_d);
    inner_loop_after(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_XY);
  }
  for (int block_size_1 = 1; block_size_1 <= max_thread_size; block_size_1++)
  {
    int block_size_2 = max_thread_size - block_size_1;
    inner_loop_before(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_XZ);
    gemm2d<M, K, N, mp2::Parallel2Dir::THREAD_XZ><<<grid, block, 0, stream>>>(A_d, B_d, C_d);
    inner_loop_after(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_XZ);
  }
  for (int block_size_1 = 1; block_size_1 <= max_thread_size; block_size_1++)
  {
    int block_size_2 = max_thread_size - block_size_1;
    inner_loop_before(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_YZ);
    gemm2d<M, K, N, mp2::Parallel2Dir::THREAD_YZ><<<grid, block, 0, stream>>>(A_d, B_d, C_d);

    inner_loop_after(block_size_1, block_size_2, mp2::Parallel2Dir::THREAD_YZ);
    // std::cout << "C_GPU:\n" << C_GPU << "\nC_CPU:\n" << C_CPU << std::endl;
  }

  cudaFree(A_d);
  cudaFree(B_d);
  cudaFree(C_d);
}
