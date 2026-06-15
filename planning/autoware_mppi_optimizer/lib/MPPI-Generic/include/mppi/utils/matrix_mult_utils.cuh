//
// Created by Bogdan on 8/20/23
//
#pragma once
#include <mppi/utils/parallel_utils.cuh>

namespace mppi
{
namespace matrix_multiplication
{
/**
 * Utility Functions
 **/
inline __host__ __device__ int2 const unravelColumnMajor(const int index, const int num_rows)
{
  int col = index / num_rows;
  int row = index % num_rows;
  return make_int2(row, col);
}

inline __host__ __device__ int2 const unravelRowMajor(const int index, const int num_cols)
{
  int row = index / num_cols;
  int col = index % num_cols;
  return make_int2(row, col);
}
inline __host__ __device__ constexpr int columnMajorIndex(const int row, const int col, const int num_rows)
{
  return col * num_rows + row;
}

inline __host__ __device__ constexpr int rowMajorIndex(const int row, const int col, const int num_cols)
{
  return row * num_cols + col;
}

/**
 * Utility Classes
 **/
enum class MAT_OP : int
{
  NONE = 0,
  TRANSPOSE
};

template <int M, int N, class T = float>
class devMatrix
{
public:
  T* data = nullptr;
  static constexpr int rows = M;
  static constexpr int cols = N;
  devMatrix(T* n_data)
  {
    data = n_data;
  };

  T operator()(const int i, const int j) const
  {
    return data[columnMajorIndex(i, j, rows)];
  }
};

/**
 * @brief GEneral Matrix Multiplication
 * Conducts the operation
 * C = alpha * op(A) * op(B) + beta * C
 * on matrices of type T
 * TODO: Add transpose options like cuBLAS GEMM
 * Inputs:
 * op(A) - T-type column-major matrix of size M * K, stored in shared/global mem
 * op(B) - T-type column-major matrix of size K * N, stored in shared/global mem
 * alpha - T-type to multiply A * B
 * beta - T-type multipling C
 * A_OP - whether or not you should use A or A transpose
 * B_OP - whether or not you should use B or B transpose
 * Outputs:
 * C - float column-major matrix of size M * N, stored in shared/global mem
 *
 */
template <int M, int K, int N, p1::Parallel1Dir P_DIR = p1::Parallel1Dir::THREAD_Y, class T = float>
inline __device__ __host__ void gemm1(const T* A, const T* B, T* C, const T alpha = 1, const T beta = 0,
                                      const MAT_OP A_OP = MAT_OP::NONE, const MAT_OP B_OP = MAT_OP::NONE)
{
  int parallel_index;
  int parallel_step;
  int p, k;
  p1::getParallel1DIndex<P_DIR>(parallel_index, parallel_step);
  int2 mn;
  const bool all_stride = (A_OP == MAT_OP::NONE) && (B_OP == MAT_OP::TRANSPOSE);
  for (p = parallel_index; p < M * N; p += parallel_step)
  {
    T accumulator = 0;
    mn = unravelColumnMajor(p, M);
    if (K % 4 == 0 && sizeof(type4<T>) <= 16 && !all_stride)
    {  // Fetch 4 B values using single load memory operator of up to 128 bits since B is contiguous wrt k
      __UNROLL(10)
      for (k = 0; k < K; k += 4)
      {
        if (A_OP == MAT_OP::NONE && B_OP == MAT_OP::NONE)
        {
          const type4<T> b_tmp = reinterpret_cast<const type4<T>*>(&B[columnMajorIndex(k, mn.y, K)])[0];
          accumulator += A[columnMajorIndex(mn.x, k + 0, M)] * b_tmp[0];
          accumulator += A[columnMajorIndex(mn.x, k + 1, M)] * b_tmp[1];
          accumulator += A[columnMajorIndex(mn.x, k + 2, M)] * b_tmp[2];
          accumulator += A[columnMajorIndex(mn.x, k + 3, M)] * b_tmp[3];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::NONE)
        {
          const type4<T> b_tmp = reinterpret_cast<const type4<T>*>(&B[columnMajorIndex(k, mn.y, K)])[0];
          const type4<T> a_tmp = reinterpret_cast<const type4<T>*>(&A[rowMajorIndex(mn.x, k, K)])[0];
          accumulator += a_tmp[0] * b_tmp[0];
          accumulator += a_tmp[1] * b_tmp[1];
          accumulator += a_tmp[2] * b_tmp[2];
          accumulator += a_tmp[3] * b_tmp[3];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::TRANSPOSE)
        {
          // const type4<T> b_tmp = reinterpret_cast<const type4<T>*>(&B[columnMajorIndex(k, mn.y, K)])[0];
          const type4<T> a_tmp = reinterpret_cast<const type4<T>*>(&A[rowMajorIndex(mn.x, k, K)])[0];
          accumulator += a_tmp[0] * B[rowMajorIndex(k + 0, mn.y, N)];
          accumulator += a_tmp[1] * B[rowMajorIndex(k + 1, mn.y, N)];
          accumulator += a_tmp[2] * B[rowMajorIndex(k + 2, mn.y, N)];
          accumulator += a_tmp[3] * B[rowMajorIndex(k + 3, mn.y, N)];
        }
      }
    }
    else if (K % 2 == 0 && sizeof(type2<T>) <= 16 && !all_stride)
    {  // Fetch 2 B values using single load memory operator of up to 128 bits since B is contiguous wrt k
      __UNROLL(10)
      for (k = 0; k < K; k += 2)
      {
        if (A_OP == MAT_OP::NONE && B_OP == MAT_OP::NONE)
        {
          const type2<T> b_tmp = reinterpret_cast<const type2<T>*>(&B[columnMajorIndex(k, mn.y, K)])[0];
          accumulator += A[columnMajorIndex(mn.x, k + 0, M)] * b_tmp[0];
          accumulator += A[columnMajorIndex(mn.x, k + 1, M)] * b_tmp[1];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::NONE)
        {
          const type2<T> b_tmp = reinterpret_cast<const type2<T>*>(&B[columnMajorIndex(k, mn.y, K)])[0];
          const type2<T> a_tmp = reinterpret_cast<const type2<T>*>(&A[rowMajorIndex(mn.x, k, K)])[0];
          accumulator += a_tmp[0] * b_tmp[0];
          accumulator += a_tmp[1] * b_tmp[1];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::TRANSPOSE)
        {
          const type2<T> a_tmp = reinterpret_cast<const type2<T>*>(&A[rowMajorIndex(mn.x, k, K)])[0];
          accumulator += a_tmp[0] * B[rowMajorIndex(k + 0, mn.y, N)];
          accumulator += a_tmp[1] * B[rowMajorIndex(k + 1, mn.y, N)];
        }
      }
    }
    else
    {  // Either K is odd or sizeof(T) is large enough that
      T a;
      T b;
      __UNROLL(10)
      for (k = 0; k < K; k++)
      {
        if (A_OP == MAT_OP::NONE && B_OP == MAT_OP::NONE)
        {
          a = A[columnMajorIndex(mn.x, k, M)];
          b = B[columnMajorIndex(k, mn.y, K)];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::NONE)
        {
          a = A[rowMajorIndex(mn.x, k, K)];
          b = B[columnMajorIndex(k, mn.y, K)];
        }
        else if (A_OP == MAT_OP::TRANSPOSE && B_OP == MAT_OP::TRANSPOSE)
        {
          a = A[rowMajorIndex(mn.x, k, K)];
          b = B[rowMajorIndex(k, mn.y, N)];
        }
        else
        {
          a = A[columnMajorIndex(mn.x, k, M)];
          b = B[rowMajorIndex(k, mn.y, N)];
        }

        accumulator += a * b;
      }
    }
    if (beta == 0)
    {  // Special case to remove extraneous memory accesses
      C[p] = alpha * accumulator;
    }
    else
    {
      C[p] = alpha * accumulator + beta * C[p];
    }
  }
}

/**
 * @brief GEneral Matrix Multiplication
 * Conducts the operation
 * C = alpha * A * B + beta * C
 * using two parallelization directions
 * TODO: Add transpose options like cuBLAS GEMM
 * Inputs:
 * A - float column-major matrix of size M * K, stored in shared/global mem
 * B - float column-major matrix of size K * N, stored in shared/global mem
 * alpha - float to multiply A * B
 * beta - float multipling C
 * Outputs:
 * C - float column-major matrix of size M * N, stored in shared/global mem
 *
 */
template <int M, int K, int N, p2::Parallel2Dir P_DIR = p2::Parallel2Dir::THREAD_XY>
inline __device__ void gemm2(const float* A, const float* B, float* C, const float alpha = 1.0f,
                             const float beta = 0.0f)
{
  int m_ind_start;
  int m_ind_size;
  int n_ind_start;
  int n_ind_size;
  p2::getParallel2DIndex<P_DIR>(m_ind_start, n_ind_start, m_ind_size, n_ind_size);
  for (int m = m_ind_start; m < M; m += m_ind_size)
  {
    for (int n = n_ind_start; n < N; n += n_ind_size)
    {
      float accumulator = 0;
      __UNROLL(10)
      for (int k = 0; k < K; k++)
      {
        accumulator += A[columnMajorIndex(m, k, M)] * B[columnMajorIndex(k, n, K)];
      }
      C[columnMajorIndex(m, n, M)] = alpha * accumulator + beta * C[columnMajorIndex(m, n, M)];
    }
  }
}

/**
 * @brief Perform Guass Jordan Elimination in place on columan-major MxN matrix A.
 * Useful for solving Cx = b where A = [C | b] as well as inverting matrices.
 *
 * @tparam M - number of rows
 * @tparam N - number of cols
 * @tparam P_DIR - Parallelization axes for on the GPU
 * @tparam T - type of data in A
 * @param A - column-major matrix of type T with M rows and N cols
 *
 * @return reduced row echelon form of A is returned in A.
 */
template <int M, int N, p1::Parallel1Dir P_DIR = p1::Parallel1Dir::THREAD_Y, class T = float>
inline __host__ __device__ void GaussJordanElimination(T* A)
{
  int p_index, step;
  p1::getParallel1DIndex<P_DIR>(p_index, step);
  int row, col, offset = 0;
  T accumulator;
  for (int i = 0; i < M; i++)
  {
    // Check if row-swap is needed
    row = i;
    while (A[columnMajorIndex(row, i + offset, M)] == 0)
    {
      row++;
      if (row == M)
      {  // column is all zeros, need to move on to the next column
        offset++;
        if (i + offset >= N)
        {  // Ran out of columns to check.
          return;
        }
        row = i;
      }
    }
    // swap rows if needed (row != i)
    for (col = i + p_index; row != i && col < N; col += step)
    {
      accumulator = A[columnMajorIndex(row, col, M)];
      A[columnMajorIndex(row, col, M)] = A[columnMajorIndex(i, col, M)];
      A[columnMajorIndex(i, col, M)] = accumulator;
    }
    // normalize the current row
    accumulator = 1.0f / A[columnMajorIndex(i, i + offset, M)];
    for (col = i + offset + p_index; col < N; col += step)
    {
      A[columnMajorIndex(i, col, M)] *= accumulator;
    }
#ifdef __CUDA_ARCH__
    __syncthreads();
#endif
    // Now eliminate pivot from both rows above and below
    for (row = p_index; row < M; row += step)
    {
      if (row == i)
      {
        continue;
      }
      accumulator = -A[columnMajorIndex(row, i + offset, M)];
      for (col = i + offset; col < N; col++)
      {
        A[columnMajorIndex(row, col, M)] += accumulator * A[columnMajorIndex(i, col, M)];
      }
    }
#ifdef __CUDA_ARCH__
    __syncthreads();
#endif
  }
}

template <p1::Parallel1Dir P_DIR = p1::Parallel1Dir::NONE, int M = 1, int K = 1, int N = 1, class T = float>
void matMult1(const devMatrix<M, K, T>& A, const devMatrix<K, N, T>& B, devMatrix<M, N, T>& C)
{
  gemm1<M, K, N, P_DIR, T>(A.data, B.data, C.data);
}

}  // namespace matrix_multiplication
}  // namespace mppi
