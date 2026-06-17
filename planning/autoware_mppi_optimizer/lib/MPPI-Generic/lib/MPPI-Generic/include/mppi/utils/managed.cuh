/*
 * Software License Agreement (BSD License)
 * Copyright (c) 2013, Georgia Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**********************************************
 * @file managed.cuh
 * @author Grady Williams <gradyrw@gmail.com>
 * @date May 24, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Class to be inherited by classes passed
 * to CUDA kernels. Helps unify stream management.
 ***********************************************/

#ifndef MPPI_MANAGED_CUH_
#define MPPI_MANAGED_CUH_

#include "gpu_err_chk.cuh"
#include <mppi/utils/logger.hpp>

/**
 * @class Managed managed.cuh
 * @brief Class for setting the stream to be used by dynamics and cost functions used
 * by MPPIController.
 *
 * This class has one variable, which is the CUDA stream, and a function which sets the
 * stream. It is meant to be inherited by costs and dynamics classes which are passed into
 * the MPPIController kernels. In the past, this class used unified memory (hence the managed name),
 * so that classes could be passed by reference to CUDA kernels. However, as of right now,
 * the difficulties of using unified memory with multi-threaded CPU programs make getting
 * good performance with unified memory difficult, so this has been removed. Future implementations
 * may bring back the unified memory feature.
 */
class Managed
{
public:
  cudaStream_t stream_ = 0;  ///< The CUDA Stream that the class is bound too. 0 is the default (NULL) stream.

  // true when allocated
  bool GPUMemStatus_ = false;

  Managed(cudaStream_t stream = 0)
  {
    this->bindToStream(stream);
    auto logger = std::make_shared<mppi::util::MPPILogger>();
    setLogger(logger);
  }

  /**
  @brief Sets the stream and synchronizes the device.
  @param stream is the CUDA stream that the object is assigned too.
  */
  void bindToStream(cudaStream_t stream)
  {
    stream_ = stream;
    cudaDeviceSynchronize();
  }

  // REQUIRED: basic interface, make sure to implement in each class
  // GPUSetup - allocates the GPU object
  // freeCudaMem -> deallocates what is setup in GPUSetup
  // getParams, setParams -> gets and sets the parameters
  // paramsToDevice -> copies the parameters over to the GPU side

  // OPTIONAL:
  // printParams
  // other printing methods

  __host__ void setLogger(const mppi::util::MPPILoggerPtr& logger)
  {
    logger_ = logger;
  }

  __host__ void setLogLevel(const mppi::util::LOG_LEVEL& level)
  {
    logger_->setLogLevel(level);
  }

  __host__ mppi::util::MPPILoggerPtr getLogger()
  {
    return logger_;
  }

  __host__ mppi::util::MPPILoggerPtr getLogger() const
  {
    return logger_;
  }

  __device__ __host__ int getGrdSharedSizeBytes() const
  {
    return SHARED_MEM_REQUEST_GRD_BYTES;
  }
  __device__ __host__ int getBlkSharedSizeBytes() const
  {
    return SHARED_MEM_REQUEST_BLK_BYTES;
  }


protected:
  template <class T>
  static T* GPUSetup(T* host_ptr)
  {
    // Allocate enough space on the GPU for the object
    T* device_ptr;
    cudaMalloc((void**)&device_ptr, sizeof(T));
    // Cudamemcpy
    HANDLE_ERROR(cudaMemcpyAsync(device_ptr, host_ptr, sizeof(T), cudaMemcpyHostToDevice, host_ptr->stream_));
    cudaDeviceSynchronize();
    host_ptr->GPUMemStatus_ = true;
    return device_ptr;
  }
  mppi::util::MPPILoggerPtr logger_ = nullptr;

  int SHARED_MEM_REQUEST_GRD_BYTES = 0;  ///< Amount of shared memory we need per BLOCK.
  int SHARED_MEM_REQUEST_BLK_BYTES = 0;  ///< Amount of shared memory we need per ROLLOUT.

  // TODO CRTP template this on the base class for allocation and dealloation
};

#endif /* MPPI_MANAGED_CUH_*/
