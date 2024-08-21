// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
// modify from
// https://github.com/NVIDIA/TensorRT/tree/master/plugin/batchedNMSPlugin
#ifndef TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__BATCHED_NMS_KERNEL_HPP_
#define TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__BATCHED_NMS_KERNEL_HPP_
#include "cuda_runtime_api.h"
#include "kernel.h"

pluginStatus_t nmsInference(
  cudaStream_t stream, const int N, const int perBatchBoxesSize, const int perBatchScoresSize,
  const bool shareLocation, const int backgroundLabelId, const int numPredsPerClass,
  const int numClasses, const int topK, const int keepTopK, const float scoreThreshold,
  const float iouThreshold, const DataType DT_BBOX, const void * locData, const DataType DT_SCORE,
  const void * confData, void * nmsedDets, void * nmsedLabels, void * nmsedIndex, void * workspace,
  bool isNormalized, bool confSigmoid, bool clipBoxes, bool rotated = false);

#endif  // TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__BATCHED_NMS_KERNEL_HPP_
