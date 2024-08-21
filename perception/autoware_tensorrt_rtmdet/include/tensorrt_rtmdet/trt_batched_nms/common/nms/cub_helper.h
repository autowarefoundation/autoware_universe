// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
// modify from
// https://github.com/NVIDIA/TensorRT/tree/master/plugin/batchedNMSPlugin
#ifndef TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__CUB_HELPER_H_
#define TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__CUB_HELPER_H_

#include "kernel.h"
template <typename KeyT, typename ValueT>
size_t cubSortPairsWorkspaceSize(int num_items, int num_segments)
{
  size_t temp_storage_bytes = 0;
  cub::DeviceSegmentedRadixSort::SortPairsDescending(
    (void *)NULL, temp_storage_bytes, (const KeyT *)NULL, (KeyT *)NULL, (const ValueT *)NULL,
    (ValueT *)NULL,
    num_items,     // # items
    num_segments,  // # segments
    (const int *)NULL, (const int *)NULL);
  return temp_storage_bytes;
}

#endif  // TENSORRT_RTMDET__TRT_BATCHED_NMS__COMMON__NMS__CUB_HELPER_H_
