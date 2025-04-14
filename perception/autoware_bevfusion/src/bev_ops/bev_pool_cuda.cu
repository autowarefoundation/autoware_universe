// Copyright (c) OpenMMLab. All rights reserved.
// Modified from
// https://github.com/open-mmlab/mmdetection3d/blob/main/projects/BEVFusion/bevfusion/ops/bev_pool/src/bev_pool_cuda.cu
// https://github.com/mit-han-lab/bevfusion/blob/main/mmdet3d/ops/bev_pool/src/bev_pool_cuda.cu
// Available under Apache-2.0 license

/*
  Function: pillar pooling
  Args:
    b                : batch size
    d                : depth of the feature map
    h                : height of pooled feature map
    w                : width of pooled feature map
    n                : number of input points
    c                : number of channels
    n_intervals      : number of unique points
    x                : input features, FloatTensor[n, c]
    geom_feats       : input coordinates, IntTensor[n, 4]
    interval_lengths : starting position for pooled point, IntTensor[n_intervals]
    interval_starts  : how many points in each pooled point, IntTensor[n_intervals]
    out              : output features, FloatTensor[b, d, h, w, c]
*/
__global__ void bev_pool_kernel(
  int d, int h, int w, int c, int n_intervals, const float * __restrict__ x,
  const int * __restrict__ geom_feats, const int * __restrict__ interval_starts,
  const int * __restrict__ interval_lengths, float * __restrict__ out)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int index = idx / c;
  int cur_c = idx % c;
  if (index >= n_intervals) return;
  int interval_start = interval_starts[index];
  int interval_length = interval_lengths[index];

  const int * cur_geom_feats = geom_feats + interval_start * 4;
  const float * cur_x = x + interval_start * c + cur_c;
  float * cur_out = out + cur_geom_feats[3] * d * h * w * c + cur_geom_feats[2] * h * w * c +
                    cur_geom_feats[0] * w * c + cur_geom_feats[1] * c + cur_c;
  float psum = 0;
  for (int i = 0; i < interval_length; i++) {
    psum += cur_x[i * c];
  }

  *cur_out = psum;
}

void bev_pool(
  int d, int h, int w, int c, int n_intervals, const float * x, const int * geom_feats,
  const int * interval_starts, const int * interval_lengths, float * out, cudaStream_t stream)
{
  constexpr int block_size = 256;
  const int num_blocks = (n_intervals * c + block_size - 1) / block_size;

  if (num_blocks == 0) {
    return;
  }

  bev_pool_kernel<<<num_blocks, block_size, 0, stream>>>(
    d, h, w, c, n_intervals, x, geom_feats, interval_starts, interval_lengths, out);
}
