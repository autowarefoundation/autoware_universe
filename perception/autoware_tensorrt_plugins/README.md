# autoware_tensorrt_plugins

## Purpose

The `autoware_tensorrt_plugins` package extends the operations available in TensorRT via plugins.

## Algorithms

The following operations are implemented:

### Sparse convolutions

We provide a wrapper for [spconv](https://github.com/traveller59/spconv) (please see the correspondent package for details about the algorithms involved).
This requires the installation of `spconv_cpp` which is automatically installed in autoware's setup script. If needed, the user can also build and install it using the repository's [instructions](https://github.com/autowarefoundation/spconv_cpp).

### Argsort

We provide an implementation for the `Argsort` operation as a plugin since the `TopK` TensorRT implementation has limitations in the size of elements it can handle.

### BEV Pool

We provide a wrapper for the `bev_pool` operation presented in [BEVFusion](https://github.com/mit-han-lab/bevfusion). Please refer to the original paper for specific details.
