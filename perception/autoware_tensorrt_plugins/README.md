# autoware_tensorrt_plugins

## Purpose

The `autoware_tensorrt_plugins` extends the operations available in TensorRT via plugins

## Algorithms

### Sparse convolutions

This package implements a TensorRT powered inference node for BEVFusion [1].
The sparse convolution backend corresponds to [spconv](https://github.com/traveller59/spconv).
Autoware installs it automatically in its setup script. If needed, the user can also build it and install it following the [following instructions](https://github.com/autowarefoundation/spconv_cpp).
