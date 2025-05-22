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

### Scatter operations

We provide a wrapper for the `segment_csr` operation presented in [torch_scatter](https://github.com/rusty1s/pytorch_scatter/tree/master). Please refer to the original code for specific details.

## Licenses

### Scatter

The codes under the `src/scatter_ops` and `include/autoware/scatter_ops` directory are copied and modified from [the original implementation](https://github.com/wy17646051/tensorrt_scatter/tree/master) and [TensorRT adaptation](https://github.com/rusty1s/pytorch_scatter/tree/master).
The original codes belong to the MIT license (original implementation) and Apache License 2.0, whereas further modifications are provided under Apache License 2.0.

> MIT License
> Copyright (c) 2020 Matthias Fey <matthias.fey@tu-dortmund.de>
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of this software and associated documentation files (the "Software"), to deal
> in the Software without restriction, including without limitation the rights
> to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Software, and to permit persons to whom the Software is
> furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
> IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
> THE SOFTWARE.
