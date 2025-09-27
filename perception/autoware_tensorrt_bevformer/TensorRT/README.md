# TensorRT Plugins <!-- cspell:ignore DTRT, NHMA, FHMA, QKVTRT, FMHA, NMHA -->

## Plugins

### Grid Sampler

|      OP Name      |                               Attributes                               |        Inputs         |  Outputs  | FP32 Speed | FP16 Speed | INT8 Speed | Half Type |     Tensor Format     | Test Device |
| :---------------: | :--------------------------------------------------------------------: | :-------------------: | :-------: | :--------: | :--------: | :--------: | :-------: | :-------------------: | :---------: |
| GridSampler2DTRT  | interpolation_mode: int<br />padding_mode: int<br />align_corners: int | input: T<br />grid: T | output: T |     x1     |    x2.0    |    x3.8    |  nv_half  |    kLinear, kCHW4     | RTX 2080Ti  |
| GridSampler2DTRT2 | interpolation_mode: int<br />padding_mode: int<br />align_corners: int | input: T<br />grid: T | output: T |     x1     |    x3.1    |    x3.8    | nv_half2  | kLinear, kCHW2, kCHW4 | RTX 2080Ti  |
| GridSampler3DTRT  | interpolation_mode: int<br />padding_mode: int<br />align_corners: int | input: T<br />grid: T | output: T |     x1     |    x1.3    |     -      |  nv_half  |        kLinear        | RTX 2080Ti  |
| GridSampler3DTRT2 | interpolation_mode: int<br />padding_mode: int<br />align_corners: int | input: T<br />grid: T | output: T |     x1     |    x2.2    |     -      | nv_half2  |        kLinear        | RTX 2080Ti  |

#### Inputs

- input: T[float/half/half2/int8]

  Tensor shape: `[N, C, H_in, W_in]` (4D case) or `[N, C, D_in, H_in, W_in]` (5D case)

- grid: T[float/half/half2/int8]

  Tensor shape: `[N, 2, H_out, W_out]` (4D case) or `[N, 3, D_out, H_out, W_out]` (5D case)

  `grid` specifies the sampling pixel locations normalized by the `input` spatial dimensions. Therefore, it should have most values in the range of `[-10, 10]`. For example, values `x = -10, y = -10` is the left-top pixel of `input`, and values `x = 10, y = 10` is the right-bottom pixel of `input`.

#### Attributes

- interpolation_mode: int

  Interpolation mode to calculate output values. (0: `bilinear` , 1: `nearest`, 2: `bicubic`)

  Note: `bicubic` supports only 4-D input.

- padding_mode: int

  Padding mode for outside grid values. (0: `zeros`, 1: `border`, 2: `reflection`)

- align_corners: int

  If `align_corners=1`, the extrema (`-1` and `1`) are considered as referring to the center points of the input's corner pixels. If `align_corners=0`, they are instead considered as referring to the corner points of the input's corner pixels, making the sampling more resolution agnostic.

#### Outputs

- output: T[float/half/half2/int8]

  Tensor shape: `[N, C, H_out, W_out]` (4D case) or `[N, C, D_out, H_out, W_out]` (5D case)

### Multi-scale Deformable Attention

|           OP Name            | Attributes |                                                           Inputs                                                           |  Outputs  | FP32 Speed | FP16 Speed | INT8/FP16 Speed | Half Type | Tensor Format | Test Device |
| :--------------------------: | :--------: | :------------------------------------------------------------------------------------------------------------------------: | :-------: | :--------: | :--------: | :-------------: | :-------: | :-----------: | :---------: |
| MultiScaleDeformableAttnTRT  |     -      |                 value: T<br />value_spatial_shapes: T<br />sampling_locations: T<br />attention_weights: T                 | output: T |     x1     |    x1.3    |      x3.2       |  nv_half  |    kLinear    | RTX 2080Ti  |
| MultiScaleDeformableAttnTRT2 |     -      | value: T<br />value_spatial_shapes: T<br />value_level_start_index: T<br />sampling_locations: T<br />attention_weights: T | output: T |     x1     |    x2.0    |      x2.7       | nv_half2  |    kLinear    | RTX 2080Ti  |

#### Inputs

- value: T[float/half/half2/int8]

  Tensor shape: `[N, num_keys, mum_heads, channel]`

- value_spatial_shapes: T[int32]

  Spatial shape of each feature map, has shape `[num_levels, 2]`, last dimension 2 represent (h, w)

- reference_points: T[float/half2]

  The reference points.

  Tensor shape: `[N, num_queries, 1, points_per_group * 2]`

- sampling_offsets: T[float/half/half2/int8]

  The offset of sampling points.

  Tensor shape: `[N, num_queries, num_heads, num_levels * num_points * 2]`

- attention_weights: T[float/half/int8]

  The weight of sampling points used when calculate the attention (before softmax), has shape`[N ,num_queries, num_heads, num_levels * num_points]`.

#### Attributes

​ -

#### Outputs

- output: T[float/half/int8]

  Tensor shape: `[N, num_queries, mum_heads, channel]`

### Modulated Deformable Conv2d

|            OP Name            |                                             Attributes                                             |                                   Inputs                                    |  Outputs  | FP32 Speed | FP16 Speed | INT8/FP16 Speed | Half Type |     Tensor Format     | Test Device |
| :---------------------------: | :------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------: | :-------: | :--------: | :--------: | :-------------: | :-------: | :-------------------: | :---------: |
| ModulatedDeformableConv2dTRT  | stride: int[2]<br />padding: int[2]<br />dilation: int[2]<br />groups: int<br />deform_groups: int | input: T<br />offset: T<br />mask: T<br />weight: T<br />bias: T (optional) | output: T |     x1     |    x2.9    |      x3.7       |  nv_half  |    kLinear, kCHW4     | RTX 2080Ti  |
| ModulatedDeformableConv2dTRT2 | stride: int[2]<br />padding: int[2]<br />dilation: int[2]<br />groups: int<br />deform_groups: int | input: T<br />offset: T<br />mask: T<br />weight: T<br />bias: T (optional) | output: T |     x1     |    x3.5    |      x3.7       | nv_half2  | kLinear, kCHW2, kCHW4 | RTX 2080Ti  |

#### Inputs

- input: T[float/half/half2/int8]

  Tensor shape: `[N, C_in, H_in, W_in]`

- offset: T[float/half/half2/int8]

  Tensor shape: `[N, deform_groups*K_h*K_w*2, H_out, W_out]`

- mask: T[float/half/half2/int8]

  Tensor shape: `[N, deform_groups*K_h*K_w, H_out, W_out]`

- weight: T[float/half/half2/int8]

  Tensor shape: `[C_out, C_in/groups, K_h, K_w]`

- bias: T[float/half/half2] (optional)

  Tensor shape: `[C_out]`

#### Attributes

- stride: int[2]

  Same as torch.nn.Conv2d.

- padding: int[2]

  Same as torch.nn.Conv2d.

- dilation: int[2]

  Same as torch.nn.Conv2d.

- groups: int

  Same as torch.nn.Conv2d.

- deform_groups: int

  Deformable conv2d groups.

#### Outputs

- output: T[float/half/half2/int8]

  Tensor shape: `[N, C_out, H_out, W_out]`

**NOTE: Values (C_in / groups) and (C_in / deform_groups) should be even numbers.**

### Rotate

|  OP Name   |     Attributes     |               Inputs                |  Outputs  | FP32 Speed | FP16 Speed | INT8/FP16 Speed | Half Type |     Tensor Format     | Test Device |
| :--------: | :----------------: | :---------------------------------: | :-------: | :--------: | :--------: | :-------------: | :-------: | :-------------------: | :---------: |
| RotateTRT  | interpolation: int | img: T<br />angle: T<br />center: T | output: T |     x1     |    X1.8    |      X4.4       |  nv_half  |    kLinear, kCHW4     | RTX 2080Ti  |
| RotateTRT2 | interpolation: int | img: T<br />angle: T<br />center: T | output: T |     x1     |    x2.2    |      x4.4       | nv_half2  | kLinear, kCHW2, kCHW4 | RTX 2080Ti  |

#### Inputs

- img: T[float/half/half2/int8]

  Tensor shape: `[C, H, W]`

- angle: T[float/half/half2]

  Tensor shape: `[1]`

- center: T[float/half/half2]

  Tensor shape: `[2]`

#### Attributes

- interpolation: int

  Interpolation mode to calculate output values. (0: `bilinear` , 1: `nearest`)

#### Outputs

- output: T[float/half/half2/int8]

  Tensor shape: `[C, H, W]`

### Inverse

|  OP Name   | Attributes |     Inputs      |     Outputs      | Tensor Format | Test Device |
| :--------: | :--------: | :-------------: | :--------------: | :-----------: | :---------: |
| InverseTRT |     -      | input: T[float] | output: T[float] |    kLinear    | RTX 2080Ti  |

#### Inputs

- input: T[float]

  Tensor shape: `[B, C, H, W]`

#### Outputs

- output: T[float]

  Tensor shape: `[B, C, H, W]`

### BEV Pool

|    OP Name    |             Attributes              |                                                              Inputs                                                               |  Outputs  | FP32 Speed | FP16 Speed | INT8 Speed | Half Type | Tensor Format | Test Device |
| :-----------: | :---------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------: | :-------: | :--------: | :--------: | :--------: | :-------: | :-----------: | :---------: |
| BEVPoolV2TRT  | out_height: int<br />out_width: int | depth: T<br />feat: T<br />ranks_depth: T<br />ranks_feat: T<br /> ranks_bev: T<br /> interval_starts: T<br />interval_lengths: T | output: T |     x1     |    X1.1    |    X2.1    |  nv_half  |    kLinear    | RTX 2080Ti  |
| BEVPoolV2TRT2 | out_height: int<br />out_width: int | depth: T<br />feat: T<br />ranks_depth: T<br />ranks_feat: T<br /> ranks_bev: T<br /> interval_starts: T<br />interval_lengths: T | output: T |     x1     |    x1.4    |    X2.1    | nv_half2  |    kLinear    | RTX 2080Ti  |

#### Inputs

- depth: T[float/half/half2/int8]

  Tensor shape: `[Cam, D, H, W]`

- feat: T[float/half/half2/int8]

  Tensor shape: `[Cam, H, W, C]`

- ranks_depth: T[int32]

- ranks_feat: T[int32]

- ranks_bev: T[int32]

- interval_starts: T[int32]

- interval_lengths: T[int32]

#### Attributes

- out_height: int

  BEV feature height

- out_width: int

  BEV feature width

#### Outputs

- output: T[float/half/half2/int8]

  Tensor shape: `[1, out_height, out_width, C]`

### Multi-Head Attention

| OP Name |               Inputs               |  Outputs  | FP32 Speed NHMA | FP16 Speed NHMA | FP32 Speed FHMA | FP16 Speed FHMA | INT8 Speed FHMA | Half Type | Test Device |
| :-----: | :--------------------------------: | :-------: | :-------------: | :-------------: | :-------------: | :-------------: | :-------------: | :-------: | :---------: |
| QKVTRT  | query: T<br />key: T<br />value: T | output: T |       x1        |      X2.0       |      x4.6       |      x6.1       |      x8.2       |  nv_half  | RTX 2080Ti  |
| QKVTRT2 | query: T<br />key: T<br />value: T | output: T |       x1        |      X2.1       |      x4.6       |      x6.3       |      x8.2       | nv_half2  | RTX 2080Ti  |

#### Inputs

- query: T[float/half/half2/int8]

  Tensor shape: `[batch, q_len, channel]`

- key: T[float/half/half2/int8]

  Tensor shape: `[batch, kv_len, channel]`

- value: T[float/half/half2/int8]

  Tensor shape: `[batch, kv_len, channel]`

#### Attributes

​ -

#### Outputs

- output: T[float/half/half2/int8]

  Tensor shape: `[batch, q_len, channel]`

**NOTE: If `q_len` and `kv_len` are both multiples of 64, the plugin will run with Flash Multi-Head Attention (FMHA), else Naive Multi-Head Attention (NMHA).**
