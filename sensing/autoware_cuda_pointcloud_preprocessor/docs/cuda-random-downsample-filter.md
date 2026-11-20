# cuda_random_downsample_filter

## Purpose

This node is a CUDA accelerated version of the `RandomDownsampleFilter` available in [autoware_pointcloud_preprocessor](../../autoware_pointcloud_preprocessor/README.md).

It bounds the number of points handed to scan matching, which is what makes NDT's per-frame cost predictable. It exists as a standalone node so that a preprocessing chain can stay GPU-resident: with a CUDA crop box in front of it and a CUDA consumer behind it, cropping and downsampling happen without a round trip through host memory.

## Inner-workings / Algorithms

The selection is "random key, full sort, take the prefix":

1. A kernel gives every point a pseudo-random 32-bit key and writes its own index alongside. The key is a counter-based mixer — the finaliser from Murmur3's avalanche family — evaluated on `(seed, index)`. Counter-based rather than stateful, because a per-point `curand` state would have to be allocated, initialised and carried across frames to stay reproducible, whereas a pure function of the index and the seed needs none of that.
2. `thrust::sort_by_key` sorts the index array by those keys.
3. The first `sample_num` indices of the sorted array are the selection.
4. Those indices are sorted ascending again — a sort over `sample_num` elements, not over all `N` — and a gather kernel copies the selected points, whole, into consecutive output slots.

Properties that follow from that shape, and are the reason for it:

- **The count is exact.** Exactly `min(N, sample_num)` points come out. The obvious cheaper alternative is thresholding: keep point `i` when its key is below `sample_num / N * UINT32_MAX`, which is one kernel plus a scan and no sort at all. But its kept count is binomial around `sample_num` rather than equal to it, so a frame can land over budget and a nearly-empty frame can come out empty. The CPU `RandomDownsampleFilterComponent` promises **at most** `sample_num`, and the budgets written against that promise — NDT's per-frame cost, the memory a downstream stage sizes — are what the sort buys. Running once per scan over a few hundred thousand points, the sort is not the bottleneck.
- **The output preserves input order**, because of the second sort in step 4. Nothing in NDT requires it, but a cloud whose points still run in scan order stays readable in RViz and diffable against its input.
- **The point layout passes through untouched.** Selection copies whole points of `point_step` bytes, so intensity, ring, timestamp and any vendor field survive without this node needing to know they exist. Unlike the crop box, no coordinate is ever read — the fields are counted, never interpreted — so there is no layout this node can fail to understand and no rejection path.
- **`num_points <= sample_num` short-circuits.** Nothing has to be chosen, so the sort is skipped entirely and the input is copied device-to-device. That is not only cheaper: it makes the pass-through case bit-exact and order-preserving, matching what the CPU component does.

The seed is a call counter, incremented once per processed cloud. A given sequence of calls therefore produces a given sequence of selections, which is what makes a bag replay — and a failing test with it — reproducible. It is not a cryptographic or statistically audited stream and does not need to be.

## Inputs / Outputs

### Input

| Name                      | Type                                             | Description                               |
| ------------------------- | ------------------------------------------------ | ----------------------------------------- |
| `~/input/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Input pointcloud's topic.                 |
| `~/input/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Input pointcloud's type negotiation topic |

### Output

| Name                       | Type                                             | Description                                |
| -------------------------- | ------------------------------------------------ | ------------------------------------------ |
| `~/output/pointcloud`      | `sensor_msgs::msg::PointCloud2`                  | Downsampled pointcloud's topic             |
| `~/output/pointcloud/cuda` | `negotiated_interfaces/msg/NegotiatedTopicsInfo` | Downsampled pointcloud's negotiation topic |

## Parameters

### Core Parameters

{{ json_to_markdown("sensing/autoware_cuda_pointcloud_preprocessor/schema/cuda_random_downsample_filter.schema.json") }}

## Assumptions / Known limits

- **`sample_num` has no default.** A point budget is a tuning decision per map and per LiDAR, and a wrong one degrades localization quietly rather than failing, so the node refuses to start rather than invent one. A non-positive value is rejected at construction: it would publish empty clouds and stall scan matching with no error anywhere downstream.
- **Keys collide.** 32-bit keys tie by the birthday bound well before the ~10^5 points a 32-beam scan carries. Ties are broken arbitrarily by the sort, which perturbs uniformity slightly among tied points. It cannot perturb the count, which stays exactly `sample_num`.
- **An empty cloud is republished, not dropped.** Downstream timeout diagnostics can only tell "the LiDAR stopped" from "the scan was empty" if the empty scan still arrives.
- The selection is uniform over points, not over space: a dense region contributes proportionally more survivors than a sparse one. Where an even spatial distribution matters, use the voxel grid downsample filter instead.
