# ring_outlier_filter

## Purpose

The purpose is to remove point cloud noise such as insects and rain.

## Inner-workings / Algorithms

A method of operating scan in chronological order and removing noise based on the rate of change in the distance between points

![ring_outlier_filter](./image/outlier_filter-ring.drawio.svg)

Another feature of this node is that it calculates visibility score based on outlier pointcloud and publish score as a topic.

### visibility score calculation algorithm

The pointcloud is divided into vertical bins (rings) and horizontal bins (azimuth divisions).
The algorithm starts by splitting the input point cloud into separate rings based on the ring value of each point. Then, for each ring, it iterates through the points and calculates the frequency of points within each horizontal bin. The frequency is determined by incrementing a counter for the corresponding bin based on the point's azimuth value.
The frequency values are stored in a frequency image matrix, where each cell represents a specific ring and azimuth bin. After calculating the frequency image, the algorithm applies a noise threshold to create a binary image. Points with frequency values above the noise threshold are considered valid, while points below the threshold are considered noise.
Finally, the algorithm calculates the visibility score by counting the number of non-zero pixels in the frequency image and dividing it by the total number of pixels (vertical bins multiplied by horizontal bins).

```plantuml
@startuml
start

:Convert input point cloud to PCL format;

:Initialize vertical and horizontal bins;

:Split point cloud into rings;

while (For each ring) is (not empty)
 :Calculate frequency of points in each azimuth bin;
 :Update frequency image matrix;
endwhile

:Apply noise threshold to create binary image;

:Count non-zero pixels in frequency image;

:Calculate visibility score as complement of filled pixel ratio;

stop
@enduml
```

## Inputs / Outputs

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherits `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/ring_outlier_filter_node.schema.json") }} |

## Assumptions / Known limits

This nodes requires that the points of the input point cloud are in chronological order and that individual points follow the memory layout specified by [PointXYZIRCAEDT](https://github.com/autowarefoundation/autoware_core/blob/main/common/autoware_point_types/include/autoware/point_types/types.hpp#L95-L116).

Two consecutive points of a ring are compared by the size of their azimuth step, so the azimuth may either increase or decrease along a ring (for example `atan2(y, x)` of a clockwise-scanning sensor), and it may wrap around anywhere, so both `[0, 2pi)` and `[-pi, pi)` are accepted. Points must still be in scan order within a ring: the filter does not sort them. It checks the order on every 8th step of each ring. When more than 5 % of those steps go against the majority direction, the ring is reported as not in scan order: the filter still runs, but it logs a throttled warning and reports a `WARN` diagnostic, because valid points of such a ring may be removed. Rings with fewer than 10 checked steps (about 80 points) are not judged.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
