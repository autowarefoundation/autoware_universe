# crop_box_filter

## Purpose

The `crop_box_filter` is a node that removes points with in a given box region. This filter is used to remove the points that hit the vehicle itself.
Further, it can filter out reflections caused by reflective surfaces within the box, using a ray intersection check.

## Inner-workings / Algorithms

An axis-aligned bounding box in the `input_frame` is specified using the below parameters.
The pointcloud input in `~/input/points` is transformed to `input_frame` and then filtered by the bounding box.

Depending on the `negative` parameter, the filter will either keep the points inside the box or outside the box:

| `negative` | Description                          |
| ---------- | ------------------------------------ |
| `false`    | Keep only the points inside the box  |
| `true`     | Keep only the points outside the box |

If `use_ray_intersection` is set to `true`, the filter will extend to filter any points whose LiDAR
ray intersected the bounding box. This is useful in the following case:

![crop_box-ray-usecase.svg](./image/crop_box-ray-usecase.svg)

Consider a reflective surface within the crop box, e.g. a mirror of the ego vehicle. When a LiDAR
ray hits the mirror, it reflects off at an angle and can hit another object in the scene. Because
the LiDAR only knows the angle it fired the ray at, it cannot distinguish between reflections and
non-reflections, and the object shows up behind the reflector - as a so-called "ghost object".

!!! note
This is the same for cameras - or humans for that matter. While dealing with reflections can in
theory be learnt by a perception model, not all modules can handle LiDAR reflections at this time.

To filter out these ghost objects, the ray intersection check ensures that any points behind the
crop box are filtered out. By definition, a point behind the crop box has to come from a ray that\
passed through it - which should not be possible if the crop box is tight around the ego vehicle.

!!! warning
The ray intersection check only makes sense on point clouds that come from a **single** LiDAR sensor
with the point cloud frame being at the origin of the LiDAR sensor.
If you are using a multi-LiDAR system, or a transformed point cloud,
the ray intersection check will not work as expected.

The above `negative` parameter can be used in conjunction with `use_ray_intersection`:

| `negative` | `use_ray_intersection` | Description                                                                                      |
| ---------- | ---------------------- | ------------------------------------------------------------------------------------------------ |
| `false`    | `true`                 | Keep only the points inside the box and any points whose LiDAR ray intersected the bounding box  |
| `true`     | `true`                 | Keep only the points outside the box and any points whose LiDAR ray intersected the bounding box |

Or, visually:

| `negative` \ `use_ray_intersection` | `false`                                                 | `true`                                                          |
| ----------------------------------- | ------------------------------------------------------- | --------------------------------------------------------------- |
| `false`                             | ![crop_box-positive.svg](./image/crop_box-positive.svg) | ![crop_box-ray-positive.svg](./image/crop_box-ray-positive.svg) |
| `true`                              | ![crop_box-negative.svg](./image/crop_box-negative.svg) | ![crop_box-ray-negative.svg](./image/crop_box-ray-negative.svg) |

## Inputs / Outputs

This implementation inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

## Parameters

### Node Parameters

This implementation inherit `autoware::pointcloud_preprocessor::Filter` class, please refer [README](../README.md).

### Core Parameters

{{ json_to_markdown("sensing/autoware_pointcloud_preprocessor/schema/crop_box_filter_node.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
