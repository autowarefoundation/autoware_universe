# Map annotation guide

This document describes how to annotate a Lanelet2 map for `autoware_environment_adaptor`.

## Overview

The node classifies the driving environment by checking whether the vehicle position is inside Lanelet2 polygons tagged as `degenerate_area`.
These polygons mark areas where geometric features are insufficient to constrain at least one translational or rotational degree of freedom in NDT scan matching or lane line matching (i.e., degenerate/underconstrained regions).
Each polygon's `subtype` is mapped to an `environment_id` via ROS parameters.

## Polygon requirements

| Attribute                   | Required | Description                                                                                                                    |
| --------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `type`                      | Yes      | Must be `degenerate_area` (degenerate localization area)                                                                       |
| `subtype`                   | Yes      | Identifier mapped to `environment_id` in the parameter file (e.g. `uniform_road`, `feature_poor_road`)                         |
| `longitudinal_scale_factor` | No       | Per-polygon override for twist velocity scaling. Attribute name is configurable via `map_longitudinal_scale_factor_attribute`. |

Polygons outside the map or with incorrect `type` are ignored.

## OSM example

The following example defines a uniform-road area with a per-polygon longitudinal scale factor:

```xml
<relation id='-10001' action='modify'>
  <tag k='type' v='degenerate_area'/>
  <tag k='subtype' v='uniform_road'/>
  <tag k='longitudinal_scale_factor' v='1.0075'/>
  <member type='way' ref='-10002' role='outer'/>
</relation>
```

Corresponding parameter mapping in `environment_adaptor.param.yaml`:

```yaml
area_subtype_uniform_road:
  environment_id: 1
environment_1_output_pose_covariance: [...]
```

When the vehicle is inside this polygon:

- `environment_id` becomes `1` (from `subtype` → parameter mapping).
- `linear.x` is multiplied by `1.0075` (`v_out = v_in × longitudinal_scale_factor`). The scale factor comes from the map attribute. If the polygon has no such attribute, `default_longitudinal_scale_factor` is applied instead.

## Parameter naming convention

### Area subtype mapping

```yaml
area_subtype_<subtype_name>:
  environment_id: <int>
```

Example:

```yaml
area_subtype_uniform_road:
  environment_id: 1
area_subtype_feature_poor_road:
  environment_id: 2
```

The `<subtype_name>` must match the `subtype` attribute on the Lanelet2 polygon.

### Per-environment pose covariance

```yaml
environment_<id>_output_pose_covariance: [36 elements, 6x6 row-major]
```

When no `environment_<id>_output_pose_covariance` is defined for the classified environment ID, the input pose covariance is passed through unchanged.

Covariance is defined in the **body frame** (x: forward, y: left, z: up).
The node rotates the position block (top-left 3×3) into the map frame using the pose orientation.

Matrix layout (row-major, indices 0–35):

```
 Index:  0  1  2  |  3  4  5
         6  7  8  |  9 10 11
        12 13 14  | 15 16 17
        ----------+-----------
        18 19 20  | 21 22 23
        24 25 26  | 27 28 29
        30 31 32  | 33 34 35

 Rows/cols 0–2: position (x, y, z)
 Rows/cols 3–5: orientation (roll, pitch, yaw)
```

### Longitudinal scale factor

```yaml
default_longitudinal_scale_factor: <double>
map_longitudinal_scale_factor_attribute: longitudinal_scale_factor
```

The scale factor is defined per polygon via the `longitudinal_scale_factor` map attribute.
When a polygon has no such attribute, `default_longitudinal_scale_factor` is applied.

Applied as: `v_out = v_in * longitudinal_scale_factor` on `twist.twist.linear.x`.

## Workflow

1. Identify road sections where localization is degenerate (tunnels, feature-poor areas, etc.).
2. Draw `degenerate_area` polygons in your map editor (e.g. JOSM with Lanelet2 plugin).
3. Set `subtype` on each polygon.
4. Optionally set `longitudinal_scale_factor` on individual polygons.
5. Add matching `area_subtype_*` and `environment_*` entries in the parameter file.
6. Tune covariance and scale factor values on a test vehicle or in simulation.

## Tips

- Keep polygons slightly larger than the physical road section to avoid flickering at boundaries.
- Use debug topics `~/debug/environment_id` and `~/debug/longitudinal_scale_factor` to verify classification in RViz or `ros2 topic echo`.
- Start with conservative covariance increases and small scale factors, then tune based on EKF behavior.
