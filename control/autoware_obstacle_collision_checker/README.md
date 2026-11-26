# obstacle_collision_checker

## Purpose

`obstacle_collision_checker` is a module to check obstacle collision for predicted trajectory and publish diagnostic errors if collision is found.

## Inner-workings / Algorithms

### Flow chart

```plantuml
@startuml
skinparam monochrome true

title obstacle collision checker : update
start

:calculate braking distance;

:resampling trajectory;
note right
to reduce calculation cost
end note
:filter point cloud by trajectory;

:create vehicle foot prints;

:create vehicle passing area;

partition will_collide {

while (has next ego vehicle foot print) is (yes)
  :found collision with obstacle foot print;
  if (has collision with obstacle) then (yes)
      :set diag to ERROR;
      stop
  endif
end while (no)
:set diag to OK;
stop
}

@enduml
```

### Algorithms

### Check data

Check that `obstacle_collision_checker` receives no ground pointcloud, predicted_trajectory, reference trajectory, and current velocity data.

### Diagnostic update

If any collision is found on predicted path, this module sets `ERROR` level as diagnostic status else sets `OK`.

## Inputs / Outputs

### Input

| Name                                           | Type                                      | Description                                                        |
| ---------------------------------------------- | ----------------------------------------- | ------------------------------------------------------------------ |
| `~/input/trajectory`                           | `autoware_planning_msgs::msg::Trajectory` | Reference trajectory                                               |
| `~/input/trajectory`                           | `autoware_planning_msgs::msg::Trajectory` | Predicted trajectory                                               |
| `/perception/obstacle_segmentation/pointcloud` | `sensor_msgs::msg::PointCloud2`           | Pointcloud of obstacles which the ego-vehicle should stop or avoid |
| `/tf`                                          | `tf2_msgs::msg::TFMessage`                | TF                                                                 |
| `/tf_static`                                   | `tf2_msgs::msg::TFMessage`                | TF static                                                          |

### Output

| Name             | Type                                   | Description              |
| ---------------- | -------------------------------------- | ------------------------ |
| `~/debug/marker` | `visualization_msgs::msg::MarkerArray` | Marker for visualization |

## Parameters

{{ json_to_markdown("control/autoware_obstacle_collision_checker/schema/obstacle_collision_checker.schema.json") }}

## Assumptions / Known limits

To perform proper collision check, it is necessary to get probably predicted trajectory and obstacle pointclouds without noise.
