# Planning Validator

The `autoware_planning_validator` node is the last module executed in the planning component, it responsible for checking the validity of the planning trajectory before it is published to control component. The status of the validation can be viewed in the `/diagnostics` and `/validation_status` topics. When an invalidity is detected, the `autoware_planning_validator` will process the trajectory following the selected option: "0. publish the trajectory as it is", "1. stop publishing the trajectory", "2. publish the last validated trajectory".

The `autoware_planning_validator` node loads multiple plugins modules, each responsible for running specific validation checks on the planning trajectory:

- **Latency Checker**: The `autoware_planning_validator_latency_checker` is responsible for checking the validity of planning trajectory age
- **Trajectory Checker**: The `autoware_planning_validator_trajectory_checker` is responsible for checking the validity of planning trajectory shape
- **Intersection Collision Checker**: The `autoware_planning_validator_intersection_collision_checker` is responsible for verifying planning trajectory does not result in collision at intersections
- **Rear Collision Checker**: The `autoware_planning_validator_rear_collision_checker` is responsible for verifying planning trajectory does not result in collision with rear vehicles

![autoware_planning_validator](./image/planning_validator.drawio.svg)

## Inputs/Outputs

### Inputs

The `autoware_planning_validator` takes in the following inputs:

| Name                      | Type                                     | Description                                    |
| ------------------------- | ---------------------------------------- | ---------------------------------------------- |
| `~/input/kinematics`      | nav_msgs/Odometry                        | ego pose and twist                             |
| `~/input/acceleration`    | geometry_msgs/AccelWithCovarianceStamped | current acceleration of the ego vehicle        |
| `~/input/trajectory`      | autoware_planning_msgs/Trajectory        | target trajectory to be validated in this node |
| `~/input/route`           | autoware_planning_msgs/LaneletRoute      | route information                              |
| `~/input/lanelet_map_bin` | autoware_map_msgs/LaneletMapBin          | lanelet vector map information                 |
| `~/input/pointcloud`      | sensor_msgs/PointCloud2                  | obstacle pointcloud with ground removed        |

### Outputs

It outputs the following:

| Name                         | Type                                       | Description                                                               |
| ---------------------------- | ------------------------------------------ | ------------------------------------------------------------------------- |
| `~/output/trajectory`        | autoware_planning_msgs/Trajectory          | validated trajectory                                                      |
| `~/output/validation_status` | planning_validator/PlanningValidatorStatus | validator status to inform the reason why the trajectory is valid/invalid |
| `/diagnostics`               | diagnostic_msgs/DiagnosticStatus           | diagnostics to report errors                                              |

## Parameters

The following parameters can be set for the `autoware_planning_validator`:

| Name                         | Type   | Description                                                                                                                                                                                      | Default value |
| :--------------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `default_handling_type`      | int    | set default handling type for when invalidity is detected. <br>0: publish invalid traj as it is, <br>1: publish last valid traj, <br>2: publish last valid traj with soft stop                   | 0             |
| `publish_diag`               | bool   | if true, diagnostics msg is published.                                                                                                                                                           | true          |
| `diag_error_count_threshold` | int    | Number of consecutive invalid trajectories to set the Diag to ERROR. (Fe.g, threshold = 1 means, even if the trajectory is invalid, the Diag will not be ERROR if the next trajectory is valid.) | 0             |
| `display_on_terminal`        | bool   | show error msg on terminal                                                                                                                                                                       | true          |
| `soft_stop_deceleration`     | double | deceleration value to be used for soft stop action. [m/ss]                                                                                                                                       | -1.0          |
| `soft_stop_jerk_lim`         | double | jerk limit value to be used for soft stop action. [m/sss]                                                                                                                                        | 0.3           |
