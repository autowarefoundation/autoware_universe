# Road User Stop

## Role

The `road_user_stop` module stops the ego vehicle when pedestrians, cyclists, or other road users are detected on or near the road within the ego's planned trajectory. This module ensures safe interaction with vulnerable road users by maintaining appropriate stopping distances and making decisions about when to stop versus when to continue driving.

## Activation

This module is activated if the launch parameter `launch_road_user_stop_module` is set to true.

## Parameters

### Option Parameters

| Name                 | Unit | Type | Description                    | Default value |
| :------------------- | :--- | :--- | :----------------------------- | :------------ |
| suppress_sudden_stop | [-]  | bool | Enable to suppress sudden stop | true          |

### Stop Planning Parameters

#### Longitudinal Margin Parameters

| Name            | Unit | Type   | Description                               | Default value |
| :-------------- | :--- | :----- | :---------------------------------------- | :------------ |
| default_margin  | [m]  | double | Default longitudinal margin to obstacle   | 5.0           |
| terminal_margin | [m]  | double | Stop margin at the goal position          | 3.0           |
| minimum_margin  | [m]  | double | Minimum stop margin for behavior decision | 3.0           |

#### Opposing Traffic Parameters

| Name                           | Unit    | Type   | Description                                                    | Default value |
| :----------------------------- | :------ | :----- | :------------------------------------------------------------- | :------------ |
| stop_margin                    | [m]     | double | Ideal stop-margin from moving opposing obstacle when ego stops | 10.0          |
| max_negative_velocity          | [m/s]   | double | Maximum velocity of opposing traffic to consider stop planning | -0.1          |
| min_velocity_for_stop_planning | [m/s]   | double | Minimum velocity of ego to consider stop planning              | 2.77          |
| effective_deceleration         | [m/s^2] | double | Higher value brings final stop-margin closer to ideal value    | 4.0           |

#### General Stop Planning Parameters

| Name                         | Unit    | Type   | Description                                                         | Default value |
| :--------------------------- | :------ | :----- | :------------------------------------------------------------------ | :------------ |
| hold_stop_velocity_threshold | [m/s]   | double | The maximum ego velocity to hold stopping                           | 0.01          |
| hold_stop_distance_threshold | [m]     | double | The ego keeps stopping if distance to stop changes within threshold | 0.3           |
| limit_min_acc                | [m/s^2] | double | Overwrite the deceleration limit (usually from common_param.yaml)   | -2.5          |
| sudden_object_acc_threshold  | [m/s^2] | double | If stop achievable by smaller deceleration, not "sudden stop"       | -1.0          |
| sudden_object_dist_threshold | [m]     | double | If stop distance is longer than this, not considered "sudden stop"  | 1000.0        |
| abandon_to_stop              | [-]     | bool   | Give up stopping when cannot avoid run over within decel limit      | false         |

#### Stop on Curve Parameters

| Name                   | Unit | Type   | Description                                    | Default value |
| :--------------------- | :--- | :----- | :--------------------------------------------- | :------------ |
| enable_approaching     | [-]  | bool   | Enable approaching behavior on curved paths    | false         |
| additional_stop_margin | [m]  | double | Additional stop margin for obstacles on curves | 3.0           |
| min_stop_margin        | [m]  | double | Minimum stop margin on curves                  | 6.0           |

### Obstacle Filtering Parameters

#### Object Type Parameters

| Name       | Unit | Type | Description                         | Default value |
| :--------- | :--- | :--- | :---------------------------------- | :------------ |
| pedestrian | [-]  | bool | Enable detection of pedestrians     | true          |
| bicycle    | [-]  | bool | Enable detection of bicycles        | true          |
| motorcycle | [-]  | bool | Enable detection of motorcycles     | false         |
| unknown    | [-]  | bool | Enable detection of unknown objects | false         |

#### General Filtering Parameters

| Name                      | Unit | Type   | Description                                          | Default value |
| :------------------------ | :--- | :----- | :--------------------------------------------------- | :------------ |
| trajectory_lateral_margin | [m]  | double | Lateral margin from ego trajectory to detect objects | 1.0           |

#### Intersection Parameters

| Name    | Unit | Type | Description                                                          | Default value |
| :------ | :--- | :--- | :------------------------------------------------------------------- | :------------ |
| exclude | [-]  | bool | If true, exclude objects inside intersection lanelets from detection | false         |

#### Crosswalk Parameters

| Name    | Unit | Type   | Description                     | Default value |
| :------ | :--- | :----- | :------------------------------ | :------------ |
| exclude | [-]  | bool   | Exclude objects near crosswalks | true          |
| margin  | [m]  | double | Margin distance from crosswalk  | 1.0           |

#### Opposing Traffic Detection Parameters

| Name                | Unit  | Type   | Description                             | Default value |
| :------------------ | :---- | :----- | :-------------------------------------- | :------------ |
| enable              | [-]   | bool   | Enable wrong-way object detection       | true          |
| angle_threshold     | [deg] | double | Angle threshold for wrong-way detection | 150.0         |
| min_speed_threshold | [m/s] | double | Minimum speed for wrong-way detection   | 0.5           |

#### Temporal Filtering Parameters

| Name                           | Unit | Type   | Description                                                                                              | Default value |
| :----------------------------- | :--- | :----- | :------------------------------------------------------------------------------------------------------- | :------------ |
| min_detection_duration         | [s]  | double | Minimum duration for object detection                                                                    | 0.1           |
| lost_object_retention_duration | [s]  | double | Duration to keep tracking objects after they are lost or exit the detection area                         | 2.0           |
| polygon_expansion_length       | [m]  | double | Distance to expand object polygon outward from centroid when object was previously inside detection area | 0.5           |

## Limitations

- If object tracking is lost for an extended period or object classification is misidentified, the module may not function properly and may fail to stop as expected
