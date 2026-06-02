# autoware_joy_controller

## Role

`autoware_joy_controller` is the package to convert a joy msg to autoware commands (e.g. steering wheel, shift, turn signal, engage) for a vehicle.

This package targets the AD API manual control pipeline:

`joy_controller -> external_cmd_selector -> external_cmd_converter -> vehicle_cmd_gate`

Legacy `tier4_external_api_msgs` manual command topics are not published by this package.

## Usage

### ROS 2 launch

```bash
# With default config (ds4)
ros2 launch autoware_joy_controller joy_controller.launch.xml

# Default config but select from the existing parameter files
ros2 launch autoware_joy_controller joy_controller_param_selection.launch.xml joy_type:=ds4 # or g29, p65, xbox

# Override the param file
ros2 launch autoware_joy_controller joy_controller.launch.xml config_file:=/path/to/your/param.yaml
```

## Input / Output

### Input topics

| Name               | Type                    | Description                       |
| ------------------ | ----------------------- | --------------------------------- |
| `~/input/joy`      | sensor_msgs::msg::Joy   | joy controller command            |
| `~/input/odometry` | nav_msgs::msg::Odometry | ego vehicle odometry to get twist |

### Output topics

| Name                           | Type                                                 | Description                              |
| ------------------------------ | ---------------------------------------------------- | ---------------------------------------- |
| `~/output/control_command`     | autoware_control_msgs::msg::Control                  | lateral and longitudinal control command |
| `~/output/pedals_command`      | autoware_adapi_v1_msgs::msg::PedalsCommand           | manual pedals command for AD API         |
| `~/output/steering_command`    | autoware_adapi_v1_msgs::msg::SteeringCommand         | manual steering command for AD API       |
| `~/output/gear_cmd`            | autoware_vehicle_msgs::msg::GearCommand              | gear command for AD API                  |
| `~/output/turn_indicators_cmd` | autoware_vehicle_msgs::msg::TurnIndicatorsCommand    | turn indicator command for AD API        |
| `~/output/hazard_lights_cmd`   | autoware_vehicle_msgs::msg::HazardLightsCommand      | hazard light command for AD API          |
| `~/output/gate_mode`           | tier4_control_msgs::msg::GateMode                    | gate mode (Auto or External)             |
| `~/output/operator_heartbeat`  | autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat | manual operator heartbeat for AD API     |
| `~/output/vehicle_engage`      | autoware_vehicle_msgs::msg::Engage                   | vehicle engage                           |

## Parameters

| Parameter                 | Type   | Description                                                                                                |
| ------------------------- | ------ | ---------------------------------------------------------------------------------------------------------- |
| `joy_type`                | string | joy controller type (default: DS4)                                                                         |
| `update_rate`             | double | update rate to publish control commands                                                                    |
| `accel_ratio`             | double | ratio to calculate acceleration (commanded acceleration is ratio \* operation)                             |
| `brake_ratio`             | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                            |
| `steer_ratio`             | double | ratio to calculate deceleration (commanded steer is ratio \* operation)                                    |
| `steering_angle_velocity` | double | steering angle velocity for operation                                                                      |
| `accel_sensitivity`       | double | sensitivity to calculate pedal throttle output (commanded acceleration is pow(operation, 1 / sensitivity)) |
| `brake_sensitivity`       | double | sensitivity to calculate pedal brake output (commanded acceleration is pow(operation, 1 / sensitivity))    |
| `raw_control`             | bool   | skip input odometry if true                                                                                |
| `velocity_gain`           | double | ratio to calculate velocity by acceleration                                                                |
| `max_forward_velocity`    | double | absolute max velocity to go forward                                                                        |
| `max_backward_velocity`   | double | absolute max velocity to go backward                                                                       |
| `backward_accel_ratio`    | double | ratio to calculate deceleration (commanded acceleration is -ratio \* operation)                            |

## P65 Joystick Key Map

| Action               | Button                |
| -------------------- | --------------------- |
| Acceleration         | R2                    |
| Brake                | L2                    |
| Steering             | Left Stick Left Right |
| Shift up             | Cursor Up             |
| Shift down           | Cursor Down           |
| Shift Drive          | Cursor Left           |
| Shift Reverse        | Cursor Right          |
| Turn Signal Left     | L1                    |
| Turn Signal Right    | R1                    |
| Hazard Lights        | L1 + R1               |
| Clear Turn Signal    | A                     |
| Gate Mode            | B                     |
| Emergency Stop       | Select                |
| Clear Emergency Stop | Start                 |
| Autoware Engage      | X                     |
| Autoware Disengage   | Y                     |
| Vehicle Engage       | PS                    |
| Vehicle Disengage    | Right Trigger         |

## DS4 Joystick Key Map

| Action               | Button                     |
| -------------------- | -------------------------- |
| Acceleration         | R2, ×, or Right Stick Up   |
| Brake                | L2, □, or Right Stick Down |
| Steering             | Left Stick Left Right      |
| Shift up             | Cursor Up                  |
| Shift down           | Cursor Down                |
| Shift Drive          | Cursor Left                |
| Shift Reverse        | Cursor Right               |
| Turn Signal Left     | L1                         |
| Turn Signal Right    | R1                         |
| Hazard Lights        | L1 + R1                    |
| Clear Turn Signal    | SHARE                      |
| Gate Mode            | OPTIONS                    |
| Emergency Stop       | PS                         |
| Clear Emergency Stop | PS                         |
| Autoware Engage      | ○                          |
| Autoware Disengage   | ○                          |
| Vehicle Engage       | △                          |
| Vehicle Disengage    | △                          |

## XBOX Joystick Key Map

| Action               | Button                |
| -------------------- | --------------------- |
| Acceleration         | RT                    |
| Brake                | LT                    |
| Steering             | Left Stick Left Right |
| Shift up             | Cursor Up             |
| Shift down           | Cursor Down           |
| Shift Drive          | Cursor Left           |
| Shift Reverse        | Cursor Right          |
| Turn Signal Left     | LB                    |
| Turn Signal Right    | RB                    |
| Hazard Lights        | LB + RB               |
| Clear Turn Signal    | A                     |
| Gate Mode            | B                     |
| Emergency Stop       | View                  |
| Clear Emergency Stop | Menu                  |
| Autoware Engage      | X                     |
| Autoware Disengage   | Y                     |
| Vehicle Engage       | Left Stick Button     |
| Vehicle Disengage    | Right Stick Button    |
