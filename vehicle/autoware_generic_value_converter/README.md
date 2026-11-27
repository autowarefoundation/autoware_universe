# autoware_generic_value_converter

## Overview

`autoware_generic_value_converter` is a generic value conversion node that converts acceleration commands to arbitrary float64 output values. It uses bilinear interpolation on a calibrated velocity-acceleration mapping table, similar to `autoware_raw_vehicle_cmd_converter`, but outputs generic float64 values instead of specific pedal commands.

## Features

- **Generic**: Outputs `tier4_debug_msgs::msg::Float64Stamped` messages
- **Bilinear Interpolation**: Uses velocity and acceleration for table lookup to obtain output values
- **CSV Loading**: Loads mapping table from CSV file
- **Real-time Conversion**: Receives control commands and converts them to output values in real-time
- **Configurable Parameters**: Supports maximum and minimum value limits

## Input Topics

| Topic Name            | Message Type                          | Description                                       |
| --------------------- | ------------------------------------- | ------------------------------------------------- |
| `~/input/control_cmd` | `autoware_control_msgs::msg::Control` | Control command (containing desired acceleration) |
| `~/input/odometry`    | `nav_msgs::msg::Odometry`             | Odometry information (for current velocity)       |

## Output Topics

| Topic Name       | Message Type                            | Description                    |
| ---------------- | --------------------------------------- | ------------------------------ |
| `~/output/value` | `tier4_debug_msgs::msg::Float64Stamped` | Converted float64 output value |

## Parameters

| Parameter Name       | Type   | Default Value                                                           | Description                               |
| -------------------- | ------ | ----------------------------------------------------------------------- | ----------------------------------------- |
| `csv_path_value_map` | string | "$(find-pkg-share autoware_generic_value_converter)/data/value_map.csv" | Path to mapping CSV file                  |
| `max_value`          | double | 3.0                                                                     | Maximum output value                      |
| `min_value`          | double | -5.0                                                                    | Minimum output value                      |
| `convert_value_cmd`  | bool   | true                                                                    | Whether to perform conversion             |
| `use_value_ff`       | bool   | true                                                                    | Whether to use feedforward (lookup table) |

## Usage

### Launch Converter

```bash
ros2 launch autoware_generic_value_converter generic_value_converter.launch.xml
```

### Specify Custom Mapping

```bash
ros2 launch autoware_generic_value_converter generic_value_converter.launch.xml \
  csv_path_value_map:=/path/to/your/value_map.csv
```

## How It Works

### Table Lookup Conversion

The converter uses **"inverse bilinear interpolation** to find output values from a 2D mapping table. The map stores the relationship `(input_value, velocity) → acceleration`, but the converter needs to solve the inverse problem: given `(desired_acceleration, current_velocity) → output_value`.

The algorithm performs a two-step interpolation:

1. **Fix Velocity**: For each input value index in the map, interpolate the acceleration value at the current velocity across the velocity dimension. This creates a 1D relationship: `input_value → acceleration` at the current velocity.

2. **Fix Acceleration**: Using the interpolated acceleration values from step 1, find the input value that corresponds to the desired acceleration. This is done by interpolating across the input value dimension.

**Mathematical Formulation**:

Given a map `accel_map[value_idx][vel_idx]` representing acceleration at `(value, velocity)`:

1. For each `value_idx`, compute:

   ```c
   accel_at_vel[value_idx] = lerp(vel_index, accel_map[value_idx][:], current_velocity)
   ```

2. Find the output value:

   ```c
   output_value = lerp(accel_at_vel[:], value_index[:], desired_acceleration)
   ```

**Boundary Handling**: If the desired acceleration is outside the map range (below minimum or above maximum), the converter returns the minimum or maximum input value respectively.

### Passthrough Mode

When `convert_value_cmd` is `false` or `use_value_ff` is `false`, the converter operates in **passthrough mode**:

- The desired acceleration from the control command is directly output as the value
- No map lookup is performed
- Output is still clamped to `[min_value, max_value]` range

This mode is useful for:

- Testing without a calibrated map
- Direct acceleration command forwarding
- Debugging calibration issues

### Mapping Format

Mapping is stored in CSV format, same as calibrator output:

```csv
default,0.0,2.0,4.0,6.0,8.0,10.0,12.0,14.0,16.0,18.0,20.0
-5.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0
-4.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0
...
3.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0
```

**Format Explanation**:

- **First row**: "default" (vehicle/map name) followed by velocity indices in m/s
- **First column**: Input value indices
- **Cells**: Acceleration values in m/s² at `(input_value, velocity)`

**Note**: The default value range is `[-5.0, 3.0]` with 11 points, but you can customize this range when generating the map with the calibrator.

## Comparison with Original Converter

| Feature        | raw_vehicle_cmd_converter   | generic_value_converter  |
| -------------- | --------------------------- | ------------------------ |
| Input Type     | Acceleration command        | Acceleration command     |
| Output Type    | Throttle/Brake pedal values | Arbitrary float64 values |
| Output Message | `ActuationCommandStamped`   | `Float64Stamped`         |
| Number of Maps | 2 (throttle and brake)      | 1                        |
| Use Case       | Vehicle control             | Generic value mapping    |

### When to Use Generic vs. Original Converter

**Use `autoware_raw_vehicle_cmd_converter` when:**

- You have a standard vehicle with throttle/brake pedal control
- You need separate acceleration and braking maps
- You want integrated steering control
- You're working with standard Autoware vehicle interfaces

**Use `autoware_generic_value_converter` when:**

- You have custom actuators (e.g., electric motor controllers, hydraulic systems)
- You need a single unified mapping (not separate throttle/brake)
- You want to output arbitrary float64 values for custom control systems
- You're prototyping or testing new control interfaces
- You're working with simulators that require custom input formats

## Example Use Cases

1. **Custom Actuator Control**: Control non-standard actuators requiring acceleration to be mapped to custom control values
2. **Simulation Environment**: Convert acceleration to simulator-specific input format
3. **Testing and Development**: Test control algorithms by converting acceleration commands to visualizing or recordable values
4. **Sensor Fusion**: Map acceleration to other sensor input ranges

## Integration Examples

### Using with Calibrator

```bash
# 1. First run the calibrator to collect data and generate mapping
ros2 launch autoware_generic_value_calibrator generic_value_calibrator.launch.xml

# 2. After calibration, run converter with generated mapping
ros2 launch autoware_generic_value_converter generic_value_converter.launch.xml \
  csv_path_value_map:=~/autoware_map_calibration/value_map.csv
```

### Custom Node Subscribing to Output

```python
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float64Stamped

class ValueSubscriber(Node):
    def __init__(self):
        super().__init__('value_subscriber')
        self.subscription = self.create_subscription(
            Float64Stamped,
            '/generic/output/value',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(f'Received value: {msg.data}')
```
