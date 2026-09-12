# Topic Converter for Generic Value Calibrator

## Overview

The Topic Converter is a flexible ROS 2 node that transforms various message types into `tier4_debug_msgs::msg::Float64Stamped` format required by the Generic Value Calibrator. This allows you to calibrate custom actuators or control values against vehicle acceleration without writing custom code.

## Why Do You Need This?

The Generic Value Calibrator requires input in `Float64Stamped` format, but most real-world data comes in structured messages like:

- `autoware_control_msgs::msg::Control` (for acceleration commands)
- `nav_msgs::msg::Odometry` (for velocity feedback)
- `geometry_msgs::msg::TwistStamped` (for motion data)
- Custom sensor messages

The Topic Converter bridges this gap by extracting specific fields from these messages and converting them to the required format.

## Quick Start

### Example 1: Calibrate Control Acceleration (Most Common)

This is the typical use case - learning how your custom actuator responds to acceleration commands from Autoware's controller.

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  enable_converter:=true \
  input_topic:=/control/command/control_cmd \
  conversion_type:=control_acceleration
```

**What this does:**

1. Subscribes to `/control/command/control_cmd` (Control messages)
2. Extracts the `longitudinal.acceleration` field
3. Converts it to `Float64Stamped`
4. Feeds it to the calibrator which learns the relationship between requested acceleration and actual vehicle response

### Example 2: Calibrate Steering Angle

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  enable_converter:=true \
  input_topic:=/control/command/control_cmd \
  conversion_type:=control_steering_angle \
  converted_topic:=/generic/input/steering_value
```

### Example 3: With Scaling (e.g., m/s to km/h)

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  enable_converter:=true \
  input_topic:=/vehicle/odometry \
  conversion_type:=odom_linear_x \
  scale_factor:=3.6
```

## Supported Conversion Types

### From `autoware_control_msgs::msg::Control`

| Conversion Type          | Extracted Field                       | Description                       |
| ------------------------ | ------------------------------------- | --------------------------------- |
| `control_acceleration`   | `longitudinal.acceleration`           | Longitudinal acceleration command |
| `control_velocity`       | `longitudinal.velocity`               | Longitudinal velocity command     |
| `control_steering_angle` | `lateral.steering_tire_angle`         | Steering angle command            |
| `control_steering_rate`  | `lateral.steering_tire_rotation_rate` | Steering rate command             |

### From `nav_msgs::msg::Odometry`

| Conversion Type  | Extracted Field         | Description                    |
| ---------------- | ----------------------- | ------------------------------ |
| `odom_linear_x`  | `twist.twist.linear.x`  | Linear velocity in X direction |
| `odom_linear_y`  | `twist.twist.linear.y`  | Linear velocity in Y direction |
| `odom_angular_z` | `twist.twist.angular.z` | Angular velocity (yaw rate)    |

### From `geometry_msgs::msg::TwistStamped`

| Conversion Type   | Extracted Field   | Description        |
| ----------------- | ----------------- | ------------------ |
| `twist_linear_x`  | `twist.linear.x`  | Linear velocity X  |
| `twist_angular_z` | `twist.angular.z` | Angular velocity Z |

### From `geometry_msgs::msg::AccelStamped`

| Conversion Type  | Extracted Field  | Description           |
| ---------------- | ---------------- | --------------------- |
| `accel_linear_x` | `accel.linear.x` | Linear acceleration X |

## Advanced Usage

### Custom Field Extraction

You can extract any field from a message using dot notation:

```bash
ros2 run autoware_generic_value_calibrator topic_converter.py \
  --ros-args \
  -p input_topic:=/custom/sensor/data \
  -p output_topic:=/generic/input/value \
  -p conversion_type:=control_acceleration \
  -p custom_field:=data.measured_force
```

### Applying Scale and Offset

Transform values using: `output = input * scale_factor + offset`

```bash
# Convert temperature from Celsius to Fahrenheit
ros2 run autoware_generic_value_calibrator topic_converter.py \
  --ros-args \
  -p scale_factor:=1.8 \
  -p offset:=32.0
```

### Testing the Converter Alone

You can test the converter without running the full calibrator:

```bash
# Terminal 1: Run converter
ros2 run autoware_generic_value_calibrator topic_converter.py \
  --ros-args \
  -p input_topic:=/control/command/control_cmd \
  -p output_topic:=/test/float64_output \
  -p conversion_type:=control_acceleration

# Terminal 2: Monitor output
ros2 topic echo /test/float64_output

# Terminal 3: Check conversion rate
ros2 topic hz /test/float64_output
```

## Launch File Parameters

### `generic_value_calibrator_with_converter.launch.xml`

| Parameter          | Type   | Default                        | Description                          |
| ------------------ | ------ | ------------------------------ | ------------------------------------ |
| `enable_converter` | bool   | `true`                         | Enable/disable the topic converter   |
| `input_topic`      | string | `/control/command/control_cmd` | Input topic to convert               |
| `conversion_type`  | string | `control_acceleration`         | Type of conversion (see table above) |
| `custom_field`     | string | `""`                           | Custom field path for extraction     |
| `scale_factor`     | float  | `1.0`                          | Multiply converted value by this     |
| `offset`           | float  | `0.0`                          | Add this to converted value          |
| `converted_topic`  | string | `/generic/input/value`         | Output topic name                    |

Plus all standard calibrator parameters:

- `update_hz`
- `csv_default_map_dir`
- `csv_calibrated_map_dir`
- `output_log_file`

## Use Cases

### 1. Custom Electric Motor Controller

You have a custom motor controller that takes a normalized command [-1.0, 1.0] and you want to learn how it maps to vehicle acceleration:

```bash
# Publish your motor command as Float64Stamped to /motor/command
# Then run:
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  enable_converter:=false
```

### 2. Hydraulic Brake System

Learning the relationship between brake pressure command and vehicle deceleration:

```bash
# If your brake pressure comes from a Control message:
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  input_topic:=/vehicle/brake_cmd \
  conversion_type:=control_acceleration \
  scale_factor:=-1.0  # Negative because braking decelerates
```

### 3. Throttle Percentage Calibration

Calibrating throttle percentage (0-100%) against acceleration:

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  input_topic:=/vehicle/throttle_cmd \
  conversion_type:=control_acceleration \
  scale_factor:=0.01  # Convert percentage to 0-1 range
```

### 4. Learning from Logged Data

If you have rosbag data with various command types:

```bash
# Terminal 1: Play rosbag
ros2 bag play my_calibration_data.bag

# Terminal 2: Run calibrator with converter
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  input_topic:=/logged/control/command \
  conversion_type:=control_acceleration \
  csv_calibrated_map_dir:=/path/to/output
```

## Troubleshooting

### Converter not receiving messages

```bash
# Check if input topic exists and has the right type
ros2 topic list
ros2 topic info /control/command/control_cmd

# Check converter is running
ros2 node list | grep topic_converter

# Check converter logs
ros2 node info /topic_converter
```

### Wrong conversion type error

```bash
# List available conversion types:
ros2 run autoware_generic_value_calibrator topic_converter.py --ros-args -p conversion_type:=invalid
# This will error and show available types
```

### Values seem wrong

```bash
# Echo both input and output to compare:
ros2 topic echo /control/command/control_cmd &
ros2 topic echo /generic/input/value &

# Check if scale_factor or offset is needed
```

## Implementation Details

### Message Type Detection

The converter automatically determines the correct message type based on `conversion_type`. No manual type specification needed.

### Timestamp Handling

- For messages with `stamp` field: Uses that timestamp
- For messages with `header.stamp`: Uses that timestamp
- Otherwise: Uses current ROS time

This ensures proper time synchronization for the calibrator's delay compensation.

### Performance

- Minimal latency (<1ms typical)
- Zero-copy where possible
- Suitable for real-time control loops (100Hz+)

## Integration with Generic Value Converter

After calibration, use the generated CSV map with `autoware_generic_value_converter`:

```bash
# 1. Calibrate (creates map)
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml

# 2. Use the map for control (converts acceleration to your custom command)
ros2 launch autoware_generic_value_converter generic_value_converter.launch.xml \
  csv_path:=$HOME/autoware_map_calibration/generic_value_map.csv
```

## See Also

- [example_conversions.yaml](config/example_conversions.yaml) - Comprehensive examples
- [Generic Value Calibrator README](README.md) - Main calibrator documentation
- [Generic Value Converter README](../autoware_generic_value_converter/README.md) - Using calibrated maps

## Contributing

To add support for new message types:

1. Import the message type in `topic_converter.py`
2. Add extraction function (e.g., `_extract_your_field`)
3. Register in `CONVERSION_MAP`
4. Update this documentation

Example:

```python
# Add to imports
from your_package.msg import YourMessage

# Add extraction function
def _extract_your_field(self, msg):
    return msg.header.stamp, msg.your_field

# Register in _setup_conversions()
self.CONVERSION_MAP['your_conversion_type'] = (
    YourMessage, self._extract_your_field
)
```
