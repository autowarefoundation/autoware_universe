# autoware_generic_value_calibrator

## Overview

> **📌 Quick Start with Topic Converter**: If your input data is not already in `Float64Stamped` format (e.g., you want to use acceleration from `Control` messages), see [README_CONVERTER.md](README_CONVERTER.md) for easy topic conversion examples.

`autoware_generic_value_calibrator` is a generic value calibration node that automatically calibrates the mapping relationship between arbitrary float64 input values and vehicle acceleration. It is similar to `autoware_accel_brake_map_calibrator`, but works with any generic numeric input rather than just throttle/brake pedal values.

## Features

- **Generic**: Accepts `tier4_debug_msgs::msg::Float64Stamped` messages as input
- **Automatic Calibration**: Uses Recursive Least Squares (RLS) algorithm to automatically update velocity-acceleration mapping
- **Data Filtering**: Automatically filters invalid data (large steering angles, pitch angles, jerk, etc.)
- **Delay Compensation**: Accounts for delay between input value and actual acceleration
- **Pitch Compensation**: Removes gravity component from acceleration measurements
- **Real-time Evaluation**: Calculates RMSE to evaluate mapping accuracy
- **CSV Storage**: Saves calibrated mapping to CSV format

## Input Topics

| Topic Name         | Message Type                                 | Description                  |
| ------------------ | -------------------------------------------- | ---------------------------- |
| `~/input/velocity` | `autoware_vehicle_msgs::msg::VelocityReport` | Vehicle velocity information |
| `~/input/steer`    | `autoware_vehicle_msgs::msg::SteeringReport` | Steering angle information   |
| `~/input/value`    | `tier4_debug_msgs::msg::Float64Stamped`      | Generic float64 input value  |

## Output Topics

| Topic Name                   | Message Type                            | Description                |
| ---------------------------- | --------------------------------------- | -------------------------- |
| `~/output/update_suggest`    | `std_msgs::msg::Bool`                   | Flag suggesting map update |
| `~/output/current_map_error` | `tier4_debug_msgs::msg::Float64Stamped` | Current map error          |
| `~/output/updated_map_error` | `tier4_debug_msgs::msg::Float64Stamped` | Updated map error          |
| `~/output/map_error_ratio`   | `tier4_debug_msgs::msg::Float64Stamped` | Error ratio                |

## Parameters

### System Parameters

| Parameter Name           | Type   | Default Value                          | Description                                                           |
| ------------------------ | ------ | -------------------------------------- | --------------------------------------------------------------------- |
| `update_hz`              | double | 10.0                                   | Update frequency                                                      |
| `update_method`          | string | "update_offset_each_cell"              | Update algorithm ("update_offset_each_cell" or "update_offset_total") |
| `get_pitch_method`       | string | "tf"                                   | Method to get pitch angle ("tf" or "none")                            |
| `progress_file_output`   | bool   | false                                  | Flag to output log file for detailed calibration data                 |
| `csv_default_map_dir`    | string | ""                                     | Default map directory (if empty, uses parameters below)               |
| `csv_calibrated_map_dir` | string | "$(env HOME)/autoware_map_calibration" | Calibrated map directory where output CSV will be saved               |
| `output_log_file`        | string | ""                                     | Path to log file for detailed calibration data                        |
| `precision`              | int    | 3                                      | Decimal precision for CSV output                                      |

### Map Index Parameters

| Parameter Name | Type   | Default Value | Description                           |
| -------------- | ------ | ------------- | ------------------------------------- |
| `value_min`    | double | -5.0          | Minimum input value range             |
| `value_max`    | double | 3.0           | Maximum input value range             |
| `value_num`    | int    | 11            | Number of points in input value range |
| `velocity_min` | double | 0.0           | Minimum velocity range (m/s)          |
| `velocity_max` | double | 20.0          | Maximum velocity range (m/s)          |
| `velocity_num` | int    | 11            | Number of points in velocity range    |

> **Note**: If `csv_default_map_dir` is provided, the map indices will be loaded from the CSV file and the index parameters above will be ignored. This ensures that each map file is self-contained with its own scale and range.

### Algorithm Parameters

| Parameter Name            | Type   | Default Value | Description                                            |
| ------------------------- | ------ | ------------- | ------------------------------------------------------ |
| `initial_covariance`      | double | 0.05          | Initial covariance for RLS algorithm                   |
| `velocity_min_threshold`  | double | 0.1           | Minimum velocity threshold                             |
| `velocity_diff_threshold` | double | 0.556         | Velocity difference threshold                          |
| `value_diff_threshold`    | double | 0.03          | Value difference threshold                             |
| `max_steer_threshold`     | double | 0.2           | Maximum steering angle threshold                       |
| `max_pitch_threshold`     | double | 0.02          | Maximum pitch angle threshold                          |
| `max_jerk_threshold`      | double | 0.7           | Maximum jerk threshold                                 |
| `value_velocity_thresh`   | double | 0.15          | Value-velocity threshold                               |
| `max_accel`               | double | 5.0           | Maximum acceleration limit (m/s²)                      |
| `min_accel`               | double | -5.0          | Minimum acceleration limit (m/s²)                      |
| `value_to_accel_delay`    | double | 0.3           | Delay from input value to acceleration                 |
| `update_suggest_thresh`   | double | 0.7           | Threshold for update suggestion (RMSE ratio threshold) |
| `max_data_count`          | int    | 200           | Maximum number of data points per cell                 |

## Usage

### Configuring Input Value Range

The calibrator supports flexible input value ranges through parameters. You can configure:

1. **Value Range**: `value_min`, `value_max`, `value_num`

   - Example: For a custom control signal ranging from -10 to 10:

     ```yaml
     value_min: -10.0
     value_max: 10.0
     value_num: 21 # Creates 21 points from -10 to 10
     ```

2. **Velocity Range**: `velocity_min`, `velocity_max`, `velocity_num`

   - Example: For high-speed vehicles (0-30 m/s):

     ```yaml
     velocity_min: 0.0
     velocity_max: 30.0
     velocity_num: 16 # Creates 16 points
     ```

**Important**:

- If you provide a `csv_default_map_dir`, the indices will be loaded from the CSV file (self-contained)
- If no CSV is provided, the map will be generated from the parameters above
- The converter automatically reads indices from the CSV file, so no separate configuration is needed

### Basic Usage (with Float64Stamped input)

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator.launch.xml
```

### Custom Range Example

```bash
ros2 launch autoware_generic_value_calibrator generic_value_calibrator.launch.xml \
  value_min:=-5.0 value_max:=5.0 value_num:=21 \
  velocity_min:=0.0 velocity_max:=30.0 velocity_num:=16
```

### With Topic Converter (recommended for most users)

If your input is not already `Float64Stamped` (e.g., you want to use Control messages):

```bash
# Example: Calibrate using acceleration from Control messages
ros2 launch autoware_generic_value_calibrator generic_value_calibrator_with_converter.launch.xml \
  enable_converter:=true \
  input_topic:=/control/command/control_cmd \
  conversion_type:=control_acceleration
```

See [README_CONVERTER.md](README_CONVERTER.md) for more examples and conversion types.

### Calibration Workflow

1. **Launch Calibrator**: Load default mapping
2. **Collect Data**: Drive vehicle or publish test data
3. **Real-time Filtering**: Only use stable data meeting conditions
4. **RLS Update**: Iteratively correct mapping offsets
5. **Evaluate Accuracy**: Calculate RMSE to determine if saving is needed
6. **Save Mapping**: Map automatically saved to CSV file
7. **Monitor Progress**: View real-time visualization in RViz or generated plots

## Visualization

### RViz Real-time Display

Launch RViz with the provided configuration:

```bash
rviz2 -d $(ros2 pkg prefix autoware_generic_value_calibrator)/share/autoware_generic_value_calibrator/rviz/occupancy.rviz
```

**Visualization Topics:**

- `~/debug/data_count_self_pose_occ_map` - Data coverage with current position
- `~/debug/original_occ_map` - Original default map
- `~/debug/update_occ_map` - Calibrated map
- `~/debug/data_average_occ_map` - Average acceleration values
- `~/debug/data_std_dev_occ_map` - Standard deviation
- `~/debug/occ_index` - Velocity and value index labels

### SVG Plot Generation

Run the visualization server to generate detailed plots:

```bash
ros2 run autoware_generic_value_calibrator generic_value_map_server.py
```

This creates `plot.svg` in the calibrated map directory with:

- Default vs calibrated map comparison
- All data points colored by pitch
- Statistics (average, stddev, count) for each cell
- Separate subplot for each velocity point

See [VISUALIZATION_IMPLEMENTATION.md](../VISUALIZATION_IMPLEMENTATION.md) for details.

## Calibration Methods

### Data Preprocessing

Before calibration, the following invalid data is automatically filtered:

- Low velocity
- Large steering angles
- Large pitch angles
- High jerk
- Fast input value changes

### Update Algorithms

The calibrator uses **Recursive Least Squares (RLS)** algorithm to iteratively update the mapping. RLS is an online learning algorithm that adapts the map based on new measurements without requiring all historical data.

#### RLS Algorithm Overview

The RLS algorithm maintains:

- **Offset estimate** (`map_offset`): The correction to apply to the map value
- **Covariance** (`covariance`): Uncertainty in the offset estimate

For each new measurement `(value, velocity, measured_acceleration)`:

1. Calculate the error: `error = measured_acceleration - map_acceleration`
2. Update covariance: `covariance = (covariance - covariance²/(forgetting_factor + covariance)) / forgetting_factor`
3. Calculate gain: `gain = covariance / (forgetting_factor + covariance)`
4. Update offset: `offset = offset + gain × error`
5. Apply offset to map: `updated_map_value = original_map_value + offset`

The **forgetting factor** (default: 0.999) controls how much weight is given to new data vs. old data. A value closer to 1.0 means the algorithm adapts more slowly but is more stable.

#### UPDATE_OFFSET_EACH_CELL

Uses RLS update independently for each grid cell in the map.

**How it works**:

- Each `(value_index, velocity_index)` cell has its own offset and covariance
- Only data points that fall within the cell's threshold are used for that cell's update
- The map is updated cell-by-cell as data is collected

**Advantages**:

- High accuracy using nearby data for each point
- Can capture non-linear relationships across the map
- Better for complex mappings with varying characteristics

**Disadvantages**:

- Requires large amounts of data to fill all cells
- Longer calibration time
- May have sparse coverage in some regions

**Best for**: Detailed calibration when you have extensive driving data covering the full operating range.

#### UPDATE_OFFSET_TOTAL

Calculates a single global offset and applies it to the entire mapping.

**How it works**:

- Maintains one global offset and covariance for the entire map
- All valid data points contribute to updating this single offset
- The same offset is added to every cell in the map

**Advantages**:

- Simple and fast
- Works well with limited data
- Good for maps that need a uniform correction

**Disadvantages**:

- Cannot capture cell-specific variations
- Assumes the error is uniform across all operating conditions
- Potentially lower accuracy for complex mappings

**Best for**: Quick calibration, initial setup, or when the map error is known to be uniform.

### Default Map Generation

When no CSV file is provided via `csv_default_map_dir`, the calibrator generates a default map from the parameter values:

**Default Relationship**: `acceleration = input_value`

This creates a simple linear mapping where:

- Positive input values produce positive acceleration
- Negative input values produce negative acceleration (braking)
- The acceleration is constant across all velocities (no velocity dependency)

**Example**: With `value_min=-5.0`, `value_max=3.0`, `value_num=11`:

- Input value `-5.0` → acceleration `-10.0 m/s²`
- Input value `0.0` → acceleration `0.0 m/s²`
- Input value `3.0` → acceleration `6.0 m/s²`

**Note**: This default is intentionally simple. The RLS algorithm will update it based on actual vehicle data during calibration. For better initial estimates, provide a pre-calibrated map via `csv_default_map_dir`.

## CSV File Format

Mapping is stored in CSV format as follows:

```csv
default,0.0,2.0,4.0,6.0,8.0,10.0,12.0,14.0,16.0,18.0,20.0
-5.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0,-10.0
-4.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0,-8.0
...
3.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0,6.0
```

**Format Structure**:

- **First row**: "default" (vehicle/map name) followed by velocity indices in m/s
- **First column**: Input value indices (from `value_min` to `value_max`)
- **Cells**: Corresponding acceleration values in m/s² at `(input_value, velocity)`

**Default Range**: The default value range is `[-5.0, 3.0]` with 11 points, but this can be customized via parameters when creating a new map.

## Comparison with Original Calibrator

| Feature        | accel_brake_map_calibrator  | generic_value_calibrator    |
| -------------- | --------------------------- | --------------------------- |
| Input Type     | Throttle/Brake pedal values | Arbitrary float64 values    |
| Input Message  | `ActuationCommandStamped`   | `Float64Stamped`            |
| Number of Maps | 2 (throttle and brake)      | 1                           |
| Use Case       | Vehicle control             | Generic mapping calibration |

### When to Use Generic vs. Original Calibrator

**Use `autoware_accel_brake_map_calibrator` when:**

- You have a standard vehicle with separate throttle and brake pedals
- Your vehicle interface uses `ActuationCommandStamped` messages
- You need separate calibration for acceleration and braking
- You want to follow the standard Autoware vehicle control pipeline
- You're working with production vehicles that have well-defined pedal interfaces

**Use `autoware_generic_value_calibrator` when:**

- You have custom actuators or control interfaces (e.g., electric motors, hydraulic systems)
- Your input is already in `Float64Stamped` format or can be easily converted
- You need a single unified mapping (not separate throttle/brake)
- You're prototyping new control systems
- You're working with simulators or test platforms
- You need flexible input value ranges (not limited to pedal values)
- You want to calibrate non-standard control signals (e.g., motor current, pressure, percentage)

**Migration Path**: If you're currently using `accel_brake_map_calibrator` but want to switch to `generic_value_calibrator`:

1. Use the topic converter to extract throttle/brake values from `ActuationCommandStamped`
2. Calibrate using the generic calibrator
3. Use the generic converter instead of `raw_vehicle_cmd_converter`
