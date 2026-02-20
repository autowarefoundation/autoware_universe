# autoware_carla_interface

## ROS 2 / Autoware Universe bridge for CARLA simulator

Thanks to <https://github.com/gezp> for ROS 2 Humble support for CARLA Communication.
This ros package enables communication between Autoware and CARLA for autonomous driving simulation.

## Supported Environment

| ubuntu |  ros   | carla  | autoware |
| :----: | :----: | :----: | :------: |
| 22.04  | humble | 0.9.15 |   Main   |

## Setup

### Install

#### Prerequisites

1. **Install CARLA 0.9.15**: Follow the [CARLA Installation Guide](https://carla.readthedocs.io/en/latest/start_quickstart/)

2. **Install CARLA Python Package**: Install [CARLA 0.9.15 ROS 2 Humble communication package](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)
   - Option A: Install the wheel using pip
   - Option B: Add the egg file to your `PYTHONPATH`

3. **Download CARLA Lanelet2 Maps**: Get the y-axis inverted maps from [CARLA Autoware Contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)

#### Map Setup

1. Download the maps (y-axis inverted version) to an arbitrary location
2. Create the map folder structure in `$HOME/autoware_map`:
   - Rename `point_cloud/Town01.pcd` → `$HOME/autoware_map/Town01/pointcloud_map.pcd`
   - Rename `vector_maps/lanelet2/Town01.osm` → `$HOME/autoware_map/Town01/lanelet2_map.osm`
3. Create `$HOME/autoware_map/Town01/map_projector_info.yaml` with:

   ```yaml
   projector_type: Local
   ```

### Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

1. Run carla, change map, spawn object if you need
   <!--- cspell:ignore prefernvidia -->

   ```bash
   cd CARLA
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
   ```

2. Run Autoware with CARLA

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml \
       map_path:=$HOME/autoware_map/Town01 \
       vehicle_model:=sample_vehicle \
       sensor_model:=carla_sensor_kit \
       simulator_type:=carla
   ```

   For E2E planning with VAD:

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml \
       map_path:=$HOME/autoware_map/Town01 \
       vehicle_model:=sample_vehicle \
       sensor_model:=carla_sensor_kit \
       simulator_type:=carla \
       use_e2e_planning:=true
   ```

3. Set initial pose (Init by GNSS)
4. Set goal position
5. Wait for planning
6. Engage

### Viewing Multi-Camera View in RViz

The `carla_sensor_kit` includes 6 cameras providing 360-degree coverage (Front, Front-Left, Front-Right, Back, Back-Left, Back-Right). A multi-camera combiner node automatically combines all camera feeds into a single 2x3 grid view.

To view the combined camera feed in RViz:

1. In the **Displays** panel (left side), click the **"Add"** button
2. Select the **"By topic"** tab
3. Navigate to `/sensing/camera/all_cameras/image_raw`
4. Select **"Image"** display type
5. Click **OK**

![Multi-Camera View in RViz](docs/images/rviz_multi_camera_view.png)

The combined view shows all 6 cameras with labels: FL (Front-Left), F (Front), FR (Front-Right), BL (Back-Left), B (Back), BR (Back-Right).

**Note:** If you don't need the multi-camera combiner (to save CPU resources), you can comment out the following line in `launch/autoware_carla_interface.launch.xml`:

```xml
<!-- Multi-camera combiner for RViz visualization -->
<!-- <node pkg="autoware_carla_interface" exec="multi_camera_combiner" output="screen"/> -->
```

## Inner-workings / Algorithms

The `InitializeInterface` class is key to setting up both the CARLA world and the ego vehicle. It fetches configuration parameters through the `autoware_carla_interface.launch.xml`.

The main simulation loop runs within the `carla_ros2_interface` class. This loop ticks simulation time inside the CARLA simulator at `fixed_delta_seconds` time, where data is received and published as ROS 2 messages at frequencies defined in `self.sensor_frequencies`.

Ego vehicle commands from Autoware are processed through the `autoware_raw_vehicle_cmd_converter`, which calibrates these commands for CARLA. The calibrated commands are then fed directly into CARLA control via `CarlaDataProvider`.

### Configurable Parameters for World Loading

All the key parameters can be configured in `autoware_carla_interface.launch.xml`.

| Name                     | Type   | Default Value                                                                     | Description                                                                                                                                                                                                         |
| ------------------------ | ------ | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `host`                   | string | "localhost"                                                                       | Hostname for the CARLA server                                                                                                                                                                                       |
| `port`                   | int    | "2000"                                                                            | Port number for the CARLA server                                                                                                                                                                                    |
| `timeout`                | int    | 20                                                                                | Timeout for the CARLA client                                                                                                                                                                                        |
| `ego_vehicle_role_name`  | string | "ego_vehicle"                                                                     | Role name for the ego vehicle                                                                                                                                                                                       |
| `vehicle_type`           | string | "vehicle.toyota.prius"                                                            | Blueprint ID of the vehicle to spawn. The Blueprint ID of vehicles can be found in [CARLA Blueprint ID](https://carla.readthedocs.io/en/latest/catalogue_vehicles/)                                                 |
| `spawn_point`            | string | None                                                                              | Coordinates for spawning the ego vehicle (None is random). Format = [x, y, z, roll, pitch, yaw]                                                                                                                     |
| `sync_mode`              | bool   | True                                                                              | Boolean flag to set synchronous mode in CARLA                                                                                                                                                                       |
| `fixed_delta_seconds`    | double | 0.05                                                                              | Time step for the simulation (related to client FPS)                                                                                                                                                                |
| `use_traffic_manager`    | bool   | False                                                                             | Boolean flag to set traffic manager in CARLA                                                                                                                                                                        |
| `max_real_delta_seconds` | double | 0.05                                                                              | Parameter to limit the simulation speed below `fixed_delta_seconds`                                                                                                                                                 |
| `sensor_kit_name`        | string | "carla_sensor_kit_description"                                                    | Name of the sensor kit package to use for sensor configuration. Should be the \*\_description package containing config/sensor_kit_calibration.yaml                                                                 |
| `sensor_mapping_file`    | string | "$(find-pkg-share autoware_carla_interface)/config/sensor_mapping.yaml"           | Path to sensor mapping YAML configuration file                                                                                                                                                                      |
| `config_file`            | string | "$(find-pkg-share autoware_carla_interface)/raw_vehicle_cmd_converter.param.yaml" | Control mapping file to be used in `autoware_raw_vehicle_cmd_converter`. Current control are calibrated based on `vehicle.toyota.prius` Blueprints ID in CARLA. Changing the vehicle type may need a recalibration. |

### Sensor Configuration

The interface uses the **`carla_sensor_kit`** which provides 6 cameras for 360-degree coverage, LiDAR, IMU, and GNSS sensors. Sensor configurations are dynamically loaded from Autoware sensor kit calibration files through two configuration files:

#### 1. Sensor Kit Calibration (from Autoware sensor kit)

Located in `<sensor_kit_name>_description/config/sensor_kit_calibration.yaml`

Defines sensor positions and orientations relative to `base_link` (rear axle center). Example:

```yaml
sensor_kit_base_link:
  CAM_FRONT/camera_link:
    x: 2.225
    y: 0.000
    z: 1.600
    roll: 0.000
    pitch: 0.000
    yaw: 0.000 # Angles in radians
```

#### 2. Sensor Mapping (CARLA-specific)

Located in `config/sensor_mapping.yaml`

Maps Autoware sensors to CARLA sensor types and parameters. Key sections:

- `default_sensor_kit_name`: Default sensor kit to use (e.g., `carla_sensor_kit_description`)
- `sensor_mappings`: Maps each sensor to CARLA type and ROS topics
- `enabled_sensors`: List of sensors to spawn in CARLA
- `vehicle_config` (optional): Vehicle parameters like wheelbase

Example sensor mapping:

```yaml
sensor_mappings:
  CAM_FRONT/camera_link:
    carla_type: sensor.camera.rgb
    id: CAM_FRONT
    ros_config:
      frame_id: CAM_FRONT/camera_optical_link
      topic_image: /sensing/camera/CAM_FRONT/image_raw
      topic_info: /sensing/camera/CAM_FRONT/camera_info
      frequency_hz: 11
      qos_profile: reliable
    parameters:
      image_size_x: 1600
      image_size_y: 900
      fov: 70.0
```

For CARLA sensor parameters, see [CARLA Sensor Reference](https://carla.readthedocs.io/en/latest/ref_sensors/).

### World Loading

The `carla_ros.py` sets up the CARLA world:

1. **Client Connection**:

   ```python
   client = carla.Client(self.local_host, self.port)
   client.set_timeout(self.timeout)
   ```

2. **Load the Map**:

   Map loaded in CARLA world with map according to `carla_map` parameter.

   ```python
   client.load_world(self.map_name)
   self.world = client.get_world()
   ```

3. **Spawn Ego Vehicle**:

   Vehicle are spawn according to `vehicle_type`, `spawn_point`, and `agent_role_name` parameter.

   ```python
   spawn_point = carla.Transform()
   point_items = self.spawn_point.split(",")
   if len(point_items) == 6:
      spawn_point.location.x = float(point_items[0])
      spawn_point.location.y = float(point_items[1])
      spawn_point.location.z = float(point_items[2]) + 2
      spawn_point.rotation.roll = float(point_items[3])
      spawn_point.rotation.pitch = float(point_items[4])
      spawn_point.rotation.yaw = float(point_items[5])
   CarlaDataProvider.request_new_actor(self.vehicle_type, spawn_point, self.agent_role_name)
   ```

## Traffic Light Recognition

The maps provided by the Carla Simulator ([Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)) currently lack proper traffic light components for Autoware and have different latitude and longitude coordinates compared to the pointcloud map. To enable traffic light recognition, follow the steps below to modify the maps.

- Options to Modify the Map
  - A. Create a New Map from Scratch
  - Use the [TIER IV Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) to create a new map.

  - B. Modify the Existing Carla Lanelet2 Maps
  - Adjust the longitude and latitude of the [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/) to align with the PCD (origin).
    - Use this [tool](https://github.com/mraditya01/offset_lanelet2/tree/main) to modify the coordinates.
    - Snap Lanelet with PCD and add the traffic lights using the [TIER IV Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/).

- When using the TIER IV Vector Map Builder, you must convert the PCD format from `binary_compressed` to `ascii`. You can use `pcl_tools` for this conversion.
- For reference, an example of Town01 with added traffic lights at one intersection can be downloaded [here](https://drive.google.com/drive/folders/1QFU0p3C8NW71sT5wwdnCKXoZFQJzXfTG?usp=sharing).

## Tips

- Misalignment might occurs during initialization, pressing `init by gnss` button should fix it.
- Changing the `fixed_delta_seconds` can increase the simulation tick (default 0.05 s), some sensor params in `sensor_mapping.yaml` need to be adjusted when it is changed (example: LIDAR rotation frequency should match the FPS).

## Known Issues and Future Works

- **Testing on procedural maps (Adv Digital Twin)**: Currently unable to test due to failures in creating the Adv Digital Twin map.
- **Traffic light recognition**: The default CARLA Lanelet2 maps lack proper traffic light regulatory elements. See the "Traffic Light Recognition" section above for workarounds.
