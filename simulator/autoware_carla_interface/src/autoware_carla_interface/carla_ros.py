# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import threading

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_perception_msgs.msg import PredictedObjects
from autoware_perception_msgs.msg import TrafficLightGroupArray
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import HazardLightsCommand
from autoware_vehicle_msgs.msg import HazardLightsReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from transforms3d.euler import euler2quat

# New modular sensor infrastructure
from .modules import ROSPublisherManager
from .modules import SensorKitLoader
from .modules import SensorPublishWorker
from .modules import SensorRegistry
from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_utils import carla_location_to_ros_point
from .modules.carla_utils import carla_rotation_to_ros_quaternion
from .modules.carla_utils import create_cloud
from .modules.carla_utils import ros_pose_to_carla_transform
from .modules.carla_wrapper import SensorInterface


def _origin_from_usdz(usdz_path: str):
    """Derive ``(lat_0, lon_0)`` from a splatsim ``.usdz`` scene bundle.

    ``scene.json`` stores ``world.ecef_anchor``: the 4x4 ENU->ECEF transform
    whose translation column is the ECEF position of the ENU/world origin the
    3DGS scene is aligned to.  Converting that point to WGS84 yields exactly the
    geographic origin splatsim needs, so no external lanelet2 map or OpenDRIVE
    GeoReference is required.
    """
    import json
    import zipfile

    from pyproj import Transformer

    try:
        with zipfile.ZipFile(usdz_path) as zf:
            with zf.open("scene.json") as f:
                scene = json.load(f)
        anchor = scene["world"]["ecef_anchor"]
    except (KeyError, OSError, zipfile.BadZipFile, ValueError) as exc:
        raise RuntimeError(
            f"Could not read 'scene.json' -> world.ecef_anchor from splatsim scene "
            f"'{usdz_path}'.  A splatsim .usdz bundle is required to derive the "
            f"geographic origin."
        ) from exc
    x, y, z = anchor[0][3], anchor[1][3], anchor[2][3]
    ecef_to_lla = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
    lon, lat, _ = ecef_to_lla.transform(x, y, z)
    return float(lat), float(lon)


class carla_ros2_interface(object):

    def _initialize_parameters(self):
        """Initialize and declare ROS 2 parameters."""
        # Parameter definitions: name -> (type, default_value)
        # None means no default (must be provided by launch file)
        self.parameters = {
            "host": (rclpy.Parameter.Type.STRING, None),
            "port": (rclpy.Parameter.Type.INTEGER, None),
            "sync_mode": (rclpy.Parameter.Type.BOOL, None),
            "timeout": (rclpy.Parameter.Type.INTEGER, None),
            "fixed_delta_seconds": (rclpy.Parameter.Type.DOUBLE, None),
            "carla_map": (rclpy.Parameter.Type.STRING, None),
            "ego_vehicle_role_name": (rclpy.Parameter.Type.STRING, None),
            "spawn_point": (rclpy.Parameter.Type.STRING, None),
            "vehicle_type": (rclpy.Parameter.Type.STRING, None),
            "use_traffic_manager": (rclpy.Parameter.Type.BOOL, None),
            "max_real_delta_seconds": (rclpy.Parameter.Type.DOUBLE, None),
            "initial_pose_ground_offset_z": (rclpy.Parameter.Type.DOUBLE, 1.0),
            # Sensor configuration parameters
            "sensor_kit_name": (rclpy.Parameter.Type.STRING, ""),  # Empty = use YAML default
            "sensor_mapping_file": (rclpy.Parameter.Type.STRING, ""),
            # SplatSim parameters (global only; per-sensor rendering settings are
            # read from the sensor mapping's per-sensor `parameters:` block).
            "render_with_splatsim": (rclpy.Parameter.Type.BOOL, False),
            "splatsim_render_camera": (rclpy.Parameter.Type.BOOL, True),
            "splatsim_render_lidar": (rclpy.Parameter.Type.BOOL, False),
            "splatsim_tileset_path": (rclpy.Parameter.Type.STRING, ""),
            "splatsim_image": (rclpy.Parameter.Type.STRING, "splatsim:latest"),
            "splatsim_grpc_port": (rclpy.Parameter.Type.INTEGER, 50051),
            "splatsim_lidar_grpc_port": (rclpy.Parameter.Type.INTEGER, 50061),
            "splatsim_use_sh": (rclpy.Parameter.Type.BOOL, True),
            "splatsim_device": (rclpy.Parameter.Type.STRING, "cuda:0"),
            "splatsim_restart_container": (rclpy.Parameter.Type.BOOL, False),
            "splatsim_compress_format": (rclpy.Parameter.Type.STRING, "jpeg"),
        }

        self.param_values = {}
        for param_name, (param_type, default_value) in self.parameters.items():
            if default_value is not None:
                self.ros2_node.declare_parameter(param_name, default_value)
            else:
                self.ros2_node.declare_parameter(param_name, param_type)
            self.param_values[param_name] = self.ros2_node.get_parameter(param_name).value

    def _initialize_clock_publisher(self):
        """Initialize and publish initial clock message."""
        self.clock_publisher = self.ros2_node.create_publisher(Clock, "/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)

    def _setup_tf_listener(self):
        """Initialize TF buffer/listener for map alignment."""
        self.tf_buffer = None
        self.tf_listener = None

    def _initialize_status_publishers(self):
        """
        Initialize all vehicle status publishers.

        Note: GNSS pose publisher is now managed via sensor registry.
        Only vehicle status publishers are created here.

        """
        self.pub_vel_state = self.ros2_node.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.ros2_node.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.ros2_node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.ros2_node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        self.pub_actuation_status = self.ros2_node.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", 1
        )
        self.pub_turn_indicators_state = self.ros2_node.create_publisher(
            TurnIndicatorsReport, "/vehicle/status/turn_indicators_status", 1
        )
        self.pub_hazard_lights_state = self.ros2_node.create_publisher(
            HazardLightsReport, "/vehicle/status/hazard_lights_status", 1
        )

    def _initialize_subscriptions(self):
        """Initialize all ROS 2 subscriptions."""
        self.sub_control = self.ros2_node.create_subscription(
            ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        )
        self.sub_vehicle_initialpose = self.ros2_node.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 1
        )
        self.sub_turn_indicators = self.ros2_node.create_subscription(
            TurnIndicatorsCommand,
            "/control/command/turn_indicators_cmd",
            self.turn_indicators_callback,
            1,
        )
        self.sub_hazard_lights = self.ros2_node.create_subscription(
            HazardLightsCommand,
            "/control/command/hazard_lights_cmd",
            self.hazard_lights_callback,
            1,
        )
        self.current_control = carla.VehicleControl()

    def _load_sensor_configuration(self):
        """Load sensor configuration and prepare publishers/metadata."""
        self.sensor_registry.clear()
        self.sensor_configs = []

        mapping_file = self.param_values.get("sensor_mapping_file", "")
        if not self.sensor_loader.load_sensor_mapping(mapping_file):
            raise FileNotFoundError(
                "Unable to locate sensor mapping YAML. "
                "Provide --ros-args -p sensor_mapping_file:=<path>"
            )

        sensor_kit_name = self._resolve_sensor_kit_name()
        self.logger.info(f"Using Autoware sensor kit calibration: {sensor_kit_name}")

        try:
            self.sensor_configs = self.sensor_loader.build_sensor_configs(
                sensor_kit_name=sensor_kit_name
            )
        except Exception as exc:
            self.logger.error(f"Failed to build sensor configuration from kit: {exc}")
            raise

        if not self.sensor_configs:
            raise RuntimeError(
                "Sensor mapping produced zero sensors. "
                "Check enabled_sensors list and calibration files."
            )

        # SplatSim: separate camera/LiDAR configs and filter them out of CARLA
        # sensors.  Camera and LiDAR rendering are independently gated so the
        # scene can be driven by LiDAR alone (splatsim_render_camera=false)
        # without spinning up a Docker container per camera.
        if self.render_with_splatsim:
            self.sensor_configs = self._partition_splatsim_configs()

        self._register_sensor_configs(self.sensor_configs)
        self._create_sensor_publishers_from_registry()
        self.sensors = {"sensors": self._build_sensor_specs(self.sensor_configs)}

        self.logger.info(f"Configured {len(self.sensor_configs)} sensors from mapping")

    def _partition_splatsim_configs(self) -> list:
        """Route camera/LiDAR configs to the splatsim lists; return the CARLA-only rest."""
        render_camera = bool(self.param_values.get("splatsim_render_camera", True))
        render_lidar = bool(self.param_values.get("splatsim_render_lidar"))
        carla_configs = []
        for cfg in self.sensor_configs:
            if cfg.carla_type.startswith("sensor.camera"):
                self._assign_splatsim_camera(cfg, render_camera)
            elif cfg.carla_type.startswith("sensor.lidar"):
                self._assign_splatsim_lidar(cfg, render_lidar)
            else:
                carla_configs.append(cfg)
        return carla_configs

    def _assign_splatsim_camera(self, cfg, render_camera) -> None:
        """Register a camera config for splatsim rendering, or log that it is disabled."""
        if render_camera:
            self._splatsim_camera_configs.append(cfg)
            self.logger.info(f"Rendering camera '{cfg.sensor_id}' via splatsim")
        else:
            self.logger.info(
                f"Skipping camera '{cfg.sensor_id}' (splatsim mode, camera rendering disabled)"
            )

    def _assign_splatsim_lidar(self, cfg, render_lidar) -> None:
        """Register a LiDAR config for splatsim rendering, or log that it is disabled."""
        if render_lidar:
            self._splatsim_lidar_configs.append(cfg)
            self.logger.info(f"Rendering LiDAR '{cfg.sensor_id}' via splatsim")
        else:
            self.logger.info(f"Skipping LiDAR '{cfg.sensor_id}' (splatsim mode)")

    def _resolve_sensor_kit_name(self) -> str:
        """Resolve the effective sensor kit name based on parameters and mapping."""
        param_value = (self.param_values.get("sensor_kit_name", "") or "").strip()
        if param_value:
            return param_value

        mapping_default = self.sensor_loader.sensor_mapping.get("default_sensor_kit_name", "")
        if mapping_default:
            return mapping_default

        self.logger.warning("No sensor kit name provided; using fallback 'sample_sensor_kit'")
        return "sample_sensor_kit"

    def _register_sensor_configs(self, configs):
        """Register sensors with the registry and update lookup tables."""
        self.id_to_sensor_type_map.clear()
        for config in configs:
            self.sensor_registry.register_sensor(config)
            self.id_to_sensor_type_map[config.sensor_id] = config.carla_type

    def _create_sensor_publishers_from_registry(self):
        """Create ROS publishers for all configured sensors."""
        self.ros_publisher_manager.create_publishers_for_registry(self.sensor_registry)

        for sensor_id, sensor in self.sensor_registry.get_all_sensors().items():
            if sensor.sensor_type.startswith("pseudo."):
                continue

            if sensor.carla_type.startswith("sensor.camera"):
                self.pub_camera[sensor_id] = sensor.publisher
                self.pub_camera_info[sensor_id] = sensor.publisher_info
            elif sensor.carla_type.startswith("sensor.lidar"):
                self.pub_lidar[sensor_id] = sensor.publisher
            elif sensor.carla_type.startswith("sensor.other.imu"):
                self.pub_imu = sensor.publisher

    def _build_sensor_specs(self, configs):
        """Convert sensor config objects to CARLA sensor specifications."""
        sensor_specs = []

        for config in configs:
            transform = config.transform or {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }

            spec = {
                "type": config.carla_type,
                "id": config.sensor_id,
                "spawn_point": transform,
            }

            spec.update(config.parameters)
            sensor_specs.append(spec)

        return sensor_specs

    def __init__(self):
        # Initialize instance variables
        self._initialize_instance_variables()

        # Initialize ROS 2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("carla_ros2_interface")
        self.logger = self.ros2_node.get_logger()
        self.sensor_registry.logger = self.logger
        self.ros_publisher_manager = ROSPublisherManager(self.ros2_node, logger=self.logger)

        # Setup all components
        self._initialize_parameters()
        self.render_with_splatsim = bool(self.param_values.get("render_with_splatsim", False))
        self._setup_tf_listener()
        self._initialize_clock_publisher()

        self._load_sensor_configuration()

        # Initialize publishers and subscriptions
        self._initialize_subscriptions()
        self._initialize_status_publishers()

        # SplatSim: create dummy perception and localization publishers
        if self.render_with_splatsim:
            self._initialize_splatsim_publishers()

        # Start ROS 2 spin thread (Thread Safety: Shared state protected by self._state_lock)
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()

    def _initialize_instance_variables(self):
        """Initialize baseline state before the ROS node is created."""
        # Sensor data bridge
        self.sensor_interface = SensorInterface()

        # Sensor metadata managers
        self.sensor_registry = SensorRegistry()
        self.sensor_loader = SensorKitLoader()
        self.sensor_configs = []

        # Legacy compatibility containers (gradually phased out)
        self.id_to_sensor_type_map = {}
        self.pub_camera = {}
        self.pub_camera_info = {}
        self.pub_lidar = {}
        self.pub_imu = None
        self.camera_info_cache = {}

        # Vehicle and control state
        self.prev_timestamp = None
        self.prev_steer_output = 0.0
        self.tau = 0.2
        self.timestamp = None
        self.ego_actor = None
        self.physics_control = None
        self.current_control = carla.VehicleControl()
        self.current_turn_indicator = TurnIndicatorsCommand.DISABLE
        self.current_hazard_lights = HazardLightsCommand.DISABLE

        # Thread synchronization (protects: current_control, ego_actor, timestamp, physics_control)
        self._state_lock = threading.Lock()

        # Per-sensor publish workers keyed by sensor ID. Heavy sensor data
        # (camera, lidar) is converted and published on these threads so the
        # synchronous tick loop is never blocked by serialization or by
        # reliable-QoS flow control (see SensorPublishWorker).
        self._publish_workers = {}

        # ROS-related helpers initialized later
        self.ros2_node = None
        self.ros_publisher_manager = None
        self.clock_publisher = None
        self.spin_thread = None
        self.cv_bridge = CvBridge()

        # SplatSim state
        self.render_with_splatsim = False
        self._splatsim_cameras = []
        self._splatsim_camera_configs = []
        self._splatsim_lidars = []
        self._splatsim_lidar_configs = []
        self._geo_transform_ready = False
        self._mgrs_offset_x = 0.0
        self._mgrs_offset_y = 0.0
        self._proj_origin = (0.0, 0.0)
        self._latest_imu_accel = None

    def _ros_context_ok(self):
        return not self._shutting_down and rclpy.ok()

    def _safe_publish(self, publisher, msg):
        if not self._ros_context_ok() or publisher is None:
            return False
        try:
            publisher.publish(msg)
            return True
        except Exception as exc:
            if self._shutting_down or not rclpy.ok():
                return False
            raise exc

    def __call__(self):
        input_data = self.sensor_interface.get_data()
        timestamp = GameTime.get_time()
        control = self.run_step(input_data, timestamp)
        return control

    def get_param(self):
        return self.param_values

    def checkFrequency(self, sensor):
        """
        Return True when publication should be throttled for the sensor.

        Uses simulation time (self.timestamp) for all sensors to ensure correct throttling in
        synchronous mode. Wall-clock timing would cause issues when simulation speed differs from
        real-time.

        """
        # Use sensor registry for all sensors (including legacy ones)
        config = self.sensor_registry.get_sensor(sensor)
        if not config:
            return False

        if self.timestamp is None:
            return False

        should_publish = self.sensor_registry.should_publish(sensor, self.timestamp)
        return not should_publish

    def get_msg_header(self, frame_id, timestamp=None):
        """Obtain and modify ROS message header.

        timestamp defaults to the latest tick time; publish workers pass the
        timestamp captured when their frame was enqueued so messages are
        stamped with the frame's own tick even when published later.
        """
        header = Header()
        header.frame_id = frame_id
        if timestamp is None:
            timestamp = self.timestamp
        seconds = int(timestamp)
        nanoseconds = int((timestamp - int(timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def _submit_to_publish_worker(self, key, fn, *args):
        """Run a publish call on the sensor's worker thread (created lazily)."""
        worker = self._publish_workers.get(key)
        if worker is None:
            worker = SensorPublishWorker(key, self.logger)
            self._publish_workers[key] = worker
        worker.submit(fn, args)

    def lidar(self, carla_lidar_measurement, id_, timestamp=None):
        """Transform the received lidar measurement into a ROS point cloud message.

        Runs on the sensor's publish worker thread; frequency gating and
        registry bookkeeping happen at the dispatch site in run_step.
        """
        config = self.sensor_registry.get_sensor(id_)
        if not config:
            self.logger.warning(f"No registry entry for LiDAR sensor '{id_}'")
            return

        publisher = self.pub_lidar.get(id_)
        if publisher is None:
            self.logger.warning(f"LiDAR publisher missing for '{id_}'")
            return
        # Skip the conversion work when nothing consumes this point cloud.
        if publisher.get_subscription_count() == 0:
            return

        header = self.get_msg_header(frame_id=config.frame_id or "base_link", timestamp=timestamp)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32
        ).reshape(-1, 4)
        intensity = lidar_data[:, 3]
        intensity = (
            numpy.clip(intensity, 0, 1) * 255
        )  # CARLA lidar intensity values are between 0 and 1
        intensity = intensity.astype(numpy.uint8).reshape(-1, 1)

        return_type = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.uint8)
        channel = numpy.empty((0, 1), dtype=numpy.uint16)

        # Determine number of channels from configuration if available
        num_channels = int(config.parameters.get("channels", 32))

        for i in range(num_channels):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            channel = numpy.vstack(
                (channel, numpy.full((current_ring_points_count, 1), i, dtype=numpy.uint16))
            )

        lidar_data = numpy.hstack((lidar_data[:, :3], intensity, return_type, channel))
        lidar_data[:, 1] *= -1

        dtype = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("intensity", "u1"),
            ("return_type", "u1"),
            ("channel", "u2"),
        ]

        structured_lidar_data = numpy.zeros(lidar_data.shape[0], dtype=dtype)
        structured_lidar_data["x"] = lidar_data[:, 0]
        structured_lidar_data["y"] = lidar_data[:, 1]
        structured_lidar_data["z"] = lidar_data[:, 2]
        structured_lidar_data["intensity"] = lidar_data[:, 3].astype(numpy.uint8)
        structured_lidar_data["return_type"] = lidar_data[:, 4].astype(numpy.uint8)
        structured_lidar_data["channel"] = lidar_data[:, 5].astype(numpy.uint16)

        point_cloud_msg = create_cloud(header, fields, structured_lidar_data)
        publisher.publish(point_cloud_msg)

    def initialpose_callback(self, data):
        """Transform RVIZ initial pose to CARLA (thread-safe)."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = ros_pose_to_carla_transform(pose)

        with self._state_lock:
            if self.ego_actor is not None:
                self.ego_actor.set_transform(carla_pose_transform)
            else:
                self.logger.warning("Cannot set initial pose: ego vehicle not available")

    def pose(self):
        """Transform odometry data to Pose and publish with covariance (thread-safe)."""
        if self.checkFrequency("pose"):
            return

        # Get GNSS sensor configuration from registry (fallback to "pose" pseudo-sensor)
        gnss_config = self.sensor_registry.get_sensor("gnss") or self.sensor_registry.get_sensor(
            "pose"
        )

        if not gnss_config or not gnss_config.publisher:
            self.logger.warning(
                "GNSS/pose publisher not initialized in registry. "
                "Check sensor_mapping.yaml includes gnss_link sensor."
            )
            return

        header = self.get_msg_header(frame_id="map")
        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()

        # Thread-safe access to ego_actor
        with self._state_lock:
            if not self.ego_actor:
                return
            ego_transform = self.ego_actor.get_transform()

        pose_carla.position = carla_location_to_ros_point(ego_transform.location)
        pose_carla.orientation = carla_rotation_to_ros_quaternion(ego_transform.rotation)
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
        out_pose_with_cov.pose.covariance = self._create_gnss_covariance_matrix()

        # Publish via registry publisher
        gnss_config.publisher.publish(out_pose_with_cov)
        self.sensor_registry.update_sensor_timestamp(gnss_config.sensor_id, self.timestamp)

    def _create_gnss_covariance_matrix(self):
        """Create GNSS covariance matrix from sensor configuration."""
        cfg = self.sensor_registry.get_sensor("gnss")
        if cfg:
            cov = getattr(cfg, "covariance", {})
        else:
            cov = {}
        pos_var = cov.get("position_variance", 0.01)
        orient_var = cov.get("orientation_variance", 1.0)
        return [
            pos_var,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            pos_var,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            pos_var,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            orient_var,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            orient_var,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            orient_var,
        ]

    def _build_camera_info(self, camera_actor):
        """Build camera info message from CARLA camera actor."""
        camera_info = CameraInfo()
        camera_info.width = camera_actor.width
        camera_info.height = camera_actor.height
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(camera_actor.fov * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return camera_info

    def camera(self, carla_camera_data, cam_id, timestamp=None):
        """Handle multiple cameras with dynamic routing by sensor ID.

        Runs on the sensor's publish worker thread; frequency gating and
        registry bookkeeping happen at the dispatch site in run_step.
        """
        config = self.sensor_registry.get_sensor(cam_id)
        if not config:
            self.logger.warning(f"No registry entry for camera '{cam_id}'")
            return

        # Converting and publishing a multi-megabyte image is expensive even
        # off the tick thread (GIL pressure from several workers starves it).
        # Skip the work entirely when nothing consumes this camera; checked
        # per frame so late subscribers start receiving immediately.
        img_pub = self.pub_camera.get(cam_id)
        info_pub = self.pub_camera_info.get(cam_id)
        has_image_subs = img_pub is not None and img_pub.get_subscription_count() > 0
        has_info_subs = info_pub is not None and info_pub.get_subscription_count() > 0
        if not has_image_subs and not has_info_subs:
            return

        if cam_id not in self.camera_info_cache:
            self.camera_info_cache[cam_id] = self._build_camera_info(carla_camera_data)

        header = self.get_msg_header(
            frame_id=config.frame_id or f"{cam_id}/camera_optical_link", timestamp=timestamp
        )

        # Publish camera info
        if has_info_subs:
            cam_info = self.camera_info_cache[cam_id]
            cam_info.header = header
            info_pub.publish(cam_info)

        # Publish image
        if has_image_subs:
            image_array = numpy.ndarray(
                shape=(carla_camera_data.height, carla_camera_data.width, 4),
                dtype=numpy.uint8,
                buffer=carla_camera_data.raw_data,
            )
            img_msg = self.cv_bridge.cv2_to_imgmsg(image_array, encoding="bgra8")
            img_msg.header = header
            img_pub.publish(img_msg)

    def imu(self, carla_imu_measurement):
        """Transform and publish IMU measurement to ROS."""
        if self.checkFrequency("imu"):
            return

        config = self.sensor_registry.get_sensor("imu")
        if not config:
            self.logger.warning("No registry entry for IMU sensor")
            return

        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(frame_id=config.frame_id or "imu_link")
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll = math.radians(carla_imu_measurement.transform.rotation.roll)
        pitch = -math.radians(carla_imu_measurement.transform.rotation.pitch)
        yaw = -math.radians(carla_imu_measurement.transform.rotation.yaw)

        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        if self.pub_imu:
            self.pub_imu.publish(imu_msg)
            self.sensor_registry.update_sensor_timestamp("imu", self.timestamp)
        else:
            self.logger.warning("IMU publisher not initialized")

        # Cache acceleration for splatsim localization publishing
        if self.render_with_splatsim:
            self._latest_imu_accel = (
                imu_msg.linear_acceleration.x,
                imu_msg.linear_acceleration.y,
                imu_msg.linear_acceleration.z,
            )

    def first_order_steering(self, steer_input):
        """
        First order steering model.

        Gracefully handles:
        - Early control commands before first simulation tick (returns raw input)
        - Multiple commands in same CARLA tick (preserves filter state, no zero spike)

        """
        # Guard against control commands arriving before first sensor callback
        if self.timestamp is None:
            return steer_input  # No filtering yet, return raw command

        # Initialize on first call
        if self.prev_timestamp is None:
            self.prev_timestamp = self.timestamp
            self.prev_steer_output = steer_input
            return steer_input

        dt = self.timestamp - self.prev_timestamp

        # Multiple commands in same simulation tick (dt = 0)
        # Preserve filter state to avoid zero spike - return previous output
        if dt <= 0.0:
            return self.prev_steer_output

        # Normal case: time has advanced, apply low-pass filter
        steer_output = self.prev_steer_output + (steer_input - self.prev_steer_output) * (
            dt / (self.tau + dt)
        )
        self.prev_steer_output = steer_output
        self.prev_timestamp = self.timestamp
        return steer_output

    def control_callback(self, in_cmd):
        """
        Convert and publish CARLA Ego Vehicle Control to AUTOWARE.

        Thread-safe: Acquires state lock when accessing shared vehicle state.

        """
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd

        with self._state_lock:
            # convert base on steer curve of the vehicle
            if not self.physics_control or not self.ego_actor:
                return  # Skip if vehicle not initialized yet

            steer_curve = self.physics_control.steering_curve
            current_vel = self.ego_actor.get_velocity()
            max_steer_ratio = numpy.interp(
                abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
            )
            out_cmd.steer = self.first_order_steering(-in_cmd.actuation.steer_cmd) * max_steer_ratio
            out_cmd.brake = in_cmd.actuation.brake_cmd
            self.current_control = out_cmd

    def turn_indicators_callback(self, in_cmd):
        """Store turn indicator command (thread-safe)."""
        with self._state_lock:
            self.current_turn_indicator = in_cmd.command

    def hazard_lights_callback(self, in_cmd):
        """Store hazard lights command (thread-safe)."""
        with self._state_lock:
            self.current_hazard_lights = in_cmd.command

    def apply_light_state(self):
        """
        Apply turn indicator and hazard lights commands to CARLA ego vehicle.

        Hazard takes priority over turn indicator. Other light bits (brake,
        reverse, headlights, etc.) are preserved so we do not interfere with
        anything CARLA or another module manages.

        """
        with self._state_lock:
            if not self.ego_actor:
                return
            turn_cmd = self.current_turn_indicator
            hazard_cmd = self.current_hazard_lights
            current_state = int(self.ego_actor.get_light_state())

            left_bit = int(carla.VehicleLightState.LeftBlinker)
            right_bit = int(carla.VehicleLightState.RightBlinker)

            new_state = current_state & ~left_bit & ~right_bit
            if hazard_cmd == HazardLightsCommand.ENABLE:
                new_state |= left_bit | right_bit
            elif turn_cmd == TurnIndicatorsCommand.ENABLE_LEFT:
                new_state |= left_bit
            elif turn_cmd == TurnIndicatorsCommand.ENABLE_RIGHT:
                new_state |= right_bit

            self.ego_actor.set_light_state(carla.VehicleLightState(new_state))

    def ego_status(self):
        """
        Publish ego vehicle status.

        Thread-safe: Acquires state lock when accessing ego_actor.

        """
        if self.checkFrequency("status"):
            return

        # Thread-safe access to ego_actor - get all needed data in one lock section
        with self._state_lock:
            if not self.ego_actor:
                return

            ego_transform = self.ego_actor.get_transform()
            ego_velocity_carla = self.ego_actor.get_velocity()
            ego_angular_velocity = self.ego_actor.get_angular_velocity()
            steer_angle = self.ego_actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            control = self.ego_actor.get_control()
            light_state = int(self.ego_actor.get_light_state())

        # convert velocity from cartesian to ego frame
        trans_mat = numpy.array(ego_transform.get_matrix()).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array(
            [ego_velocity_carla.x, ego_velocity_carla.y, ego_velocity_carla.z]
        ).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]

        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_actuation_status = ActuationStatusStamped()

        out_vel_state.header = self.get_msg_header(frame_id="base_link")
        out_vel_state.longitudinal_velocity = ego_velocity[0]
        out_vel_state.lateral_velocity = ego_velocity[1]
        out_vel_state.heading_rate = ego_transform.transform_vector(ego_angular_velocity).z

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -math.radians(steer_angle)

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        out_actuation_status.header = self.get_msg_header(frame_id="base_link")
        out_actuation_status.status.accel_status = control.throttle
        out_actuation_status.status.brake_status = control.brake
        out_actuation_status.status.steer_status = -control.steer

        # Decode CARLA blinker bits into Autoware turn-indicator / hazard reports.
        left_on = bool(light_state & int(carla.VehicleLightState.LeftBlinker))
        right_on = bool(light_state & int(carla.VehicleLightState.RightBlinker))

        out_turn_indicators_state = TurnIndicatorsReport()
        out_turn_indicators_state.stamp = out_vel_state.header.stamp
        if left_on and right_on:
            # Both blinkers on => hazard mode; turn indicator reports DISABLE.
            out_turn_indicators_state.report = TurnIndicatorsReport.DISABLE
        elif left_on:
            out_turn_indicators_state.report = TurnIndicatorsReport.ENABLE_LEFT
        elif right_on:
            out_turn_indicators_state.report = TurnIndicatorsReport.ENABLE_RIGHT
        else:
            out_turn_indicators_state.report = TurnIndicatorsReport.DISABLE

        out_hazard_lights_state = HazardLightsReport()
        out_hazard_lights_state.stamp = out_vel_state.header.stamp
        if left_on and right_on:
            out_hazard_lights_state.report = HazardLightsReport.ENABLE
        else:
            out_hazard_lights_state.report = HazardLightsReport.DISABLE

        self.pub_actuation_status.publish(out_actuation_status)
        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)
        self.pub_turn_indicators_state.publish(out_turn_indicators_state)
        self.pub_hazard_lights_state.publish(out_hazard_lights_state)
        self.sensor_registry.update_sensor_timestamp("status", self.timestamp)

    def run_step(self, input_data, timestamp):
        """
        Execute main simulation step for publishing sensor data and getting control commands.

        Thread-safe: Acquires state lock when writing timestamp and reading current_control.
        The timestamp must be protected because control_callback reads it (via
        first_order_steering) to calculate dt. Without protection, the ROS callback could
        see a partially-updated or future timestamp, yielding negative/zero dt and unstable
        steering.

        Args
        ----
            input_data: Dictionary of sensor data from CARLA
            timestamp: Current simulation timestamp

        Returns
        -------
            carla.VehicleControl: Current control command for the vehicle


        """
        # Update timestamp under lock to prevent race with control_callback
        with self._state_lock:
            self.timestamp = timestamp

        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - seconds) * 1e9)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            self._dispatch_sensor(key, data)

        # SplatSim: send ego pose to splatsim cameras and publish dummy data
        if self.render_with_splatsim:
            self._splatsim_tick(seconds, nanoseconds)

        # Push turn indicator / hazard lights to CARLA before reading status back.
        self.apply_light_state()

        # Publish ego vehicle status
        self.ego_status()

        # Thread-safe read of current control command
        with self._state_lock:
            return self.current_control

    def _dispatch_sensor(self, key, data) -> None:
        """Publish one CARLA sensor sample according to its registered type.

        Camera and lidar conversion/publishing run on per-sensor worker threads:
        publishing multi-megabyte messages inline (reliable-QoS camera images in
        particular block on DDS flow control) would stall run_step and slow
        simulation time itself. Frequency gating and registry bookkeeping stay on
        the caller's thread so the registry is never accessed concurrently.
        """
        sensor_type = self.id_to_sensor_type_map.get(key)
        if not sensor_type:
            self.logger.warning(
                f"Unknown sensor ID '{key}' received from CARLA - skipping. "
                f"This may indicate a sensor configuration mismatch."
            )
            return

        if sensor_type == "sensor.camera.rgb":
            if not self.checkFrequency(key):
                self.sensor_registry.update_sensor_timestamp(key, self.timestamp)
                self._submit_to_publish_worker(key, self.camera, data[1], key, self.timestamp)
        elif sensor_type == "sensor.other.gnss":
            # Skip GNSS pose when splatsim provides localization directly
            if not self.render_with_splatsim:
                self.pose()
        elif sensor_type == "sensor.lidar.ray_cast":
            if not self.checkFrequency(key):
                self.sensor_registry.update_sensor_timestamp(key, self.timestamp)
                self._submit_to_publish_worker(key, self.lidar, data[1], key, self.timestamp)
        elif sensor_type == "sensor.other.imu":
            self.imu(data[1])
        else:
            self.logger.debug(f"No publisher for sensor '{key}' (type={sensor_type})")

    # ── SplatSim integration methods ──────────────────────────────────────

    def _initialize_splatsim_publishers(self):
        """Create publishers for dummy perception and localization (splatsim mode)."""
        self.pub_empty_objects = self.ros2_node.create_publisher(
            PredictedObjects, "/perception/object_recognition/objects", 1
        )
        self.pub_empty_pointcloud = self.ros2_node.create_publisher(
            PointCloud2, "/perception/obstacle_segmentation/pointcloud", 1
        )
        self.pub_empty_traffic_signals = self.ros2_node.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            1,
        )
        self.pub_empty_occupancy_grid = self.ros2_node.create_publisher(
            OccupancyGrid, "/perception/occupancy_grid_map/map", 1
        )
        self.pub_tf = self.ros2_node.create_publisher(TFMessage, "/tf", 10)
        self.pub_localization_odom = self.ros2_node.create_publisher(
            Odometry, "/localization/kinematic_state", 10
        )
        self.pub_localization_pose = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose_estimator/pose_with_covariance",
            10,
        )
        self.pub_localization_accel = self.ros2_node.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 10
        )
        loc_init_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_localization_init_state = self.ros2_node.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            loc_init_qos,
        )
        self.pub_fusion_pose = self.ros2_node.create_publisher(
            PoseStamped, "/localization/pose_twist_fusion_filter/pose", 10
        )
        self.pub_initialpose3d = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", 10
        )

    def _init_geo_transform(self):
        """Resolve the splatsim geographic origin and its MGRS offset.

        The origin (lat/lon of the CARLA world origin) is read from the splatsim
        ``.usdz`` scene: its ``world.ecef_anchor`` records the ENU->ECEF anchor
        the 3DGS scene is aligned to, so the geographic origin comes straight
        from the same bundle splatsim renders — with no external lanelet2 map or
        OpenDRIVE GeoReference.
        """
        if self._geo_transform_ready:
            return
        from autoware_lanelet2_extension_python.projection import MGRSProjector
        import lanelet2.core
        import lanelet2.io

        tileset_path = str(self.param_values.get("splatsim_tileset_path", "") or "").strip()
        if not tileset_path:
            raise RuntimeError(
                "splatsim_tileset_path is required to derive the geographic origin "
                "from the scene's world.ecef_anchor."
            )
        self._proj_origin = _origin_from_usdz(tileset_path)
        self.logger.info(
            f"GeoTransform origin from splatsim scene '{tileset_path}': "
            f"origin=({self._proj_origin[0]:.8f}, {self._proj_origin[1]:.8f})"
        )
        lat_0, lon_0 = self._proj_origin

        projector = MGRSProjector(lanelet2.io.Origin(lat_0, lon_0))
        origin_gps = lanelet2.core.GPSPoint(lat_0, lon_0, 0.0)
        origin_local = projector.forward(origin_gps)
        self._mgrs_offset_x = origin_local.x
        self._mgrs_offset_y = origin_local.y

        self._geo_transform_ready = True
        self.logger.info(
            f"GeoTransform initialized: lat_0={lat_0:.8f}, lon_0={lon_0:.8f}, "
            f"mgrs_offset=({self._mgrs_offset_x:.1f}, {self._mgrs_offset_y:.1f})"
        )

    def init_splatsim_cameras(self):
        """Create SplatSimRGBCamera instances for each camera sensor.

        Must be called after the CARLA world is loaded (ego_actor is set).
        """
        if not self.render_with_splatsim or not self._splatsim_camera_configs:
            return
        if not self.param_values.get("splatsim_tileset_path"):
            self.logger.warning(
                "render_with_splatsim is true but splatsim_tileset_path is empty; "
                "skipping splatsim camera initialization"
            )
            return

        self._init_geo_transform()

        from .splatsim.splatsim_camera import SplatSimRGBCamera

        base_grpc_port = self.param_values["splatsim_grpc_port"]
        for cam_idx, cfg in enumerate(self._splatsim_camera_configs):
            cam = SplatSimRGBCamera(
                self._make_camera_spec(cfg),
                proj_origin=self._proj_origin,
                config=self._make_camera_config(cfg, base_grpc_port + cam_idx),
            )
            self._splatsim_cameras.append(cam)
            self.logger.info(f"SplatSimRGBCamera created for sensor '{cfg.sensor_id}'")

    @staticmethod
    def _make_camera_spec(cfg) -> dict:
        """Build the sensor-spec dict that SplatSimRGBCamera expects."""
        return {
            "id": cfg.sensor_id,
            "image_size_x": cfg.parameters.get("image_size_x", 1600),
            "image_size_y": cfg.parameters.get("image_size_y", 900),
            "fov": cfg.parameters.get("fov", 70.0),
            "spawn_point": cfg.transform
            or {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

    def _make_camera_config(self, cfg, grpc_port):
        """Build the SplatSimCameraConfig for one camera sensor.

        Global settings come from node params; per-sensor topics/optics/rate come
        from the sensor mapping (``cfg`` / ``cfg.parameters``), falling back to the
        SplatSimCameraConfig dataclass defaults when unset.
        """
        from .splatsim.splatsim_camera import SplatSimCameraConfig

        p = self.param_values
        params = cfg.parameters or {}
        kwargs = {
            "tileset_path": p["splatsim_tileset_path"],
            "splatsim_image": p["splatsim_image"],
            "grpc_port": grpc_port,
            "use_sh": p["splatsim_use_sh"],
            "device": p["splatsim_device"],
            "restart_container": p["splatsim_restart_container"],
            "compress_format": p.get("splatsim_compress_format", ""),
            "frame_rate": params.get("frame_rate", cfg.frequency_hz),
        }
        if cfg.topic_image:
            kwargs["image_topic"] = cfg.topic_image
        if cfg.topic_info:
            kwargs["camera_info_topic"] = cfg.topic_info
        if cfg.frame_id:
            kwargs["frame_id"] = cfg.frame_id
        for key in ("near_plane", "far_plane"):
            if key in params:
                kwargs[key] = params[key]
        return SplatSimCameraConfig(**kwargs)

    def init_splatsim_lidars(self):
        """Create SplatSimLidar instances for each LiDAR sensor.

        Must be called after the CARLA world is loaded (ego_actor is set).
        The rendered PointCloud2 is published on the sensor's sensing topic so
        the existing preprocessing pipeline (and OnePlanner) runs unchanged.
        """
        if not self.render_with_splatsim or not self._splatsim_lidar_configs:
            return
        if not self.param_values.get("splatsim_tileset_path"):
            self.logger.warning(
                "render_with_splatsim is true but splatsim_tileset_path is empty; "
                "skipping splatsim lidar initialization"
            )
            return

        self._init_geo_transform()

        from .splatsim.splatsim_lidar import SplatSimLidar

        base_grpc_port = self.param_values["splatsim_lidar_grpc_port"]
        for lidar_idx, cfg in enumerate(self._splatsim_lidar_configs):
            lidar = SplatSimLidar(
                self._make_lidar_spec(cfg),
                proj_origin=self._proj_origin,
                config=self._make_lidar_config(cfg, base_grpc_port + lidar_idx),
            )
            self._splatsim_lidars.append(lidar)
            self.logger.info(f"SplatSimLidar created for sensor '{cfg.sensor_id}'")

    @staticmethod
    def _make_lidar_spec(cfg) -> dict:
        """Build the sensor-spec dict that SplatSimLidar expects."""
        return {
            "id": cfg.sensor_id,
            "spawn_point": cfg.transform
            or {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

    def _make_lidar_config(self, cfg, grpc_port):
        """Build the SplatSimLidarConfig for one LiDAR sensor.

        Global settings come from node params; per-sensor rendering settings come
        from the sensor mapping (``cfg`` / ``cfg.parameters``), falling back to the
        SplatSimLidarConfig dataclass defaults when unset.
        """
        from .splatsim.splatsim_lidar import SplatSimLidarConfig

        p = self.param_values
        params = cfg.parameters or {}
        kwargs = {
            "tileset_path": p["splatsim_tileset_path"],
            "splatsim_image": p["splatsim_image"],
            "grpc_port": grpc_port,
            "use_sh": p["splatsim_use_sh"],
            "device": p["splatsim_device"],
            "restart_container": p["splatsim_restart_container"],
            "fps": params.get("fps", cfg.frequency_hz),
        }
        if cfg.topic:
            kwargs["pointcloud_topic"] = cfg.topic
        if cfg.frame_id:
            kwargs["frame_id"] = cfg.frame_id
        for key in (
            "sensor_type",
            "n_rows",
            "n_columns",
            "min_range_m",
            "drop_threshold",
            "alpha_threshold",
        ):
            if key in params:
                kwargs[key] = params[key]
        # max range: explicit splatsim key wins, else the CARLA sensor mapping range.
        if "max_range_m" in params:
            kwargs["max_range_m"] = params["max_range_m"]
        elif "range" in params:
            kwargs["max_range_m"] = float(params["range"])
        if "elevation_deg" in params:
            kwargs["elevation_deg"] = tuple(params["elevation_deg"])
        return SplatSimLidarConfig(**kwargs)

    def _splatsim_tick(self, seconds, nanoseconds):
        """Per-tick splatsim work: send poses, publish dummy perception/localization."""
        with self._state_lock:
            actor_matrix = (
                self.ego_actor.get_transform().get_matrix() if self.ego_actor is not None else None
            )
        if actor_matrix is not None:
            self._send_splatsim_poses(actor_matrix, seconds, nanoseconds)

        self._publish_dummy_perception()

        with self._state_lock:
            has_ego = self.ego_actor is not None
        if has_ego:
            self._publish_localization()

    def _send_splatsim_poses(self, actor_matrix, seconds, nanoseconds) -> None:
        """Forward the ego pose to every splatsim camera and LiDAR container."""
        for sensor in (*self._splatsim_cameras, *self._splatsim_lidars):
            sensor.update(
                actor_matrix_4x4=actor_matrix,
                stamp_sec=seconds,
                stamp_nanosec=nanoseconds,
            )

    def _publish_dummy_perception(self) -> None:
        """Publish empty perception outputs so downstream nodes keep ticking."""
        header = self.get_msg_header(frame_id="map")

        empty_objects = PredictedObjects()
        empty_objects.header = header
        self.pub_empty_objects.publish(empty_objects)

        empty_pc = PointCloud2()
        empty_pc.header = self.get_msg_header(frame_id="base_link")
        empty_pc.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]
        empty_pc.point_step = 16
        empty_pc.height = 1
        empty_pc.width = 0
        empty_pc.row_step = 0
        empty_pc.is_dense = True
        self.pub_empty_pointcloud.publish(empty_pc)

        empty_tl = TrafficLightGroupArray()
        empty_tl.stamp = header.stamp
        self.pub_empty_traffic_signals.publish(empty_tl)

        empty_og = OccupancyGrid()
        empty_og.header = self.get_msg_header(frame_id="map")
        empty_og.info.resolution = 0.5
        empty_og.info.width = 0
        empty_og.info.height = 0
        self.pub_empty_occupancy_grid.publish(empty_og)

    def _publish_localization(self):
        """Publish localization data from CARLA ground truth (splatsim mode).

        Position: CARLA → xodr (flip y) → MGRS absolute (add offset)
        Orientation: 2D yaw only (CARLA CW+ → ROS CCW+)
        """
        header = self.get_msg_header(frame_id="map")

        with self._state_lock:
            carla_tf = self.ego_actor.get_transform()
            ego_vel = self.ego_actor.get_velocity()
            ego_ang_vel = self.ego_actor.get_angular_velocity()
            trans_mat = numpy.array(carla_tf.get_matrix()).reshape(4, 4)

        # Position
        pose = Pose()
        pose.position.x = carla_tf.location.x + self._mgrs_offset_x
        pose.position.y = -carla_tf.location.y + self._mgrs_offset_y
        pose.position.z = carla_tf.location.z

        # Orientation (2D yaw only)
        yaw = -math.radians(carla_tf.rotation.yaw)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # /tf (map → base_link)
        tf_stamped = TransformStamped()
        tf_stamped.header = header
        tf_stamped.child_frame_id = "base_link"
        tf_stamped.transform.translation.x = pose.position.x
        tf_stamped.transform.translation.y = pose.position.y
        tf_stamped.transform.translation.z = pose.position.z
        tf_stamped.transform.rotation = pose.orientation
        self.pub_tf.publish(TFMessage(transforms=[tf_stamped]))

        # /localization/kinematic_state (Odometry)
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array([ego_vel.x, ego_vel.y, ego_vel.z]).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]
        odom.twist.twist.linear.x = float(ego_velocity[0])
        odom.twist.twist.linear.y = float(-ego_velocity[1])
        odom.twist.twist.linear.z = float(ego_velocity[2])
        odom.twist.twist.angular.z = -math.radians(ego_ang_vel.z)
        self.pub_localization_odom.publish(odom)

        # Pose with covariance
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = header
        pose_cov.pose.pose = pose
        pcov = pose_cov.pose.covariance
        pcov[0] = pcov[7] = pcov[14] = pcov[21] = pcov[28] = pcov[35] = 0.0001
        self.pub_localization_pose.publish(pose_cov)

        # Acceleration
        accel_msg = AccelWithCovarianceStamped()
        accel_msg.header = self.get_msg_header(frame_id="base_link")
        if self._latest_imu_accel is not None:
            accel_msg.accel.accel.linear.x = self._latest_imu_accel[0]
            accel_msg.accel.accel.linear.y = self._latest_imu_accel[1]
            accel_msg.accel.accel.linear.z = self._latest_imu_accel[2]
        self.pub_localization_accel.publish(accel_msg)

        # Localization initialization state
        init_state = LocalizationInitializationState()
        init_state.stamp = header.stamp
        init_state.state = LocalizationInitializationState.INITIALIZED
        self.pub_localization_init_state.publish(init_state)

        # Satisfy topic_state_monitors
        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose
        self.pub_fusion_pose.publish(pose_stamped)
        self.pub_initialpose3d.publish(pose_cov)

    # ── Shutdown ──────────────────────────────────────────────────────────

    def shutdown(self):
        """
        Clean shutdown of ROS node and spin thread.

        Properly destroys publishers, stops the spin thread, and shuts down rclpy to prevent
        process hanging and publisher leaks.

        """
        self._stop_publish_workers()
        self._shutdown_splatsim_sensors()
        self._shutdown_ros()

    def _stop_publish_workers(self) -> None:
        """Signal and stop per-sensor publish worker threads before destroying publishers."""
        # Worker loops poll self._shutting_down; stop them before destroying the
        # publishers they use, avoiding hangs and leaks.
        self._shutting_down = True
        for worker in self._publish_workers.values():
            worker.stop()
        self._publish_workers.clear()

    def _shutdown_splatsim_sensors(self) -> None:
        """Shut down and clear all splatsim camera and LiDAR containers."""
        for cam in self._splatsim_cameras:
            cam.shutdown()
        self._splatsim_cameras.clear()
        for lidar in self._splatsim_lidars:
            lidar.shutdown()
        self._splatsim_lidars.clear()

    def _shutdown_ros(self) -> None:
        """Destroy publishers/node, join the spin thread, and shut down rclpy."""
        if self.ros_publisher_manager:
            self.ros_publisher_manager.destroy_all_publishers()

        # Destroy node (this will stop rclpy.spin in the thread)
        if self.ros2_node:
            self.ros2_node.destroy_node()

        # Wait for spin thread to finish (with timeout to prevent hanging)
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)
            if self.spin_thread.is_alive():
                self.logger.warning("Spin thread did not terminate within timeout")

        # Shutdown rclpy context
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # rclpy.shutdown() can raise if already shut down
            self.logger.debug(f"rclpy shutdown raised: {e}")
