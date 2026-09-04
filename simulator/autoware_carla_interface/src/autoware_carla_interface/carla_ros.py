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
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
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
from .modules.carla_utils import project_point_to_ground
from .modules.carla_utils import ros_pose_to_carla_transform
from .modules.carla_wrapper import SensorInterface


def _parse_geo_reference(xodr_xml: str):
    """Extract ``(lat_0, lon_0)`` from the OpenDRIVE ``<geoReference>`` PROJ string."""
    import re
    import xml.etree.ElementTree as ET

    match = re.search(
        r"<geoReference>\s*(?:<!\[CDATA\[)?(.*?)(?:\]\]>)?\s*</geoReference>",
        xodr_xml,
        re.DOTALL,
    )
    if match and match.group(1).strip():
        proj_string = match.group(1).strip()
    else:
        root = ET.fromstring(xodr_xml)
        geo_ref = root.find(".//geoReference")
        if geo_ref is None or not geo_ref.text:
            raise ValueError("No <geoReference> found in OpenDRIVE XML")
        proj_string = geo_ref.text.strip()

    lat_match = re.search(r"\+lat_0=([0-9eE.+-]+)", proj_string)
    lon_match = re.search(r"\+lon_0=([0-9eE.+-]+)", proj_string)
    if lat_match is None or lon_match is None:
        raise ValueError(f"Cannot extract +lat_0/+lon_0 from geoReference: {proj_string}")
    return float(lat_match.group(1)), float(lon_match.group(1))


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
            "spawn_point_ground_snap": (rclpy.Parameter.Type.BOOL, False),
            "spawn_point_ground_offset_z": (rclpy.Parameter.Type.DOUBLE, 0.5),
            "initial_pose_ground_offset_z": (rclpy.Parameter.Type.DOUBLE, 1.0),
            "force_load_world": (rclpy.Parameter.Type.BOOL, False),
            # Minimum throttle applied while accelerating from (near) standstill.
            # Heavy CARLA vehicles (e.g. vehicle.taxi.ford) do not creep and
            # never start moving on the small throttle the actuation map yields
            # at low target accelerations.
            "min_positive_throttle": (rclpy.Parameter.Type.DOUBLE, 0.0),
            "min_positive_throttle_speed_threshold": (rclpy.Parameter.Type.DOUBLE, 0.8),
            "no_rendering_mode": (rclpy.Parameter.Type.BOOL, False),
            # Publish the CARLA ground-truth localization (kinematic_state and
            # the map->base_link TF) directly from the ego transform. Used by
            # the E2E planning setup instead of the former carla_state_publisher
            # GNSS round-trip, which duplicated these topics.
            "publish_ground_truth_localization": (rclpy.Parameter.Type.BOOL, False),
            "map_origin_x": (rclpy.Parameter.Type.DOUBLE, 0.0),
            "map_origin_y": (rclpy.Parameter.Type.DOUBLE, 0.0),
            # Sensor configuration parameters
            "sensor_kit_name": (rclpy.Parameter.Type.STRING, ""),  # Empty = use YAML default
            "sensor_mapping_file": (rclpy.Parameter.Type.STRING, ""),
            # Replace the ego vehicle's speed-based steering curve with an
            # identity curve (workaround for the corrupt curve data CARLA 0.10
            # returns, which attenuates steering at driving speeds).
            "flatten_steering_curve": (rclpy.Parameter.Type.BOOL, False),
            # Nudge the ego physics body awake when launching from a standstill.
            # Only needed on CARLA 0.10 (UE5/Chaos), where a stationary body is
            # put to sleep and VehicleControl throttle does not wake it. Off by
            # default so the supported 0.9.15 environment, where bodies never
            # sleep, keeps its unmodified launch dynamics.
            "wake_sleeping_physics": (rclpy.Parameter.Type.BOOL, False),
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
        if self.param_values.get("publish_ground_truth_localization", False):
            self.pub_gt_tf = self.ros2_node.create_publisher(TFMessage, "/tf", 10)
            self.pub_gt_odom = self.ros2_node.create_publisher(
                Odometry, "/localization/kinematic_state", 10
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

        self._register_sensor_configs(self.sensor_configs)
        self._create_sensor_publishers_from_registry()
        self.sensors = {"sensors": self._build_sensor_specs(self.sensor_configs)}

        self.logger.info(f"Configured {len(self.sensor_configs)} sensors from mapping")

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
        self._setup_tf_listener()
        self._initialize_clock_publisher()

        self._load_sensor_configuration()

        # Initialize publishers and subscriptions
        self._initialize_subscriptions()
        self._initialize_status_publishers()

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
        self._max_steer_angle_rad = None
        self.timestamp = None
        self.ego_actor = None
        self.physics_control = None
        # Map origin (CARLA->map offset) is resolved once the world/map is
        # loaded (on_world_ready); None until then. An initialpose that arrives
        # before that is buffered here and applied on_world_ready.
        self._map_origin = None
        self._pending_initialpose = None
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

    def _project_initialpose_to_ground(self, carla_pose_transform):
        """Return the CARLA ground height under the pose, or None to skip snapping.

        Returns None when spawn_point_ground_snap is disabled, or when no ground
        height can be found (older CARLA APIs without ``ground_projection``, or
        no ground hit), so callers fall back to the fixed z-offset.
        """
        if not self.param_values["spawn_point_ground_snap"]:
            return None

        return project_point_to_ground(
            CarlaDataProvider.get_world(),
            carla_pose_transform.location.x,
            carla_pose_transform.location.y,
        )

    def _current_map_origin(self):
        """Return the resolved CARLA→map offset, or (0, 0) if not yet resolved."""
        return self._map_origin if self._map_origin is not None else (0.0, 0.0)

    def _derive_map_origin(self):
        """Compute the CARLA→map-frame origin offset from the loaded map.

        Must only be called once the CARLA world/map is fully loaded (see
        :meth:`on_world_ready`); it reads ``get_map()`` directly with no
        readiness guards, since resolving against a not-yet-loaded (e.g. default)
        map would silently latch the wrong origin.

        An explicit non-zero ``map_origin_x/y`` parameter always wins.  When the
        parameters are left at 0/0 and the CARLA map carries a georeferenced
        OpenDRIVE ``<geoReference>`` (+lat_0/+lon_0 other than 0/0, e.g. maps
        converted from lanelet2), derive the offset as the origin's in-cell MGRS
        coordinates.  Hand-maintained constants for such maps can silently
        disagree with the geoReference by sub-metre amounts, shifting the GNSS
        pose and RViz initialpose against everything else that derives its
        offset from the map itself.  Stock CARLA towns (no usable geoReference)
        keep the plain 0/0 behavior.
        """
        px = float(self.param_values["map_origin_x"])
        py = float(self.param_values["map_origin_y"])
        if px != 0.0 or py != 0.0:
            return px, py
        try:
            xodr_xml = CarlaDataProvider.get_world().get_map().to_opendrive()
            lat_0, lon_0 = _parse_geo_reference(xodr_xml)
        except (RuntimeError, ValueError):
            return 0.0, 0.0
        if lat_0 == 0.0 and lon_0 == 0.0:
            return 0.0, 0.0
        from autoware_lanelet2_extension_python.projection import MGRSProjector
        import lanelet2.core
        import lanelet2.io

        projector = MGRSProjector(lanelet2.io.Origin(lat_0, lon_0))
        local = projector.forward(lanelet2.core.GPSPoint(lat_0, lon_0, 0.0))
        self.logger.info(
            f"map origin derived from OpenDRIVE geoReference: "
            f"lat_0={lat_0:.8f}, lon_0={lon_0:.8f} -> "
            f"offset=({local.x:.3f}, {local.y:.3f})"
        )
        return float(local.x), float(local.y)

    def on_world_ready(self):
        """Resolve the map origin once and flush any buffered initial pose.

        Called by the orchestrator after the CARLA world/map is fully loaded and
        the ego has been spawned.  Resolving here rather than lazily in the
        callbacks means the origin always derives from the final map, and an
        initialpose that arrived during startup is applied now instead of being
        transformed with a not-yet-known origin.
        """
        origin = self._derive_map_origin()  # reads CARLA; do it outside the lock
        with self._state_lock:
            self._map_origin = origin
            pending = self._pending_initialpose
            self._pending_initialpose = None
        self.logger.info(f"map origin resolved: ({origin[0]:.3f}, {origin[1]:.3f})")
        if pending is not None:
            self.logger.info("Applying the initial pose buffered during startup")
            self._apply_initialpose(pending)

    def initialpose_callback(self, data):
        """Buffer or apply an RViz initial pose (thread-safe).

        A map-frame pose can only be converted to CARLA once the map origin is
        known.  If it is not resolved yet (on_world_ready has not run), buffer
        the latest pose and apply it then; otherwise apply immediately.
        """
        with self._state_lock:
            ready = self._map_origin is not None
            if not ready:
                self._pending_initialpose = data
        if not ready:
            self.logger.info("Buffered initial pose until the CARLA world/map is ready")
            return
        self._apply_initialpose(data)

    def _apply_initialpose(self, data):
        """Convert a map-frame initial pose to CARLA and teleport the ego."""
        pose = data.pose.pose
        origin_x, origin_y = self._current_map_origin()
        carla_pose_transform = ros_pose_to_carla_transform(
            pose,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        # RViz's 2D Pose Estimate only carries x/y/yaw (z is always 0), so the
        # map-frame z is meaningless here. When spawn_point_ground_snap is
        # enabled and CARLA exposes ground_projection, snap onto the map
        # geometry; otherwise fall back to the fixed +2.0 z-offset that has
        # always been applied.
        ground_z = self._project_initialpose_to_ground(carla_pose_transform)
        if ground_z is not None:
            carla_pose_transform.location.z = (
                ground_z + self.param_values["initial_pose_ground_offset_z"]
            )
            self.logger.info(
                "Ground-snapped initial pose: "
                f"ground_z={ground_z:.3f}, "
                f"offset_z={self.param_values['initial_pose_ground_offset_z']:.3f}, "
                f"pose_z={carla_pose_transform.location.z:.3f}"
            )
        else:
            carla_pose_transform.location.z += 2.0

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

        origin_x, origin_y = self._current_map_origin()
        pose_carla.position = carla_location_to_ros_point(
            ego_transform.location,
            origin_x=origin_x,
            origin_y=origin_y,
        )
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
            img_msg = self._build_image_msg(carla_camera_data, config.image_encoding)
            img_msg.header = header
            img_pub.publish(img_msg)

    def _build_image_msg(self, carla_camera_data, encoding):
        """Convert a CARLA camera frame into a ROS image message.

        CARLA renders BGRA. Converting to mono8 here rather than in the
        consumer keeps three of every four bytes off the wire, and a consumer
        that wants luminance was going to do this conversion anyway.
        """
        image_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        if encoding == "mono8":
            image_array = cv2.cvtColor(image_array, cv2.COLOR_BGRA2GRAY)
        return self.cv_bridge.cv2_to_imgmsg(image_array, encoding=encoding)

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

    def _ego_speed_mps(self):
        """Return the ego speed in m/s (needs _state_lock held)."""
        velocity = self.ego_actor.get_velocity()
        return math.sqrt(
            velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
        )

    def _apply_min_positive_throttle(self, out_cmd, in_cmd):
        """Enforce the standstill throttle floor on out_cmd (needs _state_lock held).

        Heavy CARLA vehicles do not creep and never start moving on the small
        throttle the actuation map yields at low target accelerations, so
        while accelerating from (near) standstill the commanded throttle is
        raised to at least min_positive_throttle. Disabled by default (0.0).
        """
        min_positive_throttle = self.param_values.get("min_positive_throttle", 0.0)
        if min_positive_throttle <= 0.0 or out_cmd.throttle <= 0.0:
            return
        if in_cmd.actuation.brake_cmd > 0.0:
            return
        speed_threshold = self.param_values.get("min_positive_throttle_speed_threshold", 0.8)
        if speed_threshold >= 0.0 and self._ego_speed_mps() > speed_threshold:
            return
        out_cmd.throttle = max(out_cmd.throttle, min_positive_throttle)

    def _wake_sleeping_physics(self, out_cmd, in_cmd):
        """Wake the ego physics body when pulling away from a standstill.

        CARLA 0.10 (UE5/Chaos) puts a stationary vehicle's physics body to
        sleep, and VehicleControl throttle does NOT wake it, so a vehicle that
        has been stopped for a while can never launch again (observed: throttle
        applied, brake 0, first gear — velocity stays exactly 0 until an
        external set_target_velocity kick wakes the body). Nudge the body awake
        whenever the stack is trying to pull away from a standstill; once
        rolling (speed > 0.05 m/s) this no-ops. Needs ``_state_lock`` held.

        Gated behind ``wake_sleeping_physics`` (off by default): the kick
        overrides launch dynamics at every standstill start, so it must not run
        on the supported CARLA 0.9.15 environment, whose bodies never sleep.
        """
        if not self.param_values.get("wake_sleeping_physics", False):
            return
        if out_cmd.throttle <= 0.0 or in_cmd.actuation.brake_cmd > 0.0:
            return
        if self._ego_speed_mps() >= 0.05:
            return
        wake_yaw = math.radians(self.ego_actor.get_transform().rotation.yaw)
        self.ego_actor.set_target_velocity(
            carla.Vector3D(0.3 * math.cos(wake_yaw), 0.3 * math.sin(wake_yaw), 0.0)
        )

    def control_callback(self, in_cmd):
        """
        Convert and publish CARLA Ego Vehicle Control to AUTOWARE.

        Thread-safe: Acquires state lock when accessing shared vehicle state.

        """
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd
        # Keep the vehicle in first gear with manual shifting so heavy vehicles
        # respond to throttle immediately instead of idling in neutral.
        out_cmd.gear = 1
        out_cmd.manual_gear_shift = True

        with self._state_lock:
            # convert base on steer curve of the vehicle
            if not self.physics_control or not self.ego_actor:
                return  # Skip if vehicle not initialized yet

            self._apply_min_positive_throttle(out_cmd, in_cmd)
            self._wake_sleeping_physics(out_cmd, in_cmd)

            # steer_cmd is a tire angle in radians (raw_vehicle_cmd_converter
            # passes control_cmd.steering_tire_angle through), while
            # VehicleControl.steer expects a fraction of the wheel's max steer
            # angle in [-1, 1]. Normalize by the max wheel angle; the sign flips
            # because Autoware is CCW-positive and CARLA CW-positive.
            # NOTE: no steering_curve multiplication here — the simulator applies
            # its speed-based steering limit internally, and CARLA 0.10 returns
            # corrupt curve data (duplicated/unsorted points) anyway.
            steer_norm = -in_cmd.actuation.steer_cmd / self._max_wheel_steer_angle_rad()
            steer_norm = max(-1.0, min(1.0, steer_norm))
            out_cmd.steer = self.first_order_steering(steer_norm)
            out_cmd.brake = in_cmd.actuation.brake_cmd
            self.current_control = out_cmd

    def _max_wheel_steer_angle_rad(self):
        """Max steerable wheel angle [rad], cached from the vehicle physics.

        Must be called with ``physics_control`` already available.
        """
        if self._max_steer_angle_rad is None:
            max_deg = max((w.max_steer_angle for w in self.physics_control.wheels), default=0.0)
            if max_deg <= 0.0:
                max_deg = 70.0  # CARLA's usual front-wheel default
            self._max_steer_angle_rad = math.radians(max_deg)
        return self._max_steer_angle_rad

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
        # CARLA reports the angular velocity in deg/s in Unreal's left-handed
        # (CW-positive) frame, while ROS expects rad/s CCW-positive (REP-103):
        # https://carla.readthedocs.io/en/latest/python_api/#carla.Actor.get_angular_velocity
        # https://www.ros.org/reps/rep-0103.html
        out_vel_state.heading_rate = -math.radians(ego_angular_velocity.z)

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

    def _publish_ground_truth_odometry(self):
        """Publish /localization/kinematic_state and the map->base_link TF.

        Both are derived from the CARLA ground-truth ego transform, replacing
        the former carla_state_publisher GNSS round-trip. CARLA reports both
        velocities in its world frame, so each is rotated into the ego body
        frame before the CARLA-to-ROS (REP-103) conversion; the angular rate
        additionally converts deg/s to rad/s with the axis signs used by the
        official ros-bridge (x, -y, -z):
        https://carla.readthedocs.io/en/latest/python_api/#carla.Actor.get_angular_velocity
        https://www.ros.org/reps/rep-0103.html
        https://github.com/carla-simulator/ros-bridge/blob/master/carla_common/src/carla_common/transforms.py

        No-op unless publish_ground_truth_localization is enabled (the
        publishers only exist when it is).
        """
        if not self.param_values.get("publish_ground_truth_localization", False):
            return
        with self._state_lock:
            if not self.ego_actor:
                return
            ego_transform = self.ego_actor.get_transform()
            ego_vel = self.ego_actor.get_velocity()
            ego_ang_vel = self.ego_actor.get_angular_velocity()

        header = self.get_msg_header(frame_id="map")
        pose = Pose()
        pose.position = carla_location_to_ros_point(
            ego_transform.location,
            origin_x=self.param_values["map_origin_x"],
            origin_y=self.param_values["map_origin_y"],
        )
        pose.orientation = carla_rotation_to_ros_quaternion(ego_transform.rotation)

        tf_stamped = TransformStamped()
        tf_stamped.header = header
        tf_stamped.child_frame_id = "base_link"
        tf_stamped.transform.translation.x = pose.position.x
        tf_stamped.transform.translation.y = pose.position.y
        tf_stamped.transform.translation.z = pose.position.z
        tf_stamped.transform.rotation = pose.orientation
        self.pub_gt_tf.publish(TFMessage(transforms=[tf_stamped]))

        odom = Odometry()
        odom.header = header
        odom.child_frame_id = "base_link"
        odom.pose.pose = pose
        trans_mat = numpy.array(ego_transform.get_matrix()).reshape(4, 4)
        inv_rot_mat = trans_mat[0:3, 0:3].T
        vel_vec = numpy.array([ego_vel.x, ego_vel.y, ego_vel.z]).reshape(3, 1)
        body_vel = (inv_rot_mat @ vel_vec).T[0]
        odom.twist.twist.linear.x = float(body_vel[0])
        odom.twist.twist.linear.y = float(-body_vel[1])
        odom.twist.twist.linear.z = float(body_vel[2])
        ang_vel_vec = numpy.array([ego_ang_vel.x, ego_ang_vel.y, ego_ang_vel.z]).reshape(3, 1)
        body_ang_vel = (inv_rot_mat @ ang_vel_vec).T[0]
        odom.twist.twist.angular.x = math.radians(float(body_ang_vel[0]))
        odom.twist.twist.angular.y = -math.radians(float(body_ang_vel[1]))
        odom.twist.twist.angular.z = -math.radians(float(body_ang_vel[2]))
        self.pub_gt_odom.publish(odom)

    def _publish_sensor_data(self, key, data):
        """Publish one sensor's data, dispatching on its sensor type.

        Camera and lidar conversion/publishing run on per-sensor worker
        threads: publishing multi-megabyte messages inline (reliable-QoS
        camera images in particular block on DDS flow control) would stall
        the simulation loop and slow simulation time itself. Frequency
        gating and registry bookkeeping stay on the calling thread so the
        registry is never accessed concurrently.
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
            self.pose()
        elif sensor_type == "sensor.lidar.ray_cast":
            if not self.checkFrequency(key):
                self.sensor_registry.update_sensor_timestamp(key, self.timestamp)
                self._submit_to_publish_worker(key, self.lidar, data[1], key, self.timestamp)
        elif sensor_type == "sensor.other.imu":
            self.imu(data[1])
        else:
            self.logger.debug(f"No publisher for sensor '{key}' (type={sensor_type})")

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

        self._publish_ground_truth_odometry()

        # publish data of all sensors
        for key, data in input_data.items():
            self._publish_sensor_data(key, data)

        # Push turn indicator / hazard lights to CARLA before reading status back.
        self.apply_light_state()

        # Publish ego vehicle status
        self.ego_status()

        # Thread-safe read of current control command
        with self._state_lock:
            return self.current_control

    def shutdown(self):
        """
        Clean shutdown of ROS node and spin thread.

        Properly destroys publishers, stops the spin thread, and shuts down rclpy to prevent
        process hanging and publisher leaks.

        """
        # Stop publish workers before destroying the publishers they use
        for worker in self._publish_workers.values():
            worker.stop()
        self._publish_workers.clear()

        # Destroy publishers first
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
