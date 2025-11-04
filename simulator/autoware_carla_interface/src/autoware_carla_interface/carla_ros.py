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
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from transforms3d.euler import euler2quat

# New modular sensor infrastructure
from .modules import ROSPublisherManager
from .modules import SensorKitLoader
from .modules import SensorRegistry
from .modules.carla_data_provider import GameTime
from .modules.carla_utils import carla_location_to_ros_point
from .modules.carla_utils import carla_rotation_to_ros_quaternion
from .modules.carla_utils import create_cloud
from .modules.carla_utils import ros_pose_to_carla_transform
from .modules.carla_wrapper import SensorInterface


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
            # Sensor configuration parameters
            "sensor_kit_name": (rclpy.Parameter.Type.STRING, ""),  # Empty = use YAML default
            "sensor_mapping_file": (rclpy.Parameter.Type.STRING, ""),
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

    def _initialize_subscriptions(self):
        """Initialize all ROS 2 subscriptions."""
        self.sub_control = self.ros2_node.create_subscription(
            ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        )
        self.sub_vehicle_initialpose = self.ros2_node.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 1
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
        self.timestamp = None
        self.ego_actor = None
        self.physics_control = None
        self.current_control = carla.VehicleControl()

        # Thread synchronization (protects: current_control, ego_actor, timestamp, physics_control)
        self._state_lock = threading.Lock()

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

    def get_msg_header(self, frame_id):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def lidar(self, carla_lidar_measurement, id_):
        """Transform the received lidar measurement into a ROS point cloud message."""
        if self.checkFrequency(id_):
            return

        config = self.sensor_registry.get_sensor(id_)
        if not config:
            self.logger.warning(f"No registry entry for LiDAR sensor '{id_}'")
            return

        header = self.get_msg_header(frame_id=config.frame_id or "base_link")
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
        publisher = self.pub_lidar.get(id_)
        if publisher is None:
            self.logger.warning(f"LiDAR publisher missing for '{id_}'")
            return

        publisher.publish(point_cloud_msg)
        self.sensor_registry.update_sensor_timestamp(id_, self.timestamp)

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

    def camera(self, carla_camera_data, cam_id):
        """Handle multiple cameras with dynamic routing by sensor ID."""
        config = self.sensor_registry.get_sensor(cam_id)
        if not config:
            self.logger.warning(f"No registry entry for camera '{cam_id}'")
            return

        if cam_id not in self.camera_info_cache:
            self.camera_info_cache[cam_id] = self._build_camera_info(carla_camera_data)

        if self.checkFrequency(cam_id):
            return

        # Create image message
        image_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(
            frame_id=config.frame_id or f"{cam_id}/camera_optical_link"
        )

        # Publish camera info
        cam_info = self.camera_info_cache[cam_id]
        cam_info.header = img_msg.header
        info_pub = self.pub_camera_info.get(cam_id)
        if info_pub:
            info_pub.publish(cam_info)

        # Publish image
        img_pub = self.pub_camera.get(cam_id)
        if img_pub:
            img_pub.publish(img_msg)
            self.sensor_registry.update_sensor_timestamp(cam_id, self.timestamp)

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

        self.pub_actuation_status.publish(out_actuation_status)
        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)
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
            # Safely get sensor type with fallback
            sensor_type = self.id_to_sensor_type_map.get(key)
            if not sensor_type:
                self.logger.warning(
                    f"Unknown sensor ID '{key}' received from CARLA - skipping. "
                    f"This may indicate a sensor configuration mismatch."
                )
                continue

            if sensor_type == "sensor.camera.rgb":
                self.camera(data[1], key)  # Pass sensor ID for multi-camera support
            elif sensor_type == "sensor.other.gnss":
                self.pose()
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(data[1], key)
            elif sensor_type == "sensor.other.imu":
                self.imu(data[1])
            else:
                self.logger.debug(f"No publisher for sensor '{key}' (type={sensor_type})")

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
