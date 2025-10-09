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
# limitations under the License.
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
from .modules.carla_data_provider import datetime
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
        # Disabled TF listener as it causes localization issues
        # The viewer->map transform is not needed for CARLA simulation
        self.tf_buffer = None
        self.tf_listener = None

    def _initialize_status_publishers(self):
        """Initialize all vehicle status publishers."""
        self.pub_pose_with_cov = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )
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
                "Unable to locate sensor mapping YAML. Provide --ros-args -p sensor_mapping_file:=<path>"
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
                "Sensor mapping produced zero sensors. Check enabled_sensors list and calibration files."
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

        # Load sensor configuration (NEW: uses dynamic or legacy mode)
        self._load_sensor_configuration()

        # Initialize publishers and subscriptions
        self._initialize_subscriptions()
        self._initialize_status_publishers()

        # Start ROS 2 spin thread
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
        self.channels = 0
        self.current_control = carla.VehicleControl()

        # ROS-related helpers initialized later
        self.ros2_node = None
        self.ros_publisher_manager = None
        self.clock_publisher = None
        self.spin_thread = None

        # Miscellaneous
        self.cv_bridge = CvBridge()

        # Frequency control for legacy sensors
        self.sensor_frequencies = {
            "top": 11,
            "left": 11,
            "right": 11,
            "camera": 11,
            "imu": 50,
            "status": 50,
            "pose": 2,
        }
        self.publish_prev_times = {
            sensor: datetime.datetime.now() for sensor in self.sensor_frequencies
        }

    def __call__(self):
        input_data = self.sensor_interface.get_data()
        timestamp = GameTime.get_time()
        control = self.run_step(input_data, timestamp)
        return control

    def get_param(self):
        return self.param_values

    def checkFrequency(self, sensor):
        """Return True when publication should be throttled for the sensor."""
        # Check if it's a legacy sensor first
        if sensor in self.sensor_frequencies:
            if sensor not in self.publish_prev_times:
                self.publish_prev_times[sensor] = datetime.datetime.now()
                return False

            time_delta = (
                datetime.datetime.now() - self.publish_prev_times[sensor]
            ).microseconds / 1000000.0
            if time_delta == 0:
                return True
            if 1.0 / time_delta >= self.sensor_frequencies[sensor]:
                return True
            return False

        # Otherwise, use the sensor registry
        config = self.sensor_registry.get_sensor(sensor)
        if not config:
            return False

        current_time = self.timestamp if self.timestamp is not None else None
        if current_time is None:
            return False

        should_publish = self.sensor_registry.should_publish(sensor, current_time)
        return not should_publish

    def get_msg_header(self, frame_id):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def _transform_viewer_pose_to_map(self, position, orientation):
        """Transform pose from viewer frame to map frame if TF is available."""
        # Direct pass-through without transformation
        # CARLA coordinates are already in the correct frame
        return position, orientation

    def lidar(self, carla_lidar_measurement, id_):
        """Transform the received lidar measurement into a ROS point cloud message."""
        if self.checkFrequency(id_):
            return
        self.publish_prev_times[id_] = datetime.datetime.now()

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
        """Transform RVIZ initial pose to CARLA."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = ros_pose_to_carla_transform(pose)
        if self.ego_actor is not None:
            self.ego_actor.set_transform(carla_pose_transform)
        else:
            self.logger.warning("Cannot set initial pose: ego vehicle not available")

    def pose(self):
        """Transform odometry data to Pose and publish Pose with Covariance message."""
        if self.checkFrequency("pose"):
            return
        self.publish_prev_times["pose"] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="map")
        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()
        pose_carla.position = carla_location_to_ros_point(self.ego_actor.get_transform().location)
        pose_carla.orientation = carla_rotation_to_ros_quaternion(
            self.ego_actor.get_transform().rotation
        )
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
        out_pose_with_cov.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_pose_with_cov.publish(out_pose_with_cov)

    def _build_camera_info(self, camera_actor):
        """
        Build camera info message from CARLA camera actor.

        Args:
            camera_actor: CARLA camera data with width, height, fov

        Returns:
            CameraInfo: Populated camera info message
        """
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

        # Build camera info if not cached
        if cam_id not in self.camera_info_cache:
            self.camera_info_cache[cam_id] = self._build_camera_info(carla_camera_data)

        # Each camera uses its own frequency timer
        if self.checkFrequency(cam_id):
            return

        # Get native Carla image
        image_data_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )

        # cspell:ignore interp bgra
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(
            frame_id=config.frame_id or f"{cam_id}/camera_optical_link"
        )

        # Publish to appropriate topics for this camera
        cam_info = self.camera_info_cache[cam_id]
        cam_info.header = img_msg.header
        publisher = self.pub_camera.get(cam_id)
        info_publisher = self.pub_camera_info.get(cam_id)

        if info_publisher:
            info_publisher.publish(cam_info)
        else:
            self.logger.warning(f"Camera info publisher missing for '{cam_id}'")

        if publisher:
            publisher.publish(img_msg)
            self.sensor_registry.update_sensor_timestamp(cam_id, self.timestamp)
        else:
            self.logger.warning(f"Camera image publisher missing for '{cam_id}'")

    def imu(self, carla_imu_measurement):
        """Transform a received imu measurement into a ROS Imu message and publish Imu message."""
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
        """First order steering model."""
        steer_output = 0.0
        if self.prev_timestamp is None:
            self.prev_timestamp = self.timestamp

        dt = self.timestamp - self.prev_timestamp
        if dt > 0.0:
            steer_output = self.prev_steer_output + (steer_input - self.prev_steer_output) * (
                dt / (self.tau + dt)
            )
        self.prev_steer_output = steer_output
        self.prev_timestamp = self.timestamp
        return steer_output

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd
        # convert base on steer curve of the vehicle
        steer_curve = self.physics_control.steering_curve
        current_vel = self.ego_actor.get_velocity()
        max_steer_ratio = numpy.interp(
            abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
        )
        out_cmd.steer = self.first_order_steering(-in_cmd.actuation.steer_cmd) * max_steer_ratio
        out_cmd.brake = in_cmd.actuation.brake_cmd
        self.current_control = out_cmd

    def ego_status(self):
        """Publish ego vehicle status."""
        if self.checkFrequency("status"):
            return

        self.publish_prev_times["status"] = datetime.datetime.now()

        # convert velocity from cartesian to ego frame
        trans_mat = numpy.array(self.ego_actor.get_transform().get_matrix()).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array(
            [
                self.ego_actor.get_velocity().x,
                self.ego_actor.get_velocity().y,
                self.ego_actor.get_velocity().z,
            ]
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
        out_vel_state.heading_rate = (
            self.ego_actor.get_transform().transform_vector(self.ego_actor.get_angular_velocity()).z
        )

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -math.radians(
            self.ego_actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        )

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        control = self.ego_actor.get_control()
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
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == "sensor.camera.rgb":
                self.camera(data[1], key)  # Pass sensor ID for multi-camera support
            elif sensor_type == "sensor.other.gnss":
                self.pose()
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(data[1], key)
            elif sensor_type == "sensor.other.imu":
                self.imu(data[1])
            else:
                self.logger.info(f"No publisher for sensor '{key}' (type={sensor_type})")

        # Publish ego vehicle status
        self.ego_status()
        return self.current_control

    def shutdown(self):
        if self.ros_publisher_manager:
            self.ros_publisher_manager.destroy_all_publishers()

        if self.ros2_node:
            self.ros2_node.destroy_node()
