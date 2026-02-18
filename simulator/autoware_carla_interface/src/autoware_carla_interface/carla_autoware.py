#!/usr/bin/env python3

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

import random
import signal
import time

import carla

from .carla_ros import carla_ros2_interface
from .modules.carla_data_provider import CarlaDataProvider
from .modules.carla_data_provider import GameTime
from .modules.carla_wrapper import SensorReceivedNoData
from .modules.carla_wrapper import SensorWrapper


class SensorLoop(object):

    def __init__(self):
        self.start_game_time = None
        self.start_system_time = None
        self.sensor = None
        self.ego_actor = None
        self.running = False
        self.timestamp_last_run = 0.0
        self.timeout = 20.0

    def _stop_loop(self):
        self.running = False

    def _tick_sensor(self, timestamp):
        if self.timestamp_last_run < timestamp.elapsed_seconds and self.running:
            self.timestamp_last_run = timestamp.elapsed_seconds
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            try:
                ego_action = self.sensor()
            except SensorReceivedNoData as e:
                raise RuntimeError(e)
            self.ego_actor.apply_control(ego_action)
        if self.running:
            CarlaDataProvider.get_world().tick()


class InitializeInterface(object):

    def __init__(self):
        self.interface = carla_ros2_interface()
        self.param_ = self.interface.get_param()
        self.world = None
        self.sensor_wrapper = None
        self.ego_actor = None
        self.prev_tick_wall_time = 0.0

        # Parameter for Initializing Carla World
        self.local_host = self.param_["host"]
        self.port = self.param_["port"]
        self.timeout = self.param_["timeout"]
        self.sync_mode = self.param_["sync_mode"]
        self.fixed_delta_seconds = self.param_["fixed_delta_seconds"]
        self.carla_map = self.param_["carla_map"]
        self.agent_role_name = self.param_["ego_vehicle_role_name"]
        self.vehicle_type = self.param_["vehicle_type"]
        self.spawn_point = self.param_["spawn_point"]
        self.use_traffic_manager = self.param_["use_traffic_manager"]
        self.max_real_delta_seconds = self.param_["max_real_delta_seconds"]

    def _parse_spawn_point(self):
        """Parse spawn point string and return transform with randomize flag."""
        spawn_point = carla.Transform()
        point_items = self.spawn_point.split(",")
        randomize = False
        if len(point_items) == 6:
            spawn_point.location.x = float(point_items[0])
            spawn_point.location.y = float(point_items[1])
            spawn_point.location.z = (
                float(point_items[2]) + 2
            )  # +2 is used so the car did not stuck on the road when spawned.
            spawn_point.rotation.roll = float(point_items[3])
            spawn_point.rotation.pitch = float(point_items[4])
            spawn_point.rotation.yaw = float(point_items[5])
        else:
            randomize = True
        return spawn_point, randomize

    def _setup_traffic_manager(self, client):
        """Configure traffic manager with NPC vehicles."""
        traffic_manager = client.get_trafficmanager()  # cspell:ignore trafficmanager
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(0)
        random.seed(0)
        spawn_points_tm = self.world.get_map().get_spawn_points()
        for i, spawn_point in enumerate(spawn_points_tm):
            self.world.debug.draw_string(spawn_point.location, str(i), life_time=10)
        models = [
            "dodge",
            "audi",
            "model3",
            "mini",
            "mustang",
            "lincoln",
            "prius",
            "nissan",
            "crown",
            "impala",
        ]
        blueprints = []
        for vehicle in self.world.get_blueprint_library().filter("*vehicle*"):
            if any(model in vehicle.id for model in models):
                blueprints.append(vehicle)
        max_vehicles = 30
        max_vehicles = min([max_vehicles, len(spawn_points_tm)])
        vehicles = []
        for i, spawn_point in enumerate(random.sample(spawn_points_tm, max_vehicles)):
            temp = self.world.try_spawn_actor(random.choice(blueprints), spawn_point)
            if temp is not None:
                vehicles.append(temp)

        for vehicle in vehicles:
            vehicle.set_autopilot(True)

    def load_world(self):
        client = carla.Client(self.local_host, self.port)
        client.set_timeout(self.timeout)
        client.load_world(self.carla_map)

        # Wait for the world to be fully loaded
        # This is critical for non-default maps that need time to load
        time.sleep(2.0)

        self.world = client.get_world()

        # Verify world is ready by attempting to tick it
        # This ensures the world is fully initialized before accessing settings
        try:
            self.world.tick()
        except RuntimeError:
            # If synchronous mode is not enabled yet, tick() may fail
            # In this case, just wait a bit more
            time.sleep(1.0)

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.synchronous_mode = self.sync_mode
        self.world.apply_settings(settings)
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(client)

        spawn_point, randomize = self._parse_spawn_point()
        self.ego_actor = CarlaDataProvider.request_new_actor(
            self.vehicle_type, spawn_point, self.agent_role_name, random_location=randomize
        )
        self.interface.ego_actor = self.ego_actor  # TODO improve design
        self.interface.physics_control = self.ego_actor.get_physics_control()

        self.sensor_wrapper = SensorWrapper(self.interface)
        self.sensor_wrapper.setup_sensors(self.ego_actor, False)

        if self.use_traffic_manager:
            self._setup_traffic_manager(client)

    def run_bridge(self):
        self.bridge_loop = SensorLoop()
        self.bridge_loop.sensor = self.sensor_wrapper
        self.bridge_loop.ego_actor = self.ego_actor
        self.bridge_loop.start_system_time = time.time()
        self.bridge_loop.start_game_time = GameTime.get_time()
        self.bridge_loop.running = True
        while self.bridge_loop.running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                delta_step = time.time() - self.prev_tick_wall_time
                if delta_step <= self.max_real_delta_seconds:
                    # Add a wait to match the max_real_delta_seconds
                    time.sleep(self.max_real_delta_seconds - delta_step)
                self.prev_tick_wall_time = time.time()
                self.bridge_loop._tick_sensor(timestamp)

    def _stop_loop(self, sign, frame):
        self.bridge_loop._stop_loop()

    def _cleanup(self):
        """
        Clean up all CARLA resources in reverse initialization order.

        Ensures cleanup happens even if individual steps fail.

        """
        self._cleanup_sensors()
        self._cleanup_ros_interface()
        self._cleanup_ego_actor()
        self._cleanup_carla_provider()

    def _cleanup_sensors(self):
        """Clean up sensor wrapper, continuing on error."""
        if not self.sensor_wrapper:
            return
        try:
            self.sensor_wrapper.cleanup()
        except Exception as e:
            print(f"Warning: Sensor cleanup failed: {e}")

    def _cleanup_ros_interface(self):
        """Clean up ROS interface, continuing on error."""
        if not self.interface:
            return
        try:
            self.interface.shutdown()
            self.interface = None
        except Exception as e:
            print(f"Warning: ROS interface shutdown failed: {e}")

    def _cleanup_ego_actor(self):
        """Destroy ego vehicle, continuing on error."""
        if not self.ego_actor:
            return
        try:
            self.ego_actor.destroy()
            self.ego_actor = None
        except Exception as e:
            print(f"Warning: Ego actor destruction failed: {e}")

    def _cleanup_carla_provider(self):
        """Clean up CARLA data provider, continuing on error."""
        try:
            CarlaDataProvider.cleanup()
        except Exception as e:
            print(f"Warning: CARLA data provider cleanup failed: {e}")


def main():
    """Run the CARLA-Autoware bridge with proper cleanup on all exit paths."""
    carla_bridge = InitializeInterface()
    carla_bridge.load_world()

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, carla_bridge._stop_loop)
    signal.signal(signal.SIGTERM, carla_bridge._stop_loop)

    try:
        carla_bridge.run_bridge()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"\nError during bridge operation: {e}")
        raise
    finally:
        # Ensure cleanup always happens, even on exception or signal
        print("Cleaning up CARLA resources...")
        carla_bridge._cleanup()
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
