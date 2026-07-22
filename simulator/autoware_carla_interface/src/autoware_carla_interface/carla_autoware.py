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

import math
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
        self.no_rendering_mode = self.param_["no_rendering_mode"]
        self.carla_map = self.param_["carla_map"]
        self.force_load_world = self.param_["force_load_world"]
        self.agent_role_name = self.param_["ego_vehicle_role_name"]
        self.vehicle_type = self.param_["vehicle_type"]
        self.spawn_point = self.param_["spawn_point"]
        self.spawn_point_ground_snap = self.param_["spawn_point_ground_snap"]
        self.spawn_point_ground_offset_z = self.param_["spawn_point_ground_offset_z"]
        self.use_traffic_manager = self.param_["use_traffic_manager"]
        self.max_real_delta_seconds = self.param_["max_real_delta_seconds"]
        self._spectator_initialized = False

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

    def _load_carla_world(self, client):
        """Load the requested map while supporting CARLA Python API version differences."""
        if self.force_load_world:
            print(f"Loading CARLA world '{self.carla_map}' with client.load_world()", flush=True)
            try:
                client.load_world(self.carla_map)
                print(f"Loaded CARLA world '{self.carla_map}'", flush=True)
            except RuntimeError as exc:
                if "Connection refused" in str(exc):
                    raise
                print(
                    "WARNING: client.load_world() raised while loading "
                    f"'{self.carla_map}'; continuing with current world: {exc}",
                    flush=True,
                )
            return

        if hasattr(client, "load_world_if_different"):
            try:
                print(
                    f"Loading CARLA world '{self.carla_map}' with load_world_if_different()",
                    flush=True,
                )
                client.load_world_if_different(self.carla_map)
                print(f"Loaded CARLA world '{self.carla_map}'", flush=True)
                return
            except RuntimeError as exc:
                print(
                    "WARNING: load_world_if_different failed; falling back to load_world "
                    f"for '{self.carla_map}': {exc}"
                )

        current_map = None
        try:
            current_map = client.get_world().get_map().name.split("/")[-1]
        except RuntimeError:
            pass

        if current_map != self.carla_map:
            print(f"Loading CARLA world '{self.carla_map}' with client.load_world()", flush=True)
            try:
                client.load_world(self.carla_map)
                print(f"Loaded CARLA world '{self.carla_map}'", flush=True)
            except RuntimeError as exc:
                if "Connection refused" in str(exc):
                    raise
                print(
                    "WARNING: client.load_world() raised while loading "
                    f"'{self.carla_map}'; continuing with current world: {exc}",
                    flush=True,
                )

    def _snap_spawn_point_to_ground(self, spawn_point):
        if not self.spawn_point_ground_snap:
            return spawn_point

        sample_offsets = (
            (0.0, 0.0),
            (0.75, 0.0),
            (-0.75, 0.0),
            (0.0, 0.75),
            (0.0, -0.75),
            (1.5, 0.0),
            (-1.5, 0.0),
            (0.0, 1.5),
            (0.0, -1.5),
        )
        projected_heights = []
        try:
            for offset_x, offset_y in sample_offsets:
                search_origin = carla.Location(
                    x=spawn_point.location.x + offset_x,
                    y=spawn_point.location.y + offset_y,
                    z=1000.0,
                )
                labelled_point = self.world.ground_projection(search_origin, 10000.0)
                if labelled_point is not None:
                    projected_heights.append(labelled_point.location.z)
        except RuntimeError as exc:
            print(f"WARNING: Could not ground-snap CARLA spawn point: {exc}")
            return spawn_point

        if not projected_heights:
            print("WARNING: Could not ground-snap CARLA spawn point: no ground projection found")
            return spawn_point

        ground_z = max(projected_heights)
        snapped = carla.Transform(carla.Location(), spawn_point.rotation)
        snapped.location.x = spawn_point.location.x
        snapped.location.y = spawn_point.location.y
        snapped.location.z = ground_z + self.spawn_point_ground_offset_z
        print(
            "Ground-snapped spawn point: "
            f"ground_z={ground_z:.3f}, offset_z={self.spawn_point_ground_offset_z:.3f}, "
            f"spawn_z={snapped.location.z:.3f}",
            flush=True,
        )
        return snapped

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
        print("Connecting to CARLA server", flush=True)
        self._load_carla_world(client)

        # Wait for the world to be fully loaded
        # This is critical for non-default maps that need time to load
        print("Waiting for CARLA world initialization", flush=True)
        time.sleep(2.0)

        self.world = client.get_world()
        print("CARLA world handle acquired", flush=True)

        # Verify world is ready by attempting to tick it
        # This ensures the world is fully initialized before accessing settings
        try:
            print("Ticking CARLA world before applying settings", flush=True)
            self.world.tick()
            print("Initial CARLA world tick completed", flush=True)
        except RuntimeError:
            # If synchronous mode is not enabled yet, tick() may fail
            # In this case, just wait a bit more
            print("Initial CARLA world tick failed; waiting before applying settings", flush=True)
            time.sleep(1.0)

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.synchronous_mode = self.sync_mode
        settings.no_rendering_mode = self.no_rendering_mode
        print(
            "Applying CARLA settings: "
            f"sync={settings.synchronous_mode}, "
            f"fixed_delta_seconds={settings.fixed_delta_seconds}, "
            f"no_rendering_mode={settings.no_rendering_mode}",
            flush=True,
        )
        self.world.apply_settings(settings)
        print("CARLA settings applied", flush=True)
        CarlaDataProvider.set_world(self.world)
        print("CARLA data provider world set", flush=True)
        CarlaDataProvider.set_client(client)

        spawn_point, randomize = self._parse_spawn_point()
        if not randomize:
            spawn_point = self._snap_spawn_point_to_ground(spawn_point)
        print(
            f"Spawning ego vehicle '{self.vehicle_type}' at {spawn_point} "
            f"(random_location={randomize})",
            flush=True,
        )
        self.ego_actor = CarlaDataProvider.request_new_actor(
            self.vehicle_type, spawn_point, self.agent_role_name, random_location=randomize
        )
        if self.ego_actor is None:
            raise RuntimeError(
                f"Failed to spawn ego vehicle '{self.vehicle_type}' on CARLA map "
                f"'{self.carla_map}' at {spawn_point}"
            )
        print(f"Spawned ego vehicle actor id={self.ego_actor.id}", flush=True)
        self.interface.ego_actor = self.ego_actor  # TODO improve design
        self.interface.physics_control = self.ego_actor.get_physics_control()

        # Place spectator camera behind ego vehicle (third-person chase view)
        self._update_spectator()

        self.sensor_wrapper = SensorWrapper(self.interface)
        print("Spawning CARLA sensors", flush=True)
        self.sensor_wrapper.setup_sensors(self.ego_actor, False)
        print("CARLA sensors spawned", flush=True)

        # Initialize splatsim cameras after CARLA world and ego actor are ready
        self.interface.init_splatsim_cameras()

        if self.use_traffic_manager:
            self._setup_traffic_manager(client)

    def _update_spectator(self):
        """Place spectator behind spawn point once (no chase)."""
        if not self.ego_actor or not self.world:
            return
        if self._spectator_initialized:
            return
        ego_t = self.ego_actor.get_transform()
        yaw_rad = math.radians(ego_t.rotation.yaw)
        self.world.get_spectator().set_transform(
            carla.Transform(
                carla.Location(
                    x=ego_t.location.x - 15.0 * math.cos(yaw_rad),
                    y=ego_t.location.y - 15.0 * math.sin(yaw_rad),
                    z=ego_t.location.z + 5.0,
                ),
                carla.Rotation(pitch=-15.0, yaw=ego_t.rotation.yaw, roll=0.0),
            )
        )
        self._spectator_initialized = True

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
                self._update_spectator()

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
