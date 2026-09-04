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
from .modules.carla_utils import project_point_to_ground
from .modules.carla_wrapper import SensorReceivedNoData
from .modules.carla_wrapper import SensorWrapper


class CarlaWorldLoadError(RuntimeError):
    """Raised when CARLA is not running the map the bridge was asked to load."""


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
        self.logger = self.interface.logger
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
        self.spawn_point_ground_snap = self.param_["spawn_point_ground_snap"]
        self.spawn_point_ground_offset_z = self.param_["spawn_point_ground_offset_z"]
        self.force_load_world = self.param_["force_load_world"]
        self.no_rendering_mode = self.param_["no_rendering_mode"]

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

    def _get_map_spawn_points(self):
        """Return the map spawn points, or an empty list if the map is unavailable."""
        try:
            return self.world.get_map().get_spawn_points()
        except RuntimeError as error:
            # cspell:ignore mapless
            # Mapless CARLA levels (no parseable OpenDRIVE metadata) expose no map.
            print(f"WARNING: Map spawn points are unavailable (mapless level?): {error}")
            return []

    def _snap_spawn_point_to_ground(self, spawn_point):
        """Snap a spawn point onto the CARLA map geometry, if enabled.

        When spawn_point_ground_snap is disabled, or no ground height can be
        found (older CARLA APIs without ``ground_projection``, or no ground
        hit), the spawn point is returned unchanged so behavior matches the
        fixed z that the caller already set.
        """
        if not self.spawn_point_ground_snap:
            return spawn_point

        ground_z = project_point_to_ground(
            self.world, spawn_point.location.x, spawn_point.location.y
        )
        if ground_z is None:
            print("WARNING: Could not ground-snap CARLA spawn point; keeping configured z")
            return spawn_point

        snapped = carla.Transform(carla.Location(), spawn_point.rotation)
        snapped.location.x = spawn_point.location.x
        snapped.location.y = spawn_point.location.y
        snapped.location.z = ground_z + self.spawn_point_ground_offset_z
        # NOTE: request_new_actor() adds an unconditional +0.2 m safety lift for
        # non-prop models, so the actual spawn z is requested_z + 0.2. This log
        # reports the requested pose before that downstream lift.
        print(
            "Ground-snapped spawn point: "
            f"ground_z={ground_z:.3f}, offset_z={self.spawn_point_ground_offset_z:.3f}, "
            f"requested_z={snapped.location.z:.3f} (before safety lift)",
            flush=True,
        )
        return snapped

    def _flatten_steering_curve(self):
        """Replace the vehicle's speed-based steering curve with an identity curve.

        CARLA 0.10 ships corrupt steering-curve data (duplicated, unsorted
        points such as (10 m/s, 0.5)) which the simulator applies internally,
        attenuating the achievable steering angle at driving speeds. Writing a
        flat curve back removes the server-side attenuation so the commanded
        steer fraction maps directly to the wheel angle.
        """
        try:
            physics = self.ego_actor.get_physics_control()
            physics.steering_curve = [
                carla.Vector2D(0.0, 1.0),
                carla.Vector2D(120.0, 1.0),
            ]
            self.ego_actor.apply_physics_control(physics)
            self.interface.physics_control = physics
            print("INFO: Applied a flat steering curve to the ego vehicle.")
        except RuntimeError as error:
            print(f"WARNING: Failed to flatten the steering curve: {error}")

    def _reload_world(self, client):
        """Reload the world via client.load_world(); return the failure, if any."""
        self.logger.info(f"Loading CARLA world '{self.carla_map}' with client.load_world()")
        try:
            client.load_world(self.carla_map)
        except RuntimeError as exc:
            # Not fatal on its own: some API versions raise after the level has
            # actually switched. _verify_world_loaded decides, from the map that
            # is really active, whether this run can continue.
            self.logger.warning(
                f"client.load_world() raised while loading '{self.carla_map}': {exc}"
            )
            return exc
        return None

    @staticmethod
    def _normalize_map_name(map_name):
        """Reduce a CARLA level name such as 'Carla/Maps/Town01' to 'Town01'."""
        return map_name.split("/")[-1]

    def _query_world_map(self, client):
        """Return (map_name, query_failed) for the currently active CARLA world."""
        try:
            return self._normalize_map_name(client.get_world().get_map().name), False
        except RuntimeError as exc:
            self.logger.warning(f"Failed to query the active CARLA map: {exc}")
            return None, True

    def _current_world_map(self, client):
        """Return the current world's map name, or None if it cannot be determined."""
        name, _ = self._query_world_map(client)
        return name

    def _load_world_if_different(self, client):
        """Try load_world_if_different(); return True on success, False to fall back."""
        if not hasattr(client, "load_world_if_different"):
            return False
        try:
            self.logger.info(
                f"Loading CARLA world '{self.carla_map}' with load_world_if_different()"
            )
            client.load_world_if_different(self.carla_map)
            return True
        except RuntimeError as exc:
            self.logger.warning(
                "load_world_if_different failed; falling back to load_world "
                f"for '{self.carla_map}': {exc}"
            )
            return False

    def _verify_world_loaded(self, client, load_error):
        """Raise unless the requested map is the world CARLA is actually running.

        Returns True when the map name was verified, False when verification was
        skipped because the map metadata is not parseable (mapless level).
        """
        expected = self._normalize_map_name(self.carla_map)
        deadline = time.time() + max(float(self.timeout), 1.0)
        current = None
        query_ever_failed = False
        while True:
            current, query_failed = self._query_world_map(client)
            query_ever_failed = query_ever_failed or query_failed
            if current == expected:
                self.logger.info(f"Loaded CARLA world '{expected}'")
                return True
            if time.time() >= deadline:
                break
            time.sleep(1.0)

        if query_ever_failed:
            # cspell:ignore libcarla
            # The map query raised at least once instead of returning a name (e.g. a
            # CARLA level without parseable OpenDRIVE metadata). After such a failure
            # libcarla keeps serving the previous episode's cached map, so a later
            # "successful" query reporting a mismatched name cannot be trusted either.
            # CarlaDataProvider tolerates running without a map, so proceed rather
            # than aborting the bridge.
            self.logger.warning(
                f"Could not verify CARLA loaded '{expected}' by map name (no parseable "
                "OpenDRIVE metadata); continuing without map verification."
            )
            return False

        message = (
            f"CARLA world mismatch: requested map '{expected}' but the active world is "
            f"'{current if current is not None else 'unknown'}'. Continuing would run "
            "Autoware against a map the simulator is not simulating, so the bridge is "
            "aborted."
        )
        self.logger.error(message)
        raise CarlaWorldLoadError(message) from load_error

    def _load_carla_world(self, client):
        """Load the requested map while supporting CARLA Python API version differences.

        Returns True when the loaded map name was verified, False when the level
        exposes no parseable map metadata (see _verify_world_loaded).
        """
        load_error = None
        if self.force_load_world:
            load_error = self._reload_world(client)
        elif not self._load_world_if_different(client):
            if self._current_world_map(client) != self._normalize_map_name(self.carla_map):
                load_error = self._reload_world(client)

        return self._verify_world_loaded(client, load_error)

    def _setup_traffic_manager(self, client):
        """Configure traffic manager with NPC vehicles."""
        spawn_points_tm = self._get_map_spawn_points()
        if not spawn_points_tm:
            # No spawn points means there is nowhere to place NPC traffic; skip it
            # so mapless levels can still start with use_traffic_manager enabled.
            print("WARNING: Skipping traffic-manager NPC setup; no map spawn points available.")
            return

        traffic_manager = client.get_trafficmanager()  # cspell:ignore trafficmanager
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(0)
        random.seed(0)
        for i, spawn_point in enumerate(spawn_points_tm):
            self.world.debug.draw_string(spawn_point.location, str(i), life_time=10)
        self._spawn_npc_vehicles(spawn_points_tm)

    def _npc_vehicle_blueprints(self):
        """Return the blueprints of the vehicle models used as NPC traffic."""
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
        return [
            vehicle
            for vehicle in self.world.get_blueprint_library().filter("*vehicle*")
            if any(model in vehicle.id for model in models)
        ]

    def _spawn_npc_vehicles(self, spawn_points_tm):
        """Spawn autopilot NPC vehicles on a random subset of the spawn points."""
        blueprints = self._npc_vehicle_blueprints()
        max_vehicles = min(30, len(spawn_points_tm))
        vehicles = []
        for spawn_point in random.sample(spawn_points_tm, max_vehicles):
            vehicle = self.world.try_spawn_actor(random.choice(blueprints), spawn_point)
            if vehicle is not None:
                vehicles.append(vehicle)

        for vehicle in vehicles:
            vehicle.set_autopilot(True)

    def load_world(self):
        client = carla.Client(self.local_host, self.port)
        client.set_timeout(self.timeout)
        map_verified = self._load_carla_world(client)
        if not map_verified:
            # After a failed OpenDRIVE parse, libcarla keeps serving the previous
            # episode's cached map through this client, so world.get_map() would
            # return a stale (wrong) map instead of raising. Reconnect with a fresh
            # client so the mapless world reports honestly downstream
            # (CarlaDataProvider.set_world then runs its map-optional fallbacks).
            self.logger.warning(
                "Reconnecting the CARLA client to discard the stale map cache "
                "of the previous episode."
            )
            client = carla.Client(self.local_host, self.port)
            client.set_timeout(self.timeout)

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
        settings.no_rendering_mode = self.no_rendering_mode
        self.world.apply_settings(settings)
        CarlaDataProvider.set_world(self.world)
        CarlaDataProvider.set_client(client)

        spawn_point, randomize = self._parse_spawn_point()
        if not randomize:
            spawn_point = self._snap_spawn_point_to_ground(spawn_point)
        self.ego_actor = CarlaDataProvider.request_new_actor(
            self.vehicle_type, spawn_point, self.agent_role_name, random_location=randomize
        )
        if self.ego_actor is None:
            raise RuntimeError(
                f"Failed to spawn ego vehicle '{self.vehicle_type}' at "
                f"({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f}, "
                f"{spawn_point.location.z:.1f}); the spawn point may be occupied "
                "or invalid for this map"
            )
        self.interface.ego_actor = self.ego_actor  # TODO improve design
        self.interface.physics_control = self.ego_actor.get_physics_control()
        if self.interface.param_values.get("flatten_steering_curve", False):
            self._flatten_steering_curve()

        self.sensor_wrapper = SensorWrapper(self.interface)
        self.sensor_wrapper.setup_sensors(self.ego_actor, False)

        # World, map and ego are now all confirmed loaded: resolve the map
        # origin from the final map and apply any initial pose that arrived
        # (and was buffered) during startup.
        self.interface.on_world_ready()

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

    try:
        # Loading the world is inside the try so that a failure here, a map
        # mismatch above all, shuts the ROS interface down instead of leaving
        # the non-daemon spin thread keeping a half-initialized process alive.
        carla_bridge.load_world()

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, carla_bridge._stop_loop)
        signal.signal(signal.SIGTERM, carla_bridge._stop_loop)

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
