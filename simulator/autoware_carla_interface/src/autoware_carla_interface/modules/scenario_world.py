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

"""Adopt the CARLA world and the ego the scenario runner owns (scenario mode).

Extracted from ``carla_autoware`` so the handoff logic lives on its own: in
scenario mode the interface loads neither the world nor the ego, it waits for
the ``autoware_carla_scenario`` runner to bring both up and then adopts them.
See :func:`wait_for_external_world` and :func:`attach_to_scenario_ego` for the
two halves of the handoff contract.

Nothing here imports ``carla``: the world and the actors are only ever used
through the handful of methods named below, so the handoff is unit-testable
without a simulator.
"""

from __future__ import annotations

import time

#: How often to look for the scenario's ego while waiting to attach to it.
EGO_ATTACH_POLL_INTERVAL_S = 0.5


class ScenarioWorldNotOwned(RuntimeError):
    """Raised when the scenario runner never took ownership of the CARLA world."""


class ScenarioEgoMissing(RuntimeError):
    """Raised when the scenario never placed an ego for this node to attach to."""


def _active_map(client):
    """Return ``(name_lower, could_not_read)`` for the world's active map."""
    try:
        return client.get_world().get_map().name.split("/")[-1].lower(), False
    except RuntimeError:
        # CARLA 0.10 levels can expose no parseable OpenDRIVE metadata.
        return None, True


def _sync_enabled(world) -> bool:
    """Whether synchronous mode is on (the runner enabling it = it owns the world)."""
    try:
        return bool(world.get_settings().synchronous_mode)
    except RuntimeError:
        return False


def _observed_tick(world) -> bool:
    """Whether an external tick arrived within a short wait (runner is driving)."""
    try:
        world.wait_for_tick(2.0)
        return True
    except RuntimeError:
        return False


def _ownership(client, world, expected: str):
    """Return ``(owned, active_map, sync_enabled)`` for one look at the world.

    *owned* holds when the active map is the expected one -- skipped when its
    name cannot be read -- and synchronous mode is on.
    """
    current, unreadable = _active_map(client)
    sync_enabled = _sync_enabled(world)
    return (unreadable or current == expected) and sync_enabled, current, sync_enabled


def _not_owned_message(timeout: float, expected: str, current, sync_enabled: bool) -> str:
    """Return the message for a runner that never claimed the world."""
    return (
        f"The scenario runner did not take ownership of the CARLA world within "
        f"{timeout:.0f}s (expected map '{expected}', active map: {current}, "
        f"synchronous mode: {sync_enabled}). Refusing to start on a world nobody "
        "claimed: it would be the async default world or the previous episode's, "
        "so the run would drive the wrong map or a clock nobody advances. Check "
        "that the scenario runner is running, that its map matches carla_map, and "
        "raise scenario_world_wait_timeout if the runner simply needs longer."
    )


def wait_for_external_world(client, expected_map: str, timeout: float, logger):
    """Wait until the scenario runner is driving its world, then return it.

    The runner loads the map, destroys leftover actors (exempting the ego role),
    enables synchronous mode, spawns the "Ego" actor, and drives the clock.
    Adopt only once three signals hold together:
    the active map is the expected one (skipped when its name cannot be read),
    synchronous mode is on (this node never enables it in scenario mode, so that
    means the runner owns the world), and an external tick is observed (the runner
    is actively driving, so a wait_for_tick spawn is applied, not deadlocked).

    Requiring synchronous mode - not just a tick - rules out the async default
    world CARLA starts on, whose free-running ticks would otherwise cause a
    premature adopt/spawn into the wrong (soon-reloaded) world.

    On timeout this raises rather than adopting whatever world is up. The world
    that is up when the runner has not claimed one is the async default CARLA
    starts on, or the previous episode's: adopting it starts the run on the
    wrong map, or under a clock nobody drives, and the scenario is then scored
    against a road it never drove. A startup that fails here says so; one that
    adopts says so only in a log line nobody reads until the result is wrong.

    Raises:
        ScenarioWorldNotOwned: If the runner has not claimed the world within
            *timeout*.
    """
    expected = expected_map.split("/")[-1].lower()
    deadline = time.time() + max(float(timeout), 1.0)
    logger.info(
        f"Scenario mode: waiting for the scenario runner to drive its world "
        f"(expected map '{expected}'); not loading the world here (the runner owns it)."
    )
    while True:
        world = client.get_world()
        owned, current, sync_enabled = _ownership(client, world, expected)
        if owned and _observed_tick(world):
            logger.info(f"Adopted the scenario runner's live CARLA world (map '{current}').")
            return world
        if time.time() >= deadline:
            message = _not_owned_message(timeout, expected, current, sync_enabled)
            logger.error(message)
            raise ScenarioWorldNotOwned(message)
        if not owned:
            time.sleep(1.0)


def find_ego_actor(world, role_name: str):
    """Return the vehicle carrying *role_name*, if one is in *world*."""
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def attach_to_scenario_ego(world, role_name: str, timeout: float, logger):
    """Wait for the ego the scenario placed and adopt it.

    The wait is bounded so a scenario that never places an ego cannot hang the
    startup. It fails instead of falling back to spawning one here: starting
    from the interface's own spawn point would run a different scenario than
    the one that was asked for, and silently so.

    Returns:
        The scenario's ego actor.

    Raises:
        ScenarioEgoMissing: If no such actor appears within *timeout*.
    """
    deadline = time.time() + timeout
    while True:
        actor = find_ego_actor(world, role_name)
        if actor is not None:
            logger.info(f"Attached to the scenario's ego: id={actor.id} role_name='{role_name}'")
            return actor
        if time.time() >= deadline:
            raise ScenarioEgoMissing(
                f"No actor with role_name='{role_name}' appeared within {timeout:.1f}s. "
                "In scenario mode the scenario places the ego and this node attaches "
                "to it; check that the scenario runner started and reached its ego spawn."
            )
        time.sleep(EGO_ATTACH_POLL_INTERVAL_S)
