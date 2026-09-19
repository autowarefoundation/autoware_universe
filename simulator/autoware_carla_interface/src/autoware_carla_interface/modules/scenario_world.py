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

"""Adopt the CARLA world the scenario runner owns (scenario mode).

Extracted from ``carla_autoware`` so the world-handoff logic lives on its own:
in scenario mode the interface does not load the world, it waits for the
``autoware_carla_scenario`` runner to bring one up and then adopts it. See
``wait_for_external_world`` for the handoff contract.
"""

from __future__ import annotations

import time


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


def wait_for_external_world(client, expected_map: str, timeout: float, logger):
    """Wait until the scenario runner is driving its world, then return it.

    The runner loads the map, destroys leftover actors (exempting the ego role),
    enables synchronous mode, and then drives the clock while it waits for this
    node to spawn the "Ego" actor. Adopt only once three signals hold together:
    the active map is the expected one (skipped when its name cannot be read),
    synchronous mode is on (this node never enables it in scenario mode, so that
    means the runner owns the world), and an external tick is observed (the runner
    is actively driving, so a wait_for_tick spawn is applied, not deadlocked).

    Requiring synchronous mode - not just a tick - rules out the async default
    world CARLA starts on, whose free-running ticks would otherwise cause a
    premature adopt/spawn into the wrong (soon-reloaded) world. On timeout, adopt
    whatever world is up so the bridge still starts, surfacing it in the log.
    """
    expected = expected_map.split("/")[-1].lower()
    deadline = time.time() + max(float(timeout), 1.0)
    logger.info(
        f"Scenario mode: waiting for the scenario runner to drive its world "
        f"(expected map '{expected}'); not loading the world here (the runner owns it)."
    )
    while True:
        current, unreadable = _active_map(client)
        owned = (unreadable or current == expected) and _sync_enabled(client.get_world())
        if owned and _observed_tick(client.get_world()):
            logger.info(f"Adopted the scenario runner's live CARLA world (map '{current}').")
            return client.get_world()
        if time.time() >= deadline:
            logger.warning(
                f"Timed out after {timeout:.0f}s waiting for the scenario runner "
                f"(active map: {current}, owned: {owned}); adopting the current world "
                "as-is. Check that with_scenario's map matches carla_map and the runner runs."
            )
            return client.get_world()
        if not owned:
            time.sleep(1.0)
