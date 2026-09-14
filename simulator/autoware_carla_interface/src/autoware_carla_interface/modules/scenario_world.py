"""Adopt the CARLA world the scenario runner owns (scenario mode).

Extracted from ``carla_autoware`` so the world-handoff logic lives on its own:
in scenario mode the interface does not load the world, it waits for the
``autoware_carla_scenario`` runner to bring one up and then adopts it. See
``wait_for_external_world`` for the handoff contract.
"""

from __future__ import annotations

import time
from typing import Callable


def _runner_world_signals(client, expected, query_world_map):
    """Return ``(map_ok, sync_on, ticking, current_map_name)`` for the live world.

    * ``map_ok`` - the active map is the expected one, or its name could not be
      read (CARLA 0.10 levels expose no OpenDRIVE metadata; fall back to the other
      signals then).
    * ``sync_on`` - synchronous mode is enabled; the interface never enables it in
      scenario mode, so this can only be the runner having taken ownership.
    * ``ticking`` - an external tick was observed, i.e. the runner is actively
      driving the world (so a runtime-init / wait_for_tick spawn will be applied).
    """
    world = client.get_world()
    current, query_failed = query_world_map(client)
    map_ok = query_failed or (current is not None and current.lower() == expected)
    sync_on = False
    ticking = False
    if map_ok:
        try:
            sync_on = world.get_settings().synchronous_mode
        except RuntimeError:
            sync_on = False
    if map_ok and sync_on:
        try:
            # Blocks until the runner ticks; times out when nothing drives it yet.
            world.wait_for_tick(2.0)
            ticking = True
        except RuntimeError:
            ticking = False
    return map_ok, sync_on, ticking, current


def wait_for_external_world(
    client,
    carla_map: str,
    timeout: float,
    logger,
    query_world_map: Callable,
    normalize_map_name: Callable[[str], str],
):
    """Wait until the scenario runner is driving its world, then return it.

    The runner loads the map, destroys leftover actors (exempting the ego role),
    enables synchronous mode, and then drives the clock while it waits for this
    node to spawn the "Ego" actor. Adopt only once all three signals hold together
    (see :func:`_runner_world_signals`): requiring synchronous mode - not just a
    tick - rules out the async default world CARLA starts on, whose free-running
    ticks would otherwise cause a premature adopt/spawn into the wrong (soon
    reloaded) world. On timeout, adopt whatever world is up so the bridge still
    starts, surfacing the misconfiguration in the log.
    """
    expected = normalize_map_name(carla_map).lower()
    deadline = time.time() + max(float(timeout), 1.0)
    logger.info(
        "Scenario mode: waiting for the scenario runner to drive its world "
        f"(expected map '{expected}') before spawning the ego; not loading the "
        "world here (the runner owns it)."
    )
    while True:
        map_ok, sync_on, ticking, current = _runner_world_signals(
            client, expected, query_world_map
        )
        if map_ok and sync_on and ticking:
            logger.info(
                "Adopted the scenario runner's live CARLA world (map "
                f"'{current if current is not None else 'unknown'}', synchronous)."
            )
            return client.get_world()
        if time.time() >= deadline:
            logger.warning(
                f"Timed out after {timeout:.0f}s waiting for the scenario runner "
                f"(active map: {current}, synchronous: {sync_on}, external tick: "
                f"{ticking}); adopting the current world as-is. Check that "
                "with_scenario's map matches carla_map and that the runner is running."
            )
            return client.get_world()
        if not (map_ok and sync_on):
            time.sleep(1.0)
