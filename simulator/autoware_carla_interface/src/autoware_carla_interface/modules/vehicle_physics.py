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

"""Write the configured physics onto the ego actor.

Extracted from ``carla_autoware`` so everything that rewrites the ego's
``carla.VehiclePhysicsControl`` lives in one place.

CARLA 0.10 gives every vehicle the same placeholder physics, which does not
match the vehicle Autoware plans for -- most visibly the wheels'
``max_steer_angle``, the very value the interface normalizes a commanded tire
angle by, and a corrupt ``steering_curve``. :func:`apply` writes the keys named
in ``vehicle_physics_config`` (see ``config/vehicle_physics.yaml``);
:func:`flatten_steering_curve` is the narrower, opt-in workaround for the curve
alone.
"""

from __future__ import annotations

import carla
import yaml


def _merge_wheels(base, override):
    """Return the per-axle wheel settings of *base* updated by *override*."""
    merged = {axle: dict(cfg) for axle, cfg in base.items()}
    for axle, cfg in override.items():
        merged[axle] = {**merged.get(axle, {}), **cfg}
    return merged


def _merge_settings(base, override):
    """Return *base* updated by *override*, one level deep for `wheels`."""
    merged = {**base, **override}
    wheels = override.get("wheels")
    if isinstance(wheels, dict):
        merged["wheels"] = _merge_wheels(base.get("wheels") or {}, wheels)
    return merged


def _load_document(path):
    """Return the parsed ``vehicle_physics_config`` at *path*, or None if unreadable."""
    try:
        with open(path) as config_file:
            return yaml.safe_load(config_file) or {}
    except OSError as error:
        print(f"WARNING: Cannot read vehicle_physics_config {path}: {error}")
    except yaml.YAMLError as error:
        print(f"WARNING: Invalid vehicle_physics_config {path}: {error}")
    return None


def _read_settings(path, blueprint_id):
    """Return the physics settings for *blueprint_id*, or None if there are none."""
    document = _load_document(path)
    if document is None:
        return None
    vehicles = document.get("vehicles") or {}
    settings = _merge_settings(document.get("default") or {}, vehicles.get(blueprint_id) or {})
    return settings or None


def _is_front(wheels):
    """Return, per wheel, whether it sits on the steered (front) axle.

    The steered wheels are the ones the server reports a non-zero
    max_steer_angle for, so "front" keeps meaning the steered axle even after
    this has been applied once. A vehicle that steers nothing (yet) falls back
    to the first half of the list.
    """
    steered = [wheel.max_steer_angle > 0.0 for wheel in wheels]
    if any(steered):
        return steered
    return [index < len(wheels) / 2 for index in range(len(wheels))]


def _write_wheel(wheel, settings):
    """Write one axle's settings onto one wheel."""
    for key, value in settings.items():
        if hasattr(wheel, key):
            setattr(wheel, key, float(value))
        else:
            print(f"WARNING: Unknown wheel physics key '{key}'; skipped.")


def _apply_wheel_settings(physics, wheel_settings):
    """Write the front/rear wheel settings onto *physics*; return the wheel list."""
    wheels = list(physics.wheels)
    for wheel, is_front in zip(wheels, _is_front(wheels)):
        _write_wheel(wheel, wheel_settings.get("front" if is_front else "rear") or {})
    return wheels


def _apply_steer_normalization(interface, settings, path):
    """Take `steer_normalization_deg` out of *settings* and give it to the interface.

    It is the one key read rather than written: CARLA 0.10 reports 70 deg of
    steer for every car and ignores writes to a wheel's max_steer_angle, so the
    angle a commanded tire angle is normalized by has to be configured. An
    explicit ``max_wheel_steer_angle_deg`` wins over the file.
    """
    steer_deg = settings.pop("steer_normalization_deg", None)
    if steer_deg is None:
        return
    if float(interface.param_values.get("max_wheel_steer_angle_deg", 0.0)) > 0.0:
        return
    # Goes through the interface so the cached angle derived from the old value is
    # dropped under its state lock; a plain param_values write would be ignored by
    # a control_callback that had already cached CARLA's own 70 deg.
    interface.set_steer_normalization_deg(steer_deg)
    print(f"INFO: Steer normalization set to {float(steer_deg):.1f} deg from {path}.")


def _write_one(physics, key, value) -> bool:
    """Write one setting onto *physics*; return whether the key was known."""
    if key == "wheels":
        if not isinstance(value, dict):
            # Anything else (a list of axle names, a scalar) would reach
            # dict.get() on a non-mapping and raise AttributeError, which the
            # caller's except clause does not cover -- a typo in the YAML would
            # take the whole interface node down at startup.
            print("WARNING: vehicle physics 'wheels' must be a mapping of axle -> settings.")
            return False
        physics.wheels = _apply_wheel_settings(physics, value)
    elif key == "steering_curve":
        physics.steering_curve = [carla.Vector2D(float(x), float(y)) for x, y in value]
    elif hasattr(physics, key):
        setattr(physics, key, float(value))
    else:
        print(f"WARNING: Unknown vehicle physics key '{key}'; skipped.")
        return False
    return True


def _write_settings(physics, settings):
    """Write *settings* onto *physics*; return the keys that were written."""
    return [key for key, value in settings.items() if _write_one(physics, key, value)]


def _config_path(interface) -> str:
    """Return the config to apply, or "" when the ego's physics are left alone.

    The config values (45.5 deg steer normalization, a flat steering curve,
    Lincoln mass/wheel radius) are calibrated for the 0.10 placeholder physics,
    so this is a no-op on CARLA 0.9.x -- whose vehicles already have their own
    correct physics -- to avoid changing steering gain and dynamics there.
    """
    path = str(interface.param_values.get("vehicle_physics_config", "")).strip()
    if not path:
        return ""
    if interface.uses_chaos_physics:
        return path
    print(
        "INFO: Skipping vehicle_physics_config on CARLA "
        f"{interface.carla_version}: it is calibrated for the 0.10 "
        "placeholder physics and only applied on CARLA 0.10+."
    )
    return ""


def apply(ego_actor, interface):
    """Apply the configured physics to the ego, so it moves like the modelled car.

    Reads ``vehicle_physics_config`` (empty, or a CARLA older than 0.10,
    disables this) and writes only the keys it names.
    """
    path = _config_path(interface)
    if not path:
        return
    settings = _read_settings(path, ego_actor.type_id)
    if not settings:
        print(f"INFO: No vehicle physics for {ego_actor.type_id} in {path}.")
        return

    _apply_steer_normalization(interface, settings, path)
    try:
        physics = ego_actor.get_physics_control()
        applied = _write_settings(physics, settings)
        ego_actor.apply_physics_control(physics)
        interface.set_physics_control(ego_actor.get_physics_control())
        print(
            f"INFO: Applied vehicle physics from {path} to {ego_actor.type_id} "
            f"({', '.join(applied) or 'nothing'})."
        )
    except (RuntimeError, TypeError, ValueError) as error:
        print(f"WARNING: Failed to apply vehicle physics from {path}: {error}")


def flatten_steering_curve(ego_actor, interface):
    """Replace the vehicle's speed-based steering curve with an identity curve.

    CARLA 0.10 ships corrupt steering-curve data (duplicated, unsorted
    points such as (10, 0.5); the curve's speed axis is mph on Chaos) which
    the simulator applies internally, attenuating the achievable steering
    angle at driving speeds. Writing a flat curve back removes the
    server-side attenuation so the commanded steer fraction maps directly to
    the wheel angle.
    """
    try:
        physics = ego_actor.get_physics_control()
        physics.steering_curve = [
            carla.Vector2D(0.0, 1.0),
            carla.Vector2D(120.0, 1.0),
        ]
        ego_actor.apply_physics_control(physics)
        interface.set_physics_control(physics)
        print("INFO: Applied a flat steering curve to the ego vehicle.")
    except RuntimeError as error:
        print(f"WARNING: Failed to flatten the steering curve: {error}")
