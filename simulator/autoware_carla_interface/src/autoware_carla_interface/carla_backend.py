# Copyright 2026 Tier IV, Inc.
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

"""Select the CARLA client library used by the bridge.

By default the official ``carla`` Python package is used. Setting the
environment variable ``AUTOWARE_CARLA_USE_TYPESAFE_CARLA`` to a truthy value
(``1``, ``true``, ``yes`` or ``on``) switches to typesafe_carla
(``import typesafe_carla.carla``), which provides the same API names.

Usage::

    from autoware_carla_interface.carla_backend import carla
"""

import os

USE_TYPESAFE_CARLA_ENV = "AUTOWARE_CARLA_USE_TYPESAFE_CARLA"


def use_typesafe_carla() -> bool:
    """Return True when the environment variable selects typesafe_carla."""
    value = os.environ.get(USE_TYPESAFE_CARLA_ENV, "")
    return value.strip().lower() in ("1", "true", "yes", "on")


if use_typesafe_carla():
    import typesafe_carla.carla as carla  # noqa: F401
else:
    import carla  # noqa: F401

__all__ = ["carla", "use_typesafe_carla", "USE_TYPESAFE_CARLA_ENV"]
