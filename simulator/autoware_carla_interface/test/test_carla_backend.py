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

# cspell:ignore delenv

"""Unit tests for selecting the CARLA client library with an environment variable."""

import importlib
import sys
import types

import pytest

ENV = "AUTOWARE_CARLA_USE_TYPESAFE_CARLA"
MODULE = "autoware_carla_interface.carla_backend"


def _reload_backend(monkeypatch, value):
    """Import carla_backend with stub CARLA modules and ENV set to value (None: unset)."""
    official = types.ModuleType("carla")
    typesafe_pkg = types.ModuleType("typesafe_carla")
    typesafe = types.ModuleType("typesafe_carla.carla")
    typesafe_pkg.carla = typesafe
    monkeypatch.setitem(sys.modules, "carla", official)
    monkeypatch.setitem(sys.modules, "typesafe_carla", typesafe_pkg)
    monkeypatch.setitem(sys.modules, "typesafe_carla.carla", typesafe)
    if value is None:
        monkeypatch.delenv(ENV, raising=False)
    else:
        monkeypatch.setenv(ENV, value)
    monkeypatch.delitem(sys.modules, MODULE, raising=False)
    backend = importlib.import_module(MODULE)
    return backend, official, typesafe


@pytest.mark.parametrize("value", [None, "", "0", "false", "off", "no", "typesafe"])
def test_official_carla_by_default(monkeypatch, value):
    backend, official, _ = _reload_backend(monkeypatch, value)
    assert not backend.use_typesafe_carla()
    assert backend.carla is official


@pytest.mark.parametrize("value", ["1", "true", "TRUE", "yes", "on", " On "])
def test_typesafe_carla_when_enabled(monkeypatch, value):
    backend, _, typesafe = _reload_backend(monkeypatch, value)
    assert backend.use_typesafe_carla()
    assert backend.carla is typesafe
