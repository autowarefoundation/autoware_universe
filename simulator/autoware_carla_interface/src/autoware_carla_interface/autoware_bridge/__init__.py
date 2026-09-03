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

"""Autoware side of the CARLA scenario ``AutowareBridge``.

Splatsim-consistent topology: the scenario library hosts the ``AutowareBridge``
gRPC server and this package is the client that dials it (mirroring
``splatsim/grpc_client.py`` against a Docker-launched server).  The client pulls
the scenario's mission (initial pose + goal) via ``GetMission`` and pushes a
single readiness flag via ``ReportReadiness``.

The heavy lifting -- localization init, routing, engage, and readiness
aggregation -- lives here on the Autoware side (see :mod:`.node`), driven through
the Autoware AD API.  The scenario framework stays free of any ROS 2 dependency.
"""
