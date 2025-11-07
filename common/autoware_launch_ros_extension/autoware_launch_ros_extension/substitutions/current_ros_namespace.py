# Copyright 2025 The Autoware Contributors
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Sequence
from typing import Text

from launch.frontend import expose_substitution
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution


@expose_substitution("current-ros-namespace")
class CurrentRosNamespace(Substitution):
    def __init__(self):
        super().__init__()

    @classmethod
    def parse(cls, data: Sequence[SomeSubstitutionsType]):
        if len(data) != 0:
            raise TypeError("current-ros-namespace substitution expects 0 arguments")
        return cls, {}

    def describe(self) -> Text:
        return f"{self.__class__.__name__}()"

    def perform(self, context: LaunchContext) -> Text:
        return context.launch_configurations.get("ros_namespace", "")
