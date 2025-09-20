# Copyright 2024 Proxima Technology Inc, TIER IV
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

"""Class for training neural nets from driving data."""
from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
from autoware_smart_mpc_trajectory_follower.training_and_data_check import (
    train_drive_NN_model_with_memory,
)
from autoware_smart_mpc_trajectory_follower.training_and_data_check import (
    train_drive_NN_model_without_memory,
)

if drive_functions.use_memory_for_training:
    train_drive_NN_model = train_drive_NN_model_with_memory.train_drive_NN_model_with_memory
else:
    train_drive_NN_model = train_drive_NN_model_without_memory.train_drive_NN_model_without_memory
