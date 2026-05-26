// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "interface.hpp"

namespace autoware::trajectory_gate
{

void TrajectorySender::set_output(TrajectoryReceiver * output)
{
  output_ = output;
}

void TrajectorySender::send(const Trajectory & msg)
{
  output_->receive(msg);
}

}  // namespace autoware::trajectory_gate
