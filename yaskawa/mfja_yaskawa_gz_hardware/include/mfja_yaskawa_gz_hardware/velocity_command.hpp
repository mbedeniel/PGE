// Copyright 2026 Niel MBEDE
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

#ifndef MFJA_YASKAWA_GZ_HARDWARE__VELOCITY_COMMAND_HPP_
#define MFJA_YASKAWA_GZ_HARDWARE__VELOCITY_COMMAND_HPP_

#include <cmath>

namespace mfja_yaskawa_gz_hardware
{

/// Velocity (rad/s) applied to a simulated joint, computed from the commands of the controller.
/**
 * Imitates the servo of the robot controller following the points queued in MotoROS2, each of
 * which carries a position and a velocity: the commanded velocity is applied as a feedforward and
 * the position error is corrected with position_gain (1/s).
 *
 * A command interface that no controller has claimed, or that has no value yet (NaN), contributes
 * nothing. With no command at all the result is 0: the joint is held still, as the brakes hold the
 * real arm.
 */
inline double velocity_command(
  double position, bool position_claimed, double position_command,
  bool velocity_claimed, double commanded_velocity, double position_gain)
{
  double velocity = 0.0;
  if (position_claimed && std::isfinite(position_command) && std::isfinite(position)) {
    velocity += position_gain * (position_command - position);
  }
  if (velocity_claimed && std::isfinite(commanded_velocity)) {
    velocity += commanded_velocity;
  }
  return velocity;
}

}  // namespace mfja_yaskawa_gz_hardware

#endif  // MFJA_YASKAWA_GZ_HARDWARE__VELOCITY_COMMAND_HPP_
