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

#include <gtest/gtest.h>

#include <limits>

#include "mfja_yaskawa_gz_hardware/velocity_command.hpp"

using mfja_yaskawa_gz_hardware::velocity_command;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kGain = 10.0;
}  // namespace

TEST(VelocityCommand, HoldsWhenNothingIsClaimed)
{
  EXPECT_DOUBLE_EQ(velocity_command(0.3, false, 1.0, false, 0.5, kGain), 0.0);
}

TEST(VelocityCommand, HoldsBeforeTheFirstCommand)
{
  EXPECT_DOUBLE_EQ(velocity_command(0.3, true, kNaN, true, kNaN, kGain), 0.0);
}

TEST(VelocityCommand, PositionAndVelocityLikeMotoROS2)
{
  // Feedforward 0.2 rad/s plus correction of 0.01 rad at 10 1/s.
  EXPECT_NEAR(velocity_command(0.50, true, 0.51, true, 0.2, kGain), 0.3, 1e-12);
  // On the reference: the commanded velocity only.
  EXPECT_DOUBLE_EQ(velocity_command(0.5, true, 0.5, true, 0.2, kGain), 0.2);
}

TEST(VelocityCommand, PositionOnly)
{
  EXPECT_NEAR(velocity_command(0.0, true, -0.1, false, 0.4, kGain), -1.0, 1e-12);
}

TEST(VelocityCommand, VelocityOnly)
{
  EXPECT_DOUBLE_EQ(velocity_command(0.0, false, 3.0, true, -0.25, kGain), -0.25);
}

TEST(VelocityCommand, UnknownPositionIsNotCorrected)
{
  EXPECT_DOUBLE_EQ(velocity_command(kNaN, true, 0.1, true, 0.2, kGain), 0.2);
}
