// Copyright 2026 Harsh Patel
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

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "robot_safety_monitor/safety_logic.hpp"

namespace
{

constexpr float kPi = 3.14159265358979323846F;

float evaluate(const std::vector<float> & ranges)
{
  return robot_safety_monitor::minimum_range_in_sector(
    ranges,
    -kPi / 2.0F,
    kPi / 4.0F,
    0.1F,
    10.0F,
    kPi / 4.0F);
}

double dynamic_distance(const double speed)
{
  return robot_safety_monitor::calculate_dynamic_stop_distance(
    0.45,
    speed,
    0.20,
    0.80,
    1.50);
}

}  // namespace

TEST(SafetyLogic, FindsClosestFrontalObstacle)
{
  EXPECT_FLOAT_EQ(
    evaluate({3.0F, 2.0F, 0.4F, 1.0F, 3.0F}),
    0.4F);
}

TEST(SafetyLogic, IgnoresObstacleOutsideFieldOfView)
{
  EXPECT_FLOAT_EQ(
    evaluate({0.2F, 2.0F, 3.0F, 2.0F, 0.3F}),
    2.0F);
}

TEST(SafetyLogic, IgnoresNonFiniteAndNonPositiveRanges)
{
  const float nan =
    std::numeric_limits<float>::quiet_NaN();

  const float infinity =
    std::numeric_limits<float>::infinity();

  EXPECT_FLOAT_EQ(
    evaluate({nan, 0.0F, 1.0F, -0.5F, infinity}),
    1.0F);
}

TEST(SafetyLogic, TreatsPositiveBelowMinimumRangeAsHazard)
{
  EXPECT_FLOAT_EQ(
    evaluate({3.0F, 2.0F, 0.05F, 1.0F, 3.0F}),
    0.05F);
}

TEST(SafetyLogic, ReturnsInfinityWhenNoValidReadingExists)
{
  const float nan =
    std::numeric_limits<float>::quiet_NaN();

  EXPECT_TRUE(
    std::isinf(
      evaluate({nan, nan, nan, nan, nan})));
}

TEST(DynamicStopDistance, ReturnsBaseDistanceAtZeroSpeed)
{
  EXPECT_DOUBLE_EQ(dynamic_distance(0.0), 0.45);
}

TEST(DynamicStopDistance, TreatsReverseSpeedAsZero)
{
  EXPECT_DOUBLE_EQ(dynamic_distance(-0.50), 0.45);
}

TEST(DynamicStopDistance, IncludesReactionAndBrakingDistance)
{
  const double expected =
    0.45 +
    (0.50 * 0.20) +
    ((0.50 * 0.50) / (2.0 * 0.80));

  EXPECT_NEAR(
    dynamic_distance(0.50),
    expected,
    1.0e-9);
}

TEST(DynamicStopDistance, IncreasesAsForwardSpeedIncreases)
{
  EXPECT_GT(
    dynamic_distance(0.50),
    dynamic_distance(0.20));
}

TEST(DynamicStopDistance, RespectsMaximumDistance)
{
  EXPECT_DOUBLE_EQ(dynamic_distance(3.0), 1.50);
}
