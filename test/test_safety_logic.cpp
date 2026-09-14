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
    ranges, -kPi / 2.0F, kPi / 4.0F, 0.1F, 10.0F, kPi / 4.0F);
}
}  // namespace

TEST(SafetyLogic, FindsClosestFrontalObstacle)
{
  EXPECT_FLOAT_EQ(evaluate({3.0F, 2.0F, 0.4F, 1.0F, 3.0F}), 0.4F);
}

TEST(SafetyLogic, IgnoresObstacleOutsideFieldOfView)
{
  EXPECT_FLOAT_EQ(evaluate({0.2F, 2.0F, 3.0F, 2.0F, 0.3F}), 2.0F);
}

TEST(SafetyLogic, IgnoresInvalidRanges)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  EXPECT_FLOAT_EQ(evaluate({nan, nan, 1.0F, 0.05F, infinity}), 1.0F);
}

TEST(SafetyLogic, ReturnsInfinityWhenNoValidReadingExists)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(std::isinf(evaluate({nan, nan, nan, nan, nan})));
}
