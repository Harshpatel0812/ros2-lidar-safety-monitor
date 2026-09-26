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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace robot_safety_monitor
{

inline float minimum_range_in_sector(
  const std::vector<float> & ranges,
  const float angle_min,
  const float angle_increment,
  const float range_min,
  const float range_max,
  const float half_field_of_view_rad)
{
  float minimum = std::numeric_limits<float>::infinity();

  for (std::size_t index = 0; index < ranges.size(); ++index) {
    const float angle =
      angle_min +
      static_cast<float>(index) * angle_increment;

    const float range = ranges[index];

    if (std::abs(angle) > half_field_of_view_rad) {
      continue;
    }

    if (!std::isfinite(range)) {
      continue;
    }

    if (range <= 0.0F) {
      continue;
    }

    if (range > range_max) {
      continue;
    }

    if (range < range_min) {
      // A finite positive return below the documented minimum range may
      // indicate an obstacle too close for reliable measurement. Treat it
      // conservatively as a hazard instead of discarding it.
      minimum = std::min(minimum, range);
      continue;
    }

    minimum = std::min(minimum, range);
  }

  return minimum;
}

inline double calculate_dynamic_stop_distance(
  const double base_stop_distance,
  const double commanded_forward_speed,
  const double reaction_time,
  const double braking_deceleration,
  const double maximum_stop_distance)
{
  const double forward_speed =
    std::max(0.0, commanded_forward_speed);

  const double reaction_distance =
    forward_speed * reaction_time;

  const double braking_distance =
    (forward_speed * forward_speed) /
    (2.0 * braking_deceleration);

  const double calculated_distance =
    base_stop_distance +
    reaction_distance +
    braking_distance;

  return std::min(
    calculated_distance,
    maximum_stop_distance);
}

}  // namespace robot_safety_monitor
