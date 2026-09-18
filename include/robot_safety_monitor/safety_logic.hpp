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
    const float angle = angle_min + static_cast<float>(index) * angle_increment;
    const float range = ranges[index];

    const bool inside_sector = std::abs(angle) <= half_field_of_view_rad;
    const bool valid_range = std::isfinite(range) && range >= range_min && range <= range_max;
    if (inside_sector && valid_range) {
      minimum = std::min(minimum, range);
    }
  }

  return minimum;
}

}  // namespace robot_safety_monitor
