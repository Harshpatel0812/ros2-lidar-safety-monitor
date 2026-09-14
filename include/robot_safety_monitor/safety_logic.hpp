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
