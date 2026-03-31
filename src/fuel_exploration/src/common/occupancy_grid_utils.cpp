#include "fuel_exploration/common/occupancy_grid_utils.hpp"

#include <cmath>

namespace fuel_exploration {

int flatten_index(const int row, const int col, const int width) {
  return row * width + col;
}

bool is_inside_map(const int row, const int col, const OccupancyGrid &map) {
  return row >= 0 && row < static_cast<int>(map.info.height) && col >= 0 &&
         col < static_cast<int>(map.info.width);
}

bool world_to_grid(const double x, const double y, const OccupancyGrid &map,
                   int &row, int &col) {
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  col = static_cast<int>(std::floor((x - origin_x) / resolution));
  row = static_cast<int>(std::floor((y - origin_y) / resolution));
  return is_inside_map(row, col, map);
}

std::pair<double, double> grid_to_world(const int row, const int col,
                                        const OccupancyGrid &map) {
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;

  return {origin_x + (static_cast<double>(col) + 0.5) * resolution,
          origin_y + (static_cast<double>(row) + 0.5) * resolution};
}

bool is_occupied(const int8_t value, const int occupied_threshold) {
  return value >= occupied_threshold;
}

}  // namespace fuel_exploration
