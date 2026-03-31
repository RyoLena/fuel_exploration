#pragma once

#include <array>
#include <cstdint>
#include <utility>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace fuel_exploration {

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

inline constexpr int8_t kFreeCell = 0;
inline constexpr int8_t kUnknownCell = -1;

inline constexpr std::array<std::pair<int, int>, 8> kNeighborhood{
    std::pair<int, int>{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1},                      {1, -1}, {1, 0},  {1, 1}};

int flatten_index(int row, int col, int width);

bool is_inside_map(int row, int col, const OccupancyGrid &map);

bool world_to_grid(double x, double y, const OccupancyGrid &map, int &row,
                   int &col);

std::pair<double, double> grid_to_world(int row, int col,
                                        const OccupancyGrid &map);

bool is_occupied(int8_t value, int occupied_threshold);

}  // namespace fuel_exploration
