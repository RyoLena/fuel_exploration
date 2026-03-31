#include <algorithm>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <limits>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <string>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr int8_t kFreeCell = 0;
constexpr int8_t kUnknownCell = -1;

struct ViewpointCandidate {
  double x;
  double y;
  double yaw;
  int score;
};

struct FrontierCluster {
  std::vector<int> cells;
  double centroid_x;
  double centroid_y;
  std::vector<ViewpointCandidate> viewpoints;
};

struct RobotPose {
  double x;
  double y;
  double yaw;
  std::string frame_id;
};

struct SelectedGoal {
  ViewpointCandidate viewpoint;
  std::size_t cluster_index;
  std::size_t viewpoint_index;
  double utility;
  double distance;
  std::vector<int> path_indices;
};

struct PlannedPath {
  std::vector<int> indices;
  double length_m;
};

struct VisitedViewpoint {
  double x;
  double y;
};

const std::array<std::pair<int, int>, 8> kNeighborhood{
    std::pair<int, int>{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1},                     {1, -1},  {1, 0},  {1, 1}};

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

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &quaternion) {
  const double siny_cosp =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw / 2.0);
  quaternion.w = std::cos(yaw / 2.0);
  return quaternion;
}

bool is_near_visited_viewpoint(
    const ViewpointCandidate &candidate,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    const double revisit_block_radius_m) {
  for (const auto &visited_viewpoint : visited_viewpoints) {
    if (std::hypot(candidate.x - visited_viewpoint.x,
                   candidate.y - visited_viewpoint.y) <=
        revisit_block_radius_m) {
      return true;
    }
  }
  return false;
}

bool has_free_clearance(const int row, const int col, const OccupancyGrid &map,
                        const int clearance_cells) {
  if (!is_inside_map(row, col, map)) {
    return false;
  }

  for (int dr = -clearance_cells; dr <= clearance_cells; ++dr) {
    for (int dc = -clearance_cells; dc <= clearance_cells; ++dc) {
      if (dr * dr + dc * dc > clearance_cells * clearance_cells) {
        continue;
      }

      const int nr = row + dr;
      const int nc = col + dc;
      if (!is_inside_map(nr, nc, map)) {
        return false;
      }

      if (map.data[flatten_index(nr, nc, map.info.width)] != kFreeCell) {
        return false;
      }
    }
  }

  return true;
}

bool is_path_traversable(const int row, const int col, const OccupancyGrid &map,
                         const int occupied_threshold,
                         const bool block_unknown,
                         const int clearance_cells) {
  if (!is_inside_map(row, col, map)) {
    return false;
  }

  const int8_t value = map.data[flatten_index(row, col, map.info.width)];
  if (is_occupied(value, occupied_threshold) ||
      (block_unknown && value == kUnknownCell)) {
    return false;
  }

  if (clearance_cells <= 0) {
    return value == kFreeCell || (!block_unknown && value == kUnknownCell);
  }

  return has_free_clearance(row, col, map, clearance_cells);
}

std::optional<int>
snap_to_nearest_traversable_index(const int seed_row, const int seed_col,
                                  const OccupancyGrid &map,
                                  const int occupied_threshold,
                                  const bool block_unknown,
                                  const int clearance_cells,
                                  const int max_search_radius) {
  if (is_path_traversable(seed_row, seed_col, map, occupied_threshold,
                          block_unknown, clearance_cells)) {
    return flatten_index(seed_row, seed_col, map.info.width);
  }

  for (int radius = 1; radius <= max_search_radius; ++radius) {
    for (int dr = -radius; dr <= radius; ++dr) {
      for (int dc = -radius; dc <= radius; ++dc) {
        if (std::max(std::abs(dr), std::abs(dc)) != radius) {
          continue;
        }

        const int row = seed_row + dr;
        const int col = seed_col + dc;
        if (!is_inside_map(row, col, map)) {
          continue;
        }

        if (is_path_traversable(row, col, map, occupied_threshold, block_unknown,
                                clearance_cells)) {
          return flatten_index(row, col, map.info.width);
        }
      }
    }
  }

  return std::nullopt;
}

std::optional<PlannedPath> plan_astar_path(
    const RobotPose &robot_pose, const ViewpointCandidate &goal_viewpoint,
    const OccupancyGrid &map, const int occupied_threshold,
    const bool block_unknown, const double path_clearance_m) {
  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!world_to_grid(robot_pose.x, robot_pose.y, map, start_row, start_col) ||
      !world_to_grid(goal_viewpoint.x, goal_viewpoint.y, map, goal_row, goal_col)) {
    return std::nullopt;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int cell_count = width * height;
  const int clearance_cells =
      std::max(0, static_cast<int>(std::ceil(path_clearance_m / map.info.resolution)));
  const int max_snap_radius = std::max(2, clearance_cells + 2);

  const auto start_index = snap_to_nearest_traversable_index(
      start_row, start_col, map, occupied_threshold, block_unknown,
      clearance_cells, max_snap_radius);
  const auto goal_index = snap_to_nearest_traversable_index(
      goal_row, goal_col, map, occupied_threshold, block_unknown,
      clearance_cells, max_snap_radius);
  if (!start_index.has_value() || !goal_index.has_value()) {
    return std::nullopt;
  }

  struct OpenNode {
    double f_score;
    double g_score;
    int index;
  };

  struct CompareOpenNode {
    bool operator()(const OpenNode &lhs, const OpenNode &rhs) const {
      return lhs.f_score > rhs.f_score;
    }
  };

  const auto heuristic = [width](const int from_index, const int to_index) {
    const int from_row = from_index / width;
    const int from_col = from_index % width;
    const int to_row = to_index / width;
    const int to_col = to_index % width;
    return std::hypot(static_cast<double>(to_row - from_row),
                      static_cast<double>(to_col - from_col));
  };

  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open_set;
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  g_score[*start_index] = 0.0;
  open_set.push(OpenNode{heuristic(*start_index, *goal_index), 0.0, *start_index});

  while (!open_set.empty()) {
    const OpenNode current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;

    if (current.index == *goal_index) {
      PlannedPath path;
      path.length_m = g_score[current.index] * map.info.resolution;

      int trace_index = current.index;
      while (trace_index != -1) {
        path.indices.push_back(trace_index);
        trace_index = parent[trace_index];
      }
      std::reverse(path.indices.begin(), path.indices.end());
      return path;
    }

    const int row = current.index / width;
    const int col = current.index % width;
    for (const auto &[dr, dc] : kNeighborhood) {
      const int next_row = row + dr;
      const int next_col = col + dc;
      if (!is_path_traversable(next_row, next_col, map, occupied_threshold,
                               block_unknown, clearance_cells)) {
        continue;
      }

      const int next_index = flatten_index(next_row, next_col, width);
      if (closed[next_index]) {
        continue;
      }

      const double step_cost =
          std::hypot(static_cast<double>(dr), static_cast<double>(dc));
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      parent[next_index] = current.index;
      g_score[next_index] = tentative_g;
      open_set.push(OpenNode{tentative_g + heuristic(next_index, *goal_index),
                             tentative_g, next_index});
    }
  }

  return std::nullopt;
}

std::optional<double> compute_path_cost(
    const double start_x, const double start_y,
    const double goal_x, const double goal_y,
    const OccupancyGrid &map, const int occupied_threshold,
    const bool block_unknown, const double path_clearance_m) {
  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!world_to_grid(start_x, start_y, map, start_row, start_col) ||
      !world_to_grid(goal_x, goal_y, map, goal_row, goal_col)) {
    return std::nullopt;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int cell_count = width * height;
  const int clearance_cells = std::max(
      0, static_cast<int>(std::ceil(path_clearance_m / map.info.resolution)));
  const int max_snap_radius = std::max(2, clearance_cells + 2);

  const auto start_index = snap_to_nearest_traversable_index(
      start_row, start_col, map, occupied_threshold, block_unknown,
      clearance_cells, max_snap_radius);
  const auto goal_index = snap_to_nearest_traversable_index(
      goal_row, goal_col, map, occupied_threshold, block_unknown,
      clearance_cells, max_snap_radius);
  if (!start_index.has_value() || !goal_index.has_value()) {
    return std::nullopt;
  }

  struct OpenNode {
    double f_score;
    double g_score;
    int index;
  };

  struct CompareOpenNode {
    bool operator()(const OpenNode &lhs, const OpenNode &rhs) const {
      return lhs.f_score > rhs.f_score;
    }
  };

  const auto heuristic = [width](const int from_index, const int to_index) {
    const int from_row = from_index / width;
    const int from_col = from_index % width;
    const int to_row = to_index / width;
    const int to_col = to_index % width;
    return std::hypot(static_cast<double>(to_row - from_row),
                      static_cast<double>(to_col - from_col));
  };

  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode>
      open_set;
  std::vector<double> g_score(cell_count,
                              std::numeric_limits<double>::infinity());
  std::vector<bool> closed(cell_count, false);

  g_score[*start_index] = 0.0;
  open_set.push(
      OpenNode{heuristic(*start_index, *goal_index), 0.0, *start_index});

  while (!open_set.empty()) {
    const OpenNode current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;

    if (current.index == *goal_index) {
      return g_score[current.index] * map.info.resolution;
    }

    const int row = current.index / width;
    const int col = current.index % width;
    for (const auto &[dr, dc] : kNeighborhood) {
      const int next_row = row + dr;
      const int next_col = col + dc;
      if (!is_path_traversable(next_row, next_col, map, occupied_threshold,
                               block_unknown, clearance_cells)) {
        continue;
      }

      const int next_index = flatten_index(next_row, next_col, width);
      if (closed[next_index]) {
        continue;
      }

      const double step_cost =
          std::hypot(static_cast<double>(dr), static_cast<double>(dc));
      const double tentative_g = g_score[current.index] + step_cost;
      if (tentative_g >= g_score[next_index]) {
        continue;
      }

      g_score[next_index] = tentative_g;
      open_set.push(OpenNode{tentative_g + heuristic(next_index, *goal_index),
                             tentative_g, next_index});
    }
  }

  return std::nullopt;
}

bool has_line_of_sight(const int start_row, const int start_col,
                       const int goal_row, const int goal_col,
                       const OccupancyGrid &map, const int occupied_threshold,
                       const bool block_unknown) {
  if (!is_inside_map(start_row, start_col, map) ||
      !is_inside_map(goal_row, goal_col, map)) {
    return false;
  }

  int current_row = start_row;
  int current_col = start_col;
  const int dy = std::abs(goal_row - start_row);
  const int dx = std::abs(goal_col - start_col);
  const int step_col = (start_col < goal_col) ? 1 : -1;
  const int step_row = (start_row < goal_row) ? 1 : -1;
  int error = dx - dy;

  while (current_row != goal_row || current_col != goal_col) {
    const int twice_error = 2 * error;
    if (twice_error > -dy) {
      error -= dy;
      current_col += step_col;
    }
    if (twice_error < dx) {
      error += dx;
      current_row += step_row;
    }

    if (!is_inside_map(current_row, current_col, map)) {
      return false;
    }
    if (current_row == goal_row && current_col == goal_col) {
      return true;
    }

    const int8_t value =
        map.data[flatten_index(current_row, current_col, map.info.width)];
    if (is_occupied(value, occupied_threshold) ||
        (block_unknown && value == kUnknownCell)) {
      return false;
    }
  }

  return true;
}

int score_candidate(const FrontierCluster &cluster,
                    const ViewpointCandidate &candidate,
                    const OccupancyGrid &map, const double max_view_range_m,
                    const int occupied_threshold,
                    const bool block_unknown) {
  int candidate_row = 0;
  int candidate_col = 0;
  if (!world_to_grid(candidate.x, candidate.y, map, candidate_row,
                     candidate_col)) {
    return 0;
  }

  int score = 0;
  const int width = static_cast<int>(map.info.width);
  for (const int idx : cluster.cells) {
    const int frontier_row = idx / width;
    const int frontier_col = idx % width;
    const auto frontier_world = grid_to_world(frontier_row, frontier_col, map);
    if (std::hypot(frontier_world.first - candidate.x,
                   frontier_world.second - candidate.y) > max_view_range_m) {
      continue;
    }

    if (has_line_of_sight(candidate_row, candidate_col, frontier_row,
                          frontier_col, map, occupied_threshold,
                          block_unknown)) {
      ++score;
    }
  }

  return score;
}

std::vector<ViewpointCandidate> generate_candidates(
    const FrontierCluster &cluster, const OccupancyGrid &map,
    const std::vector<double> &sample_radii_m, const int angular_samples,
    const double candidate_clearance_m, const double max_view_range_m,
    const int min_viewpoint_coverage, const int max_viewpoints,
    const int occupied_threshold, const bool block_unknown) {
  const auto sample_candidates =
      [&](const std::vector<double> &radii_m,
          const int clearance_cells) -> std::vector<ViewpointCandidate> {
    std::vector<ViewpointCandidate> scored_candidates;
    std::set<int> visited_candidate_cells;

    for (const double radius : radii_m) {
      if (radius <= 0.0) {
        continue;
      }

      for (int sample_idx = 0; sample_idx < angular_samples; ++sample_idx) {
        const double theta =
            kTwoPi * static_cast<double>(sample_idx) / angular_samples;
        const double x = cluster.centroid_x + radius * std::cos(theta);
        const double y = cluster.centroid_y + radius * std::sin(theta);

        int row = 0;
        int col = 0;
        if (!world_to_grid(x, y, map, row, col)) {
          continue;
        }

        const int cell_index = flatten_index(row, col, map.info.width);
        if (visited_candidate_cells.count(cell_index) > 0) {
          continue;
        }
        visited_candidate_cells.insert(cell_index);

        if (!has_free_clearance(row, col, map, clearance_cells)) {
          continue;
        }

        ViewpointCandidate candidate{
            x,
            y,
            std::atan2(cluster.centroid_y - y, cluster.centroid_x - x),
            0,
        };
        candidate.score =
            score_candidate(cluster, candidate, map, max_view_range_m,
                            occupied_threshold, block_unknown);
        if (candidate.score > 0) {
          scored_candidates.push_back(candidate);
        }
      }
    }

    return scored_candidates;
  };

  const int clearance_cells = std::max(
      0, static_cast<int>(std::ceil(candidate_clearance_m / map.info.resolution)));
  std::vector<ViewpointCandidate> scored_candidates =
      sample_candidates(sample_radii_m, clearance_cells);

  if (scored_candidates.empty() && clearance_cells > 0) {
    scored_candidates = sample_candidates(sample_radii_m, 0);
  }

  if (scored_candidates.empty()) {
    std::vector<double> relaxed_radii{0.35, 0.55, 0.8, 1.1, 1.5, 2.0};
    for (const double radius : sample_radii_m) {
      relaxed_radii.push_back(radius);
    }
    std::sort(relaxed_radii.begin(), relaxed_radii.end());
    relaxed_radii.erase(
        std::unique(relaxed_radii.begin(), relaxed_radii.end(),
                    [](const double lhs, const double rhs) {
                      return std::abs(lhs - rhs) < 1e-3;
                    }),
        relaxed_radii.end());
    scored_candidates = sample_candidates(relaxed_radii, 0);
  }

  std::sort(scored_candidates.begin(), scored_candidates.end(),
            [&cluster](const ViewpointCandidate &lhs,
                       const ViewpointCandidate &rhs) {
              if (lhs.score != rhs.score) {
                return lhs.score > rhs.score;
              }

              const double lhs_distance =
                  std::hypot(lhs.x - cluster.centroid_x, lhs.y - cluster.centroid_y);
              const double rhs_distance =
                  std::hypot(rhs.x - cluster.centroid_x, rhs.y - cluster.centroid_y);
              return lhs_distance < rhs_distance;
            });

  std::vector<ViewpointCandidate> selected_candidates;
  for (const auto &candidate : scored_candidates) {
    if (candidate.score < min_viewpoint_coverage) {
      continue;
    }
    selected_candidates.push_back(candidate);
    if (static_cast<int>(selected_candidates.size()) >= max_viewpoints) {
      break;
    }
  }

  if (selected_candidates.empty() && !scored_candidates.empty()) {
    selected_candidates.push_back(scored_candidates.front());
  }

  return selected_candidates;
}

// TSP 最近邻启发式：节点0=机器人，节点1..N=簇
// 返回访问顺序（簇索引，不含机器人）
std::vector<std::size_t> solve_tsp_nearest_neighbor(
    const std::vector<std::vector<double>> &cost_matrix) {
  const std::size_t n = cost_matrix.size();
  if (n <= 1) {
    return {};
  }

  std::vector<bool> visited(n, false);
  visited[0] = true; // 机器人是起点
  std::size_t current = 0;
  std::vector<std::size_t> order;

  for (std::size_t step = 1; step < n; ++step) {
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t best_next = 0;
    for (std::size_t j = 1; j < n; ++j) {
      if (!visited[j] && cost_matrix[current][j] < best_cost) {
        best_cost = cost_matrix[current][j];
        best_next = j;
      }
    }

    if (best_cost == std::numeric_limits<double>::infinity()) {
      // 剩余节点都不可达，跳过
      break;
    }

    visited[best_next] = true;
    current = best_next;
    order.push_back(best_next - 1); // 转换回簇索引(0-based)
  }

  return order;
}

// 第二层：Dijkstra 视点图搜索
// 从 TSP 排序的前 K 个簇中，为每个簇选一个视点，使总路径代价最小
// 返回每个簇选中的视点索引
struct DijkstraResult {
  std::vector<std::size_t> selected_viewpoint_indices; // 每个簇选的视点idx
  double total_cost;
};

std::optional<DijkstraResult> solve_viewpoint_dijkstra(
    const double robot_x, const double robot_y,
    const std::vector<FrontierCluster> &layer_clusters,
    const OccupancyGrid &map, const int occupied_threshold,
    const bool block_unknown, const double path_clearance_m,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    const double revisit_block_radius_m) {
  if (layer_clusters.empty()) {
    return std::nullopt;
  }

  const std::size_t num_layers = layer_clusters.size();

  // 每个节点 = (layer, viewpoint_index)
  // 用 Dijkstra 逐层松弛：layer 0 的各视点 → layer 1 的各视点 → ...
  // dist[layer][vi] = 从机器人经过 layer 0..layer 各选一个视点的最小总代价
  // prev[layer][vi] = 上一层选的视点索引

  std::vector<std::vector<double>> dist(num_layers);
  std::vector<std::vector<std::size_t>> prev(num_layers);

  // 初始化第 0 层：机器人到簇0各视点的代价
  const auto &first_cluster = layer_clusters[0];
  dist[0].resize(first_cluster.viewpoints.size(),
                 std::numeric_limits<double>::infinity());
  prev[0].resize(first_cluster.viewpoints.size(), 0);

  for (std::size_t vi = 0; vi < first_cluster.viewpoints.size(); ++vi) {
    const auto &vp = first_cluster.viewpoints[vi];
    if (is_near_visited_viewpoint(vp, visited_viewpoints,
                                  revisit_block_radius_m)) {
      continue;
    }
    const auto cost =
        compute_path_cost(robot_x, robot_y, vp.x, vp.y, map,
                          occupied_threshold, block_unknown, path_clearance_m);
    if (cost.has_value()) {
      dist[0][vi] = *cost;
    }
  }

  // 逐层松弛
  for (std::size_t layer = 1; layer < num_layers; ++layer) {
    const auto &prev_cluster = layer_clusters[layer - 1];
    const auto &curr_cluster = layer_clusters[layer];
    dist[layer].resize(curr_cluster.viewpoints.size(),
                       std::numeric_limits<double>::infinity());
    prev[layer].resize(curr_cluster.viewpoints.size(), 0);

    for (std::size_t ci = 0; ci < curr_cluster.viewpoints.size(); ++ci) {
      const auto &curr_vp = curr_cluster.viewpoints[ci];
      if (is_near_visited_viewpoint(curr_vp, visited_viewpoints,
                                    revisit_block_radius_m)) {
        continue;
      }

      for (std::size_t pi = 0; pi < prev_cluster.viewpoints.size(); ++pi) {
        if (dist[layer - 1][pi] ==
            std::numeric_limits<double>::infinity()) {
          continue;
        }

        const auto &prev_vp = prev_cluster.viewpoints[pi];
        const auto edge_cost = compute_path_cost(
            prev_vp.x, prev_vp.y, curr_vp.x, curr_vp.y, map,
            occupied_threshold, block_unknown, path_clearance_m);
        if (!edge_cost.has_value()) {
          continue;
        }

        const double total = dist[layer - 1][pi] + *edge_cost;
        if (total < dist[layer][ci]) {
          dist[layer][ci] = total;
          prev[layer][ci] = pi;
        }
      }
    }
  }

  // 在最后一层找最小代价节点
  const std::size_t last_layer = num_layers - 1;
  double best_cost = std::numeric_limits<double>::infinity();
  std::size_t best_vi = 0;
  for (std::size_t vi = 0; vi < dist[last_layer].size(); ++vi) {
    if (dist[last_layer][vi] < best_cost) {
      best_cost = dist[last_layer][vi];
      best_vi = vi;
    }
  }

  if (best_cost == std::numeric_limits<double>::infinity()) {
    return std::nullopt;
  }

  // 回溯：从最后一层往前，取出每层选的视点索引
  DijkstraResult result;
  result.total_cost = best_cost;
  result.selected_viewpoint_indices.resize(num_layers);
  result.selected_viewpoint_indices[last_layer] = best_vi;

  for (std::size_t layer = last_layer; layer > 0; --layer) {
    result.selected_viewpoint_indices[layer - 1] =
        prev[layer][result.selected_viewpoint_indices[layer]];
  }

  return result;
}

visualization_msgs::msg::Marker make_delete_all_marker(
    const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

} // namespace

class FrontierDetector : public rclcpp::Node {
public:
  FrontierDetector() : Node("frontier_detector") {
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 10);
    sample_radii_m_ =
        this->declare_parameter<std::vector<double>>("sample_radii_m",
                                                     {0.35, 0.55, 0.8, 1.1, 1.5});
    angular_samples_ = std::max<int>(
        1, this->declare_parameter<int>("angular_samples", 16));
    max_view_range_m_ = this->declare_parameter<double>("max_view_range_m", 3.5);
    candidate_clearance_m_ =
        this->declare_parameter<double>("candidate_clearance_m", 0.1);
    min_viewpoint_coverage_ = std::max<int>(
        1, this->declare_parameter<int>("min_viewpoint_coverage", 1));
    max_viewpoints_ = std::max<int>(
        1, this->declare_parameter<int>("max_viewpoints", 15));
    occupied_threshold_ =
        this->declare_parameter<int>("occupied_threshold", 50);
    block_unknown_in_los_ =
        this->declare_parameter<bool>("block_unknown_in_los", true);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    goal_topic_ =
        this->declare_parameter<std::string>("goal_topic", "/next_exploration_goal");
    path_topic_ =
        this->declare_parameter<std::string>("path_topic", "/next_exploration_path");
    trajectory_topic_ = this->declare_parameter<std::string>(
        "trajectory_topic", "/exploration_trajectory");
    goal_distance_weight_ =
        this->declare_parameter<double>("goal_distance_weight", 1.0);
    path_clearance_m_ =
        this->declare_parameter<double>("path_clearance_m", 0.15);
    block_unknown_in_path_ =
        this->declare_parameter<bool>("block_unknown_in_path", true);
    robot_pose_timeout_s_ =
        this->declare_parameter<double>("robot_pose_timeout_s", 0.1);
    min_goal_distance_m_ =
        this->declare_parameter<double>("min_goal_distance_m", 0.35);
    viewpoint_rank_penalty_ =
        this->declare_parameter<double>("viewpoint_rank_penalty", 0.05);
    goal_reached_tolerance_m_ =
        this->declare_parameter<double>("goal_reached_tolerance_m", 0.25);
    revisit_block_radius_m_ =
        this->declare_parameter<double>("revisit_block_radius_m", 0.45);
    trajectory_record_distance_m_ = std::max(
        0.0, this->declare_parameter<double>("trajectory_record_distance_m", 0.05));
    tsp_lookahead_ =
        this->declare_parameter<int>("tsp_lookahead", 4);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&FrontierDetector::map_callback, this,
                  std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&FrontierDetector::odom_callback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_topic_, 10);
    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    trajectory_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(trajectory_topic_, 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/exploration_status", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "frontiers", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish_status(const std::string &status,
                      const std::string &detail = "");
  std::optional<RobotPose> robot_pose_in_frame(
      const std::string &target_frame) const;
  void append_trajectory_pose(const std_msgs::msg::Header &header,
                              const RobotPose &robot_pose);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  
  // 对比新旧地图，找变化的 cell
  std::set<int> frontier_set_;
  
  std::vector<int8_t> prev_map_data_;

  int min_cluster_size_{10};
  std::vector<double> sample_radii_m_;
  int angular_samples_{16};
  double max_view_range_m_{3.5};
  double candidate_clearance_m_{0.2};
  int min_viewpoint_coverage_{1};
  int max_viewpoints_{15};
  int occupied_threshold_{50};
  bool block_unknown_in_los_{true};
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string trajectory_topic_;
  double goal_distance_weight_{1.0};
  double path_clearance_m_{0.15};
  bool block_unknown_in_path_{true};
  double robot_pose_timeout_s_{0.1};
  double min_goal_distance_m_{0.35};
  double viewpoint_rank_penalty_{0.05};
  double goal_reached_tolerance_m_{0.25};
  double revisit_block_radius_m_{0.45};
  double trajectory_record_distance_m_{0.05};
  int tsp_lookahead_{4};
  std::optional<RobotPose> robot_pose_;
  std::optional<ViewpointCandidate> active_goal_;
  std::vector<VisitedViewpoint> visited_viewpoints_;
  nav_msgs::msg::Path trajectory_path_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  mutable bool warned_missing_pose_{false};
  mutable bool warned_transform_failure_{false};
  std::string last_status_;
};

void FrontierDetector::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = RobotPose{msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          quaternion_to_yaw(msg->pose.pose.orientation),
                          msg->header.frame_id};
}

void FrontierDetector::publish_status(const std::string &status,
                                      const std::string &detail) {
  std_msgs::msg::String status_msg;
  status_msg.data = detail.empty() ? status : status + " | " + detail;
  status_pub_->publish(status_msg);

  if (status != last_status_) {
    RCLCPP_INFO(this->get_logger(), "Exploration status -> %s",
                status_msg.data.c_str());
    last_status_ = status;
  }
}

std::optional<RobotPose> FrontierDetector::robot_pose_in_frame(
    const std::string &target_frame) const {
  if (!robot_pose_.has_value()) {
    return std::nullopt;
  }

  if (robot_pose_->frame_id == target_frame) {
    return robot_pose_;
  }

  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header.frame_id = robot_pose_->frame_id;
  source_pose.header.stamp = builtin_interfaces::msg::Time{};
  source_pose.pose.position.x = robot_pose_->x;
  source_pose.pose.position.y = robot_pose_->y;
  source_pose.pose.position.z = 0.0;
  source_pose.pose.orientation = yaw_to_quaternion(robot_pose_->yaw);

  try {
    const auto transformed_pose = tf_buffer_->transform(
        source_pose, target_frame,
        tf2::durationFromSec(robot_pose_timeout_s_));
    warned_transform_failure_ = false;
    return RobotPose{transformed_pose.pose.position.x,
                     transformed_pose.pose.position.y,
                     quaternion_to_yaw(transformed_pose.pose.orientation),
                     target_frame};
  } catch (const tf2::TransformException &exception) {
    if (!warned_transform_failure_) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to transform robot pose from '%s' to '%s': %s",
                  robot_pose_->frame_id.c_str(), target_frame.c_str(),
                  exception.what());
      warned_transform_failure_ = true;
    }
    return std::nullopt;
  }
}

void FrontierDetector::append_trajectory_pose(
    const std_msgs::msg::Header &header, const RobotPose &robot_pose) {
  if (!trajectory_path_.poses.empty() &&
      trajectory_path_.header.frame_id != header.frame_id) {
    RCLCPP_WARN(this->get_logger(),
                "Trajectory frame changed from '%s' to '%s'; clearing accumulated "
                "trajectory.",
                trajectory_path_.header.frame_id.c_str(), header.frame_id.c_str());
    trajectory_path_.poses.clear();
  }

  trajectory_path_.header = header;

  geometry_msgs::msg::PoseStamped trajectory_pose;
  trajectory_pose.header = header;
  trajectory_pose.pose.position.x = robot_pose.x;
  trajectory_pose.pose.position.y = robot_pose.y;
  trajectory_pose.pose.position.z = 0.0;
  trajectory_pose.pose.orientation = yaw_to_quaternion(robot_pose.yaw);

  if (trajectory_path_.poses.empty()) {
    trajectory_path_.poses.push_back(trajectory_pose);
    return;
  }

  const auto &last_position = trajectory_path_.poses.back().pose.position;
  const double distance_since_last_sample =
      std::hypot(trajectory_pose.pose.position.x - last_position.x,
                 trajectory_pose.pose.position.y - last_position.y);
  if (distance_since_last_sample >= trajectory_record_distance_m_) {
    trajectory_path_.poses.push_back(trajectory_pose);
  } else {
    trajectory_path_.poses.back().header = header;
    trajectory_path_.poses.back().pose.orientation =
        trajectory_pose.pose.orientation;
  }
}

void FrontierDetector::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received map: %u x %u", msg->info.width,
              msg->info.height);

  const auto &map_data = msg->data;
  const int width = static_cast<int>(msg->info.width);
  const int height = static_cast<int>(msg->info.height);

  auto is_frontier = [&](const int idx) -> bool {
    if (map_data[idx] != kFreeCell) {
      return false;
    }

    const int row = idx / width;
    const int col = idx % width;
    for (const auto &[dr, dc] : kNeighborhood) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (!is_inside_map(nr, nc, *msg)) {
        continue;
      }
      if (map_data[flatten_index(nr, nc, width)] == kUnknownCell) {
        return true;
      }
    }
    return false;
  };

  if (prev_map_data_.empty()) {
    for (int idx = 0; idx < width * height; ++idx) {
      if (is_frontier(idx)) {
        frontier_set_.insert(idx);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Initial frontier cells: %zu",
                frontier_set_.size());
  } else {
    std::set<int> wait_check;
    for (int idx = 0; idx < width * height; ++idx) {
      if (map_data[idx] == prev_map_data_[idx]) {
        continue;
      }

      wait_check.insert(idx);
      const int row = idx / width;
      const int col = idx % width;
      for (const auto &[dr, dc] : kNeighborhood) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (is_inside_map(nr, nc, *msg)) {
          wait_check.insert(flatten_index(nr, nc, width));
        }
      }
    }

    for (const int idx : wait_check) {
      if (is_frontier(idx)) {
        frontier_set_.insert(idx);
      } else {
        frontier_set_.erase(idx);
      }
    }
    RCLCPP_INFO(this->get_logger(),
                "Incremental update: %zu cells checked, %zu frontiers",
                wait_check.size(), frontier_set_.size());
  }
  // 保存当前 map 用于下次对比
  prev_map_data_ = msg->data;

  std::vector<FrontierCluster> clusters;
  std::set<int> visited;
  for (const int idx : frontier_set_) {
    if (visited.count(idx) > 0) {
      continue;
    }

    std::vector<int> current_cluster;
    std::queue<int> pending_cells;
    pending_cells.push(idx);
    visited.insert(idx);

    while (!pending_cells.empty()) {
      const int current = pending_cells.front();
      pending_cells.pop();
      current_cluster.push_back(current);

      const int row = current / width;
      const int col = current % width;
      for (const auto &[dr, dc] : kNeighborhood) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (!is_inside_map(nr, nc, *msg)) {
          continue;
        }

        const int neighbor_idx = flatten_index(nr, nc, width);
        if (frontier_set_.count(neighbor_idx) == 0 ||
            visited.count(neighbor_idx) > 0) {
          continue;
        }

        visited.insert(neighbor_idx);
        pending_cells.push(neighbor_idx);
      }
    }

    if (static_cast<int>(current_cluster.size()) < min_cluster_size_) {
      continue;
    }

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const int frontier_idx : current_cluster) {
      const auto world =
          grid_to_world(frontier_idx / width, frontier_idx % width, *msg);
      centroid_x += world.first;
      centroid_y += world.second;
    }
    centroid_x /= static_cast<double>(current_cluster.size());
    centroid_y /= static_cast<double>(current_cluster.size());

    FrontierCluster cluster{current_cluster, centroid_x, centroid_y, {}};
    cluster.viewpoints =
        generate_candidates(cluster, *msg, sample_radii_m_, angular_samples_,
                            candidate_clearance_m_, max_view_range_m_,
                            min_viewpoint_coverage_, max_viewpoints_,
                            occupied_threshold_, block_unknown_in_los_);
    clusters.push_back(cluster);
  }

  std::size_t clusters_without_viewpoint = 0;
  std::size_t clusters_with_viewpoint = 0;
  std::size_t total_viewpoints = 0;
  for (const auto &cluster : clusters) {
    if (cluster.viewpoints.empty()) {
      ++clusters_without_viewpoint;
    } else {
      ++clusters_with_viewpoint;
      total_viewpoints += cluster.viewpoints.size();
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Retained %zu frontier clusters, %zu without valid viewpoints, "
              "%zu viewpoints total",
              clusters.size(), clusters_without_viewpoint, total_viewpoints);

  std::optional<SelectedGoal> selected_goal;
  nav_msgs::msg::Path planned_path_msg;
  planned_path_msg.header = msg->header;
  const auto planning_pose = robot_pose_in_frame(msg->header.frame_id);
  if (planning_pose.has_value()) {
    append_trajectory_pose(msg->header, *planning_pose);
  } else if (!trajectory_path_.poses.empty()) {
    trajectory_path_.header.stamp = msg->header.stamp;
  }
  std::string exploration_status = "idle";
  std::string status_detail = "clusters=" + std::to_string(clusters.size()) +
                              ", viewpoints=" +
                              std::to_string(total_viewpoints) + ", visited=" +
                              std::to_string(visited_viewpoints_.size());
  if (planning_pose.has_value()) {
    if (active_goal_.has_value() &&
        std::hypot(planning_pose->x - active_goal_->x,
                   planning_pose->y - active_goal_->y) <=
            goal_reached_tolerance_m_) {
      visited_viewpoints_.push_back(
          VisitedViewpoint{active_goal_->x, active_goal_->y});
      RCLCPP_INFO(this->get_logger(),
                  "Reached goal viewpoint, blocking revisits within %.2f m. "
                  "Visited viewpoints: %zu",
                  revisit_block_radius_m_, visited_viewpoints_.size());
      active_goal_.reset();
    }

    // --- 第一层：TSP 排序 ---
    // 过滤出有视点且视点未被访问过的簇
    std::vector<std::size_t> viable_cluster_indices;
    for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
      const auto &cluster = clusters[ci];
      bool has_unvisited = false;
      for (const auto &vp : cluster.viewpoints) {
        if (!is_near_visited_viewpoint(vp, visited_viewpoints_,
                                       revisit_block_radius_m_)) {
          has_unvisited = true;
          break;
        }
      }
      if (has_unvisited) {
        viable_cluster_indices.push_back(ci);
      }
    }

    if (!viable_cluster_indices.empty()) {
      // 构建代价矩阵：节点0=机器人, 节点1..N=各可行簇
      const std::size_t n = viable_cluster_indices.size() + 1;
      std::vector<std::vector<double>> cost_matrix(
          n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
      cost_matrix[0][0] = 0.0;

      // 机器人到各簇（用最佳视点坐标代替质心）
      // 探索场景：代价矩阵不阻塞未知格子，否则大部分路径不可达
      for (std::size_t i = 0; i < viable_cluster_indices.size(); ++i) {
        const auto &cluster = clusters[viable_cluster_indices[i]];
        const auto &rep = cluster.viewpoints[0];
        const auto cost = compute_path_cost(
            planning_pose->x, planning_pose->y, rep.x, rep.y, *msg,
            occupied_threshold_, block_unknown_in_path_, path_clearance_m_);
        if (cost.has_value()) {
          cost_matrix[0][i + 1] = *cost;
          cost_matrix[i + 1][0] = *cost;
        }
      }

      // 簇间代价（用最佳视点坐标代替质心）
      for (std::size_t i = 0; i < viable_cluster_indices.size(); ++i) {
        cost_matrix[i + 1][i + 1] = 0.0;
        const auto &repi = clusters[viable_cluster_indices[i]].viewpoints[0];
        for (std::size_t j = i + 1; j < viable_cluster_indices.size(); ++j) {
          const auto &repj = clusters[viable_cluster_indices[j]].viewpoints[0];
          const auto cost = compute_path_cost(
              repi.x, repi.y, repj.x, repj.y, *msg, occupied_threshold_,
              block_unknown_in_path_, path_clearance_m_);
          if (cost.has_value()) {
            cost_matrix[i + 1][j + 1] = *cost;
            cost_matrix[j + 1][i + 1] = *cost;
          }
        }
      }

      const auto tsp_order = solve_tsp_nearest_neighbor(cost_matrix);
      std::ostringstream tsp_order_stream;
      if (tsp_order.empty()) {
        tsp_order_stream << "<none>";
      } else {
        for (std::size_t order_idx = 0; order_idx < tsp_order.size();
             ++order_idx) {
          if (order_idx > 0) {
            tsp_order_stream << " -> ";
          }
          tsp_order_stream << "c"
                           << viable_cluster_indices[tsp_order[order_idx]];
        }
      }

      RCLCPP_INFO(this->get_logger(), "TSP order: %s",
                  tsp_order_stream.str().c_str());

      // --- 第二层：Dijkstra 视点选择 ---
      // 取 TSP 前 K 个簇，用 Dijkstra 选各簇最优视点组合
      if (!tsp_order.empty()) {
        const std::size_t k = std::min(
            static_cast<std::size_t>(std::max(1, tsp_lookahead_)),
            tsp_order.size());

        std::vector<FrontierCluster> layer_clusters;
        std::vector<std::size_t> layer_original_indices;
        for (std::size_t i = 0; i < k; ++i) {
          const std::size_t ci = viable_cluster_indices[tsp_order[i]];
          layer_clusters.push_back(clusters[ci]);
          layer_original_indices.push_back(ci);
        }

        const auto dijkstra_result = solve_viewpoint_dijkstra(
            planning_pose->x, planning_pose->y, layer_clusters, *msg,
            occupied_threshold_, block_unknown_in_path_, path_clearance_m_,
            visited_viewpoints_, revisit_block_radius_m_);

        if (dijkstra_result.has_value()) {
          // Dijkstra 结果的第一个簇的选中视点就是下一个目标
          const std::size_t first_cluster_idx = layer_original_indices[0];
          const std::size_t first_vp_idx =
              dijkstra_result->selected_viewpoint_indices[0];
          const auto &vp =
              clusters[first_cluster_idx].viewpoints[first_vp_idx];

          const auto path = plan_astar_path(
              *planning_pose, vp, *msg, occupied_threshold_,
              block_unknown_in_path_, path_clearance_m_);
          if (path.has_value() && path->length_m >= min_goal_distance_m_) {
            const double utility =
                static_cast<double>(vp.score) /
                    (1.0 + goal_distance_weight_ * dijkstra_result->total_cost) -
                viewpoint_rank_penalty_ * static_cast<double>(first_vp_idx);
            selected_goal =
                SelectedGoal{vp, first_cluster_idx, first_vp_idx, utility,
                             path->length_m, path->indices};
          }
        }

        // Dijkstra 失败时回退：按 TSP 顺序逐簇找第一个可达视点
        if (!selected_goal.has_value()) {
          for (std::size_t tsp_rank = 0; tsp_rank < tsp_order.size();
               ++tsp_rank) {
            const std::size_t tsp_idx = tsp_order[tsp_rank];
            const std::size_t cluster_idx = viable_cluster_indices[tsp_idx];
            const auto &cluster = clusters[cluster_idx];

            for (std::size_t vi = 0; vi < cluster.viewpoints.size(); ++vi) {
              const auto &vp = cluster.viewpoints[vi];
              if (is_near_visited_viewpoint(vp, visited_viewpoints_,
                                            revisit_block_radius_m_)) {
                continue;
              }

              const auto path = plan_astar_path(
                  *planning_pose, vp, *msg, occupied_threshold_,
                  block_unknown_in_path_, path_clearance_m_);
              if (!path.has_value() ||
                  path->length_m < min_goal_distance_m_) {
                continue;
              }

              const double utility =
                  static_cast<double>(vp.score) /
                      (1.0 + goal_distance_weight_ * path->length_m +
                       static_cast<double>(tsp_rank)) -
                  viewpoint_rank_penalty_ * static_cast<double>(vi);
              selected_goal = SelectedGoal{vp, cluster_idx, vi, utility,
                                           path->length_m, path->indices};
              break;
            }

            if (selected_goal.has_value()) {
              break;
            }
          }
        }
      }
    }

    if (selected_goal.has_value()) {
      exploration_status = "exploring";
      status_detail +=
          ", goal_cluster=" + std::to_string(selected_goal->cluster_index) +
          ", goal_viewpoint=" + std::to_string(selected_goal->viewpoint_index);
      active_goal_ = selected_goal->viewpoint;
      geometry_msgs::msg::PoseStamped goal_msg;
      goal_msg.header = msg->header;
      goal_msg.pose.position.x = selected_goal->viewpoint.x;
      goal_msg.pose.position.y = selected_goal->viewpoint.y;
      goal_msg.pose.position.z = 0.0;
      goal_msg.pose.orientation = yaw_to_quaternion(selected_goal->viewpoint.yaw);
      goal_pub_->publish(goal_msg);

      for (const int path_index : selected_goal->path_indices) {
        const int path_row = path_index / width;
        const int path_col = path_index % width;
        const auto world_point = grid_to_world(path_row, path_col, *msg);

        geometry_msgs::msg::PoseStamped path_pose;
        path_pose.header = msg->header;
        path_pose.pose.position.x = world_point.first;
        path_pose.pose.position.y = world_point.second;
        path_pose.pose.position.z = 0.0;
        path_pose.pose.orientation.w = 1.0;
        planned_path_msg.poses.push_back(path_pose);
      }
    } else if (clusters_with_viewpoint > 0U) {
      exploration_status = "stalled";
      RCLCPP_INFO(this->get_logger(),
                  "No goal selected beyond %.2f m; remaining candidate "
                  "viewpoints are either already visited or unreachable.",
                  min_goal_distance_m_);
    }
  } else if (!robot_pose_.has_value() && !warned_missing_pose_) {
    exploration_status = "waiting_for_pose";
    RCLCPP_WARN(this->get_logger(),
                "No robot pose received on %s yet, so next-goal selection is "
                "disabled for now.",
                odom_topic_.c_str());
    warned_missing_pose_ = true;
  } else if (robot_pose_.has_value()) {
    exploration_status = "waiting_for_tf";
  }
  path_pub_->publish(planned_path_msg);
  trajectory_pub_->publish(trajectory_path_);

  if (frontier_set_.empty()) {
    exploration_status = "finished";
    status_detail = "frontiers=0";
  } else if (clusters.empty() && total_viewpoints == 0U &&
             exploration_status != "waiting_for_pose" &&
             exploration_status != "waiting_for_tf") {
    exploration_status = "no_clusters";
    status_detail = "frontiers=" + std::to_string(frontier_set_.size()) +
                    ", below_cluster_threshold";
  } else if (clusters_with_viewpoint == 0U && !clusters.empty() &&
             exploration_status != "waiting_for_pose" &&
             exploration_status != "waiting_for_tf") {
    exploration_status = "no_viewpoints";
  }

  publish_status(exploration_status, status_detail);

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(make_delete_all_marker(msg->header));

  if (!frontier_set_.empty()) {
    visualization_msgs::msg::Marker frontier_cells_marker;
    frontier_cells_marker.header = msg->header;
    frontier_cells_marker.ns = "frontier_cells";
    frontier_cells_marker.id = 0;
    frontier_cells_marker.type = visualization_msgs::msg::Marker::POINTS;
    frontier_cells_marker.action = visualization_msgs::msg::Marker::ADD;
    frontier_cells_marker.pose.orientation.w = 1.0;
    frontier_cells_marker.scale.x = std::max(0.03, msg->info.resolution * 0.75);
    frontier_cells_marker.scale.y = std::max(0.03, msg->info.resolution * 0.75);
    frontier_cells_marker.color.r = 1.0F;
    frontier_cells_marker.color.g = 0.45F;
    frontier_cells_marker.color.b = 0.15F;
    frontier_cells_marker.color.a = 0.85F;

    for (const int idx : frontier_set_) {
      const int frontier_row = idx / width;
      const int frontier_col = idx % width;
      const auto world_point = grid_to_world(frontier_row, frontier_col, *msg);

      geometry_msgs::msg::Point point;
      point.x = world_point.first;
      point.y = world_point.second;
      point.z = 0.04;
      frontier_cells_marker.points.push_back(point);
    }

    marker_array.markers.push_back(frontier_cells_marker);
  }

  if (!trajectory_path_.poses.empty()) {
    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header = trajectory_path_.header;
    trajectory_marker.ns = "exploration_trajectory";
    trajectory_marker.id = 0;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.pose.orientation.w = 1.0;
    trajectory_marker.scale.x = 0.08;
    trajectory_marker.color.r = 0.1F;
    trajectory_marker.color.g = 0.95F;
    trajectory_marker.color.b = 0.95F;
    trajectory_marker.color.a = 0.95F;

    for (const auto &pose : trajectory_path_.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = 0.08;
      trajectory_marker.points.push_back(point);
    }

    marker_array.markers.push_back(trajectory_marker);
  }

  const int viewpoint_id_stride = std::max(1, max_viewpoints_);
  for (std::size_t cluster_idx = 0; cluster_idx < clusters.size(); ++cluster_idx) {
    const auto &cluster = clusters[cluster_idx];

    visualization_msgs::msg::Marker centroid_marker;
    centroid_marker.header = msg->header;
    centroid_marker.ns = "centroids";
    centroid_marker.id = static_cast<int>(cluster_idx);
    centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
    centroid_marker.action = visualization_msgs::msg::Marker::ADD;
    centroid_marker.pose.orientation.w = 1.0;
    centroid_marker.pose.position.x = cluster.centroid_x;
    centroid_marker.pose.position.y = cluster.centroid_y;
    centroid_marker.pose.position.z = 0.1;
    centroid_marker.scale.x = 0.30;
    centroid_marker.scale.y = 0.30;
    centroid_marker.scale.z = 0.30;
    centroid_marker.color.r = 1.0;
    centroid_marker.color.a = 1.0;
    marker_array.markers.push_back(centroid_marker);

    for (std::size_t viewpoint_idx = 0; viewpoint_idx < cluster.viewpoints.size();
         ++viewpoint_idx) {
      const auto &viewpoint = cluster.viewpoints[viewpoint_idx];
      const bool is_best_viewpoint = viewpoint_idx == 0;
      const int marker_id = static_cast<int>(cluster_idx) * viewpoint_id_stride +
                            static_cast<int>(viewpoint_idx);

      visualization_msgs::msg::Marker viewpoint_marker;
      viewpoint_marker.header = msg->header;
      viewpoint_marker.ns =
          is_best_viewpoint ? "best_viewpoints" : "candidate_viewpoints";
      viewpoint_marker.id = marker_id;
      viewpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
      viewpoint_marker.action = visualization_msgs::msg::Marker::ADD;
      viewpoint_marker.pose.orientation.w = 1.0;
      viewpoint_marker.pose.position.x = viewpoint.x;
      viewpoint_marker.pose.position.y = viewpoint.y;
      viewpoint_marker.pose.position.z = 0.12;
      viewpoint_marker.scale.x = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.scale.y = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.scale.z = is_best_viewpoint ? 0.25 : 0.18;
      viewpoint_marker.color.r = is_best_viewpoint ? 0.1F : 0.2F;
      viewpoint_marker.color.g = is_best_viewpoint ? 0.9F : 0.6F;
      viewpoint_marker.color.b = is_best_viewpoint ? 0.2F : 1.0F;
      viewpoint_marker.color.a = is_best_viewpoint ? 1.0F : 0.65F;
      marker_array.markers.push_back(viewpoint_marker);

      if (!is_best_viewpoint) {
        continue;
      }

      visualization_msgs::msg::Marker heading_marker;
      heading_marker.header = msg->header;
      heading_marker.ns = "best_viewpoint_heading";
      heading_marker.id = static_cast<int>(cluster_idx);
      heading_marker.type = visualization_msgs::msg::Marker::ARROW;
      heading_marker.action = visualization_msgs::msg::Marker::ADD;
      heading_marker.pose.position.x = viewpoint.x;
      heading_marker.pose.position.y = viewpoint.y;
      heading_marker.pose.position.z = 0.15;
      heading_marker.pose.orientation.z = std::sin(viewpoint.yaw / 2.0);
      heading_marker.pose.orientation.w = std::cos(viewpoint.yaw / 2.0);
      heading_marker.scale.x = 0.35;
      heading_marker.scale.y = 0.08;
      heading_marker.scale.z = 0.08;
      heading_marker.color.g = 0.9F;
      heading_marker.color.b = 0.1F;
      heading_marker.color.a = 1.0F;
      marker_array.markers.push_back(heading_marker);

      visualization_msgs::msg::Marker score_marker;
      score_marker.header = msg->header;
      score_marker.ns = "viewpoint_scores";
      score_marker.id = static_cast<int>(cluster_idx);
      score_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      score_marker.action = visualization_msgs::msg::Marker::ADD;
      score_marker.pose.orientation.w = 1.0;
      score_marker.pose.position.x = viewpoint.x;
      score_marker.pose.position.y = viewpoint.y;
      score_marker.pose.position.z = 0.45;
      score_marker.scale.z = 0.18;
      score_marker.color.r = 1.0F;
      score_marker.color.g = 1.0F;
      score_marker.color.b = 1.0F;
      score_marker.color.a = 1.0F;
      score_marker.text = "score: " + std::to_string(viewpoint.score);
      marker_array.markers.push_back(score_marker);
    }
  }

  if (selected_goal.has_value()) {
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header = msg->header;
    goal_marker.ns = "selected_goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.pose.position.x = selected_goal->viewpoint.x;
    goal_marker.pose.position.y = selected_goal->viewpoint.y;
    goal_marker.pose.position.z = 0.05;
    goal_marker.scale.x = 0.40;
    goal_marker.scale.y = 0.40;
    goal_marker.scale.z = 0.10;
    goal_marker.color.r = 1.0F;
    goal_marker.color.g = 0.9F;
    goal_marker.color.a = 0.85F;
    marker_array.markers.push_back(goal_marker);

    visualization_msgs::msg::Marker goal_label;
    goal_label.header = msg->header;
    goal_label.ns = "selected_goal_label";
    goal_label.id = 0;
    goal_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    goal_label.action = visualization_msgs::msg::Marker::ADD;
    goal_label.pose.orientation.w = 1.0;
    goal_label.pose.position.x = selected_goal->viewpoint.x;
    goal_label.pose.position.y = selected_goal->viewpoint.y;
    goal_label.pose.position.z = 0.70;
    goal_label.scale.z = 0.22;
    goal_label.color.r = 1.0F;
    goal_label.color.g = 0.95F;
    goal_label.color.b = 0.2F;
    goal_label.color.a = 1.0F;
    goal_label.text =
        "goal c" + std::to_string(selected_goal->cluster_index) + " v" +
        std::to_string(selected_goal->viewpoint_index) + " | u:" +
        std::to_string(selected_goal->utility).substr(0, 4);
    marker_array.markers.push_back(goal_label);

    if (robot_pose_.has_value()) {
      visualization_msgs::msg::Marker path_marker;
      path_marker.header = msg->header;
      path_marker.ns = "selected_goal_path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::msg::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;
      for (const auto &pose : planned_path_msg.poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.pose.position.x;
        point.y = pose.pose.position.y;
        point.z = 0.18;
        path_marker.points.push_back(point);
      }
      path_marker.scale.x = 0.06;
      path_marker.scale.y = 0.0;
      path_marker.scale.z = 0.0;
      path_marker.color.r = 1.0F;
      path_marker.color.g = 0.8F;
      path_marker.color.b = 0.1F;
      path_marker.color.a = 0.95F;
      marker_array.markers.push_back(path_marker);
    }
  }

  marker_pub_->publish(marker_array);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
