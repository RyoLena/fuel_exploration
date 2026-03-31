#include "fuel_exploration/frontier/frontier_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

namespace {

using fuel_exploration::flatten_index;
using fuel_exploration::grid_to_world;
using fuel_exploration::is_inside_map;
using fuel_exploration::is_occupied;
using fuel_exploration::kFreeCell;
using fuel_exploration::kNeighborhood;
using fuel_exploration::kUnknownCell;
using fuel_exploration::world_to_grid;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool has_free_clearance(const int row, const int col,
                        const fuel_exploration::OccupancyGrid &map,
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

bool is_path_traversable(const int row, const int col,
                         const fuel_exploration::OccupancyGrid &map,
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

std::optional<int> snap_to_nearest_traversable_index(
    const int seed_row, const int seed_col,
    const fuel_exploration::OccupancyGrid &map,
    const int occupied_threshold, const bool block_unknown,
    const int clearance_cells, const int max_search_radius) {
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

        if (is_path_traversable(row, col, map, occupied_threshold,
                                block_unknown, clearance_cells)) {
          return flatten_index(row, col, map.info.width);
        }
      }
    }
  }

  return std::nullopt;
}

bool has_line_of_sight(const int start_row, const int start_col,
                       const int goal_row, const int goal_col,
                       const fuel_exploration::OccupancyGrid &map,
                       const int occupied_threshold,
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

int score_candidate(
    const fuel_exploration::frontier::FrontierCluster &cluster,
    const fuel_exploration::frontier::ViewpointCandidate &candidate,
    const fuel_exploration::OccupancyGrid &map, const double max_view_range_m,
    const int occupied_threshold, const bool block_unknown) {
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

}  // namespace

namespace fuel_exploration::frontier {

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

std::optional<PlannedPath> plan_astar_path(
    const RobotPose &robot_pose, const ViewpointCandidate &goal_viewpoint,
    const OccupancyGrid &map, const int occupied_threshold,
    const bool block_unknown, const double path_clearance_m) {
  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!world_to_grid(robot_pose.x, robot_pose.y, map, start_row, start_col) ||
      !world_to_grid(goal_viewpoint.x, goal_viewpoint.y, map, goal_row,
                     goal_col)) {
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

  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open_set;
  std::vector<double> g_score(cell_count,
                              std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
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
    const double start_x, const double start_y, const double goal_x,
    const double goal_y, const OccupancyGrid &map,
    const int occupied_threshold, const bool block_unknown,
    const double path_clearance_m) {
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

  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open_set;
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
                  std::hypot(lhs.x - cluster.centroid_x,
                             lhs.y - cluster.centroid_y);
              const double rhs_distance =
                  std::hypot(rhs.x - cluster.centroid_x,
                             rhs.y - cluster.centroid_y);
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

std::vector<std::size_t> solve_tsp_nearest_neighbor(
    const std::vector<std::vector<double>> &cost_matrix) {
  const std::size_t n = cost_matrix.size();
  if (n <= 1) {
    return {};
  }

  std::vector<bool> visited(n, false);
  visited[0] = true;
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
      break;
    }

    visited[best_next] = true;
    current = best_next;
    order.push_back(best_next - 1);
  }

  return order;
}

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
  std::vector<std::vector<double>> dist(num_layers);
  std::vector<std::vector<std::size_t>> prev(num_layers);

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

}  // namespace fuel_exploration::frontier
