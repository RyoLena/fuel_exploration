#include "fuel_exploration/frontier/frontier_map_processing.hpp"

#include "fuel_exploration/frontier/frontier_algorithms.hpp"

#include <queue>

namespace {

using fuel_exploration::flatten_index;
using fuel_exploration::grid_to_world;
using fuel_exploration::is_inside_map;
using fuel_exploration::kFreeCell;
using fuel_exploration::kNeighborhood;
using fuel_exploration::kUnknownCell;

}  // namespace

namespace fuel_exploration::frontier {

FrontierSetUpdateResult update_frontier_set(
    const OccupancyGrid &map, const std::vector<int8_t> &prev_map_data,
    std::set<int> &frontier_set) {
  const auto &map_data = map.data;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);

  const auto is_frontier = [&](const int idx) -> bool {
    if (map_data[idx] != kFreeCell) {
      return false;
    }

    const int row = idx / width;
    const int col = idx % width;
    for (const auto &[dr, dc] : kNeighborhood) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (!is_inside_map(nr, nc, map)) {
        continue;
      }
      if (map_data[flatten_index(nr, nc, width)] == kUnknownCell) {
        return true;
      }
    }
    return false;
  };

  FrontierSetUpdateResult result;
  result.map_resized =
      !prev_map_data.empty() && prev_map_data.size() != map_data.size();
  result.full_rebuild = prev_map_data.empty() || result.map_resized;

  if (result.full_rebuild) {
    frontier_set.clear();
    for (int idx = 0; idx < width * height; ++idx) {
      if (is_frontier(idx)) {
        frontier_set.insert(idx);
      }
    }
    result.checked_cell_count = static_cast<std::size_t>(width * height);
    return result;
  }

  std::set<int> wait_check;
  for (int idx = 0; idx < width * height; ++idx) {
    if (map_data[idx] == prev_map_data[idx]) {
      continue;
    }

    wait_check.insert(idx);
    const int row = idx / width;
    const int col = idx % width;
    for (const auto &[dr, dc] : kNeighborhood) {
      const int nr = row + dr;
      const int nc = col + dc;
      if (is_inside_map(nr, nc, map)) {
        wait_check.insert(flatten_index(nr, nc, width));
      }
    }
  }

  for (const int idx : wait_check) {
    if (is_frontier(idx)) {
      frontier_set.insert(idx);
    } else {
      frontier_set.erase(idx);
    }
  }

  result.checked_cell_count = wait_check.size();
  return result;
}

std::vector<FrontierCluster> build_frontier_clusters(
    const std::set<int> &frontier_set, const OccupancyGrid &map,
    const int min_cluster_size, const std::vector<double> &sample_radii_m,
    const int angular_samples, const double candidate_clearance_m,
    const double max_view_range_m, const int min_viewpoint_coverage,
    const int max_viewpoints, const int occupied_threshold,
    const bool block_unknown_in_los) {
  std::vector<FrontierCluster> clusters;
  std::set<int> visited;
  const int width = static_cast<int>(map.info.width);

  for (const int idx : frontier_set) {
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
        if (!is_inside_map(nr, nc, map)) {
          continue;
        }

        const int neighbor_idx = flatten_index(nr, nc, width);
        if (frontier_set.count(neighbor_idx) == 0 ||
            visited.count(neighbor_idx) > 0) {
          continue;
        }

        visited.insert(neighbor_idx);
        pending_cells.push(neighbor_idx);
      }
    }

    if (static_cast<int>(current_cluster.size()) < min_cluster_size) {
      continue;
    }

    double centroid_x = 0.0;
    double centroid_y = 0.0;
    for (const int frontier_idx : current_cluster) {
      const auto world =
          grid_to_world(frontier_idx / width, frontier_idx % width, map);
      centroid_x += world.first;
      centroid_y += world.second;
    }
    centroid_x /= static_cast<double>(current_cluster.size());
    centroid_y /= static_cast<double>(current_cluster.size());

    FrontierCluster cluster{current_cluster, centroid_x, centroid_y, {}};
    cluster.viewpoints = generate_candidates(
        cluster, map, sample_radii_m, angular_samples, candidate_clearance_m,
        max_view_range_m, min_viewpoint_coverage, max_viewpoints,
        occupied_threshold, block_unknown_in_los);
    clusters.push_back(cluster);
  }

  return clusters;
}

ClusterStatistics summarize_clusters(
    const std::vector<FrontierCluster> &clusters) {
  ClusterStatistics stats;
  for (const auto &cluster : clusters) {
    if (cluster.viewpoints.empty()) {
      ++stats.clusters_without_viewpoint;
    } else {
      ++stats.clusters_with_viewpoint;
      stats.total_viewpoints += cluster.viewpoints.size();
    }
  }
  return stats;
}

}  // namespace fuel_exploration::frontier
