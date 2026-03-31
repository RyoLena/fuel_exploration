#include "fuel_exploration/frontier/frontier_goal_selection.hpp"

#include "fuel_exploration/frontier/frontier_algorithms.hpp"

#include <algorithm>
#include <limits>
#include <sstream>

namespace fuel_exploration::frontier {

GoalSelectionResult select_next_goal(
    const RobotPose &planning_pose,
    const std::vector<FrontierCluster> &clusters,
    const OccupancyGrid &map,
    const int occupied_threshold,
    const bool block_unknown_in_path,
    const double path_clearance_m,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    const double revisit_block_radius_m,
    const int tsp_lookahead,
    const double min_goal_distance_m,
    const double goal_distance_weight,
    const double viewpoint_rank_penalty) {
  GoalSelectionResult result;

  const auto collect_viable_cluster_indices =
      [&](const double effective_revisit_radius_m) {
        std::vector<std::size_t> viable_cluster_indices;
        for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
          const auto &cluster = clusters[ci];
          bool has_unvisited = false;
          for (const auto &vp : cluster.viewpoints) {
            if (!is_near_visited_viewpoint(vp, visited_viewpoints,
                                           effective_revisit_radius_m)) {
              has_unvisited = true;
              break;
            }
          }
          if (has_unvisited) {
            viable_cluster_indices.push_back(ci);
          }
        }
        return viable_cluster_indices;
      };

  double effective_revisit_radius_m = revisit_block_radius_m;
  std::vector<std::size_t> viable_cluster_indices =
      collect_viable_cluster_indices(effective_revisit_radius_m);

  // If all candidate viewpoints are blocked only by revisit suppression,
  // retry once without the block so exploration can continue on the same frontier.
  if (viable_cluster_indices.empty() && !visited_viewpoints.empty() &&
      revisit_block_radius_m > 0.0) {
    effective_revisit_radius_m = 0.0;
    viable_cluster_indices =
        collect_viable_cluster_indices(effective_revisit_radius_m);
    result.revisit_filter_relaxed = !viable_cluster_indices.empty();
  }

  if (viable_cluster_indices.empty()) {
    result.tsp_order_description = "<none>";
    return result;
  }

  const std::size_t n = viable_cluster_indices.size() + 1;
  std::vector<std::vector<double>> cost_matrix(
      n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
  cost_matrix[0][0] = 0.0;

  for (std::size_t i = 0; i < viable_cluster_indices.size(); ++i) {
    const auto &cluster = clusters[viable_cluster_indices[i]];
    const auto &rep = cluster.viewpoints[0];
    const auto cost = compute_path_cost(
        planning_pose.x, planning_pose.y, rep.x, rep.y, map,
        occupied_threshold, block_unknown_in_path, path_clearance_m);
    if (cost.has_value()) {
      cost_matrix[0][i + 1] = *cost;
      cost_matrix[i + 1][0] = *cost;
    }
  }

  for (std::size_t i = 0; i < viable_cluster_indices.size(); ++i) {
    cost_matrix[i + 1][i + 1] = 0.0;
    const auto &repi = clusters[viable_cluster_indices[i]].viewpoints[0];
    for (std::size_t j = i + 1; j < viable_cluster_indices.size(); ++j) {
      const auto &repj = clusters[viable_cluster_indices[j]].viewpoints[0];
      const auto cost = compute_path_cost(
          repi.x, repi.y, repj.x, repj.y, map,
          occupied_threshold, block_unknown_in_path, path_clearance_m);
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
    for (std::size_t order_idx = 0; order_idx < tsp_order.size(); ++order_idx) {
      if (order_idx > 0) {
        tsp_order_stream << " -> ";
      }
      tsp_order_stream << "c" << viable_cluster_indices[tsp_order[order_idx]];
    }
  }
  result.tsp_order_description = tsp_order_stream.str();

  if (tsp_order.empty()) {
    return result;
  }

  const std::size_t k = std::min(
      static_cast<std::size_t>(std::max(1, tsp_lookahead)),
      tsp_order.size());

  std::vector<FrontierCluster> layer_clusters;
  std::vector<std::size_t> layer_original_indices;
  for (std::size_t i = 0; i < k; ++i) {
    const std::size_t ci = viable_cluster_indices[tsp_order[i]];
    layer_clusters.push_back(clusters[ci]);
    layer_original_indices.push_back(ci);
  }

  const auto dijkstra_result = solve_viewpoint_dijkstra(
      planning_pose.x, planning_pose.y, layer_clusters, map,
      occupied_threshold, block_unknown_in_path, path_clearance_m,
      visited_viewpoints, effective_revisit_radius_m);

  if (dijkstra_result.has_value()) {
    const std::size_t first_cluster_idx = layer_original_indices[0];
    const std::size_t first_vp_idx =
        dijkstra_result->selected_viewpoint_indices[0];
    const auto &vp = clusters[first_cluster_idx].viewpoints[first_vp_idx];

    const auto path = plan_astar_path(
        planning_pose, vp, map, occupied_threshold, block_unknown_in_path,
        path_clearance_m);
    if (path.has_value() && path->length_m >= min_goal_distance_m) {
      const double utility =
          static_cast<double>(vp.score) /
              (1.0 + goal_distance_weight * dijkstra_result->total_cost) -
          viewpoint_rank_penalty * static_cast<double>(first_vp_idx);
      result.selected_goal =
          SelectedGoal{vp, first_cluster_idx, first_vp_idx, utility,
                       path->length_m, path->indices};
      return result;
    }
  }

  for (std::size_t tsp_rank = 0; tsp_rank < tsp_order.size(); ++tsp_rank) {
    const std::size_t tsp_idx = tsp_order[tsp_rank];
    const std::size_t cluster_idx = viable_cluster_indices[tsp_idx];
    const auto &cluster = clusters[cluster_idx];

    for (std::size_t vi = 0; vi < cluster.viewpoints.size(); ++vi) {
      const auto &vp = cluster.viewpoints[vi];
      if (is_near_visited_viewpoint(vp, visited_viewpoints,
                                    effective_revisit_radius_m)) {
        continue;
      }

      const auto path = plan_astar_path(
          planning_pose, vp, map, occupied_threshold, block_unknown_in_path,
          path_clearance_m);
      if (!path.has_value() || path->length_m < min_goal_distance_m) {
        continue;
      }

      const double utility =
          static_cast<double>(vp.score) /
              (1.0 + goal_distance_weight * path->length_m +
               static_cast<double>(tsp_rank)) -
          viewpoint_rank_penalty * static_cast<double>(vi);
      result.selected_goal =
          SelectedGoal{vp, cluster_idx, vi, utility, path->length_m,
                       path->indices};
      return result;
    }
  }

  return result;
}

}  // namespace fuel_exploration::frontier
