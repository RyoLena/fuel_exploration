#pragma once

#include "fuel_exploration/common/navigation_utils.hpp"
#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace fuel_exploration::frontier {

bool is_near_visited_viewpoint(
    const ViewpointCandidate &candidate,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    double revisit_block_radius_m);

std::optional<PlannedPath> plan_astar_path(
    const RobotPose &robot_pose, const ViewpointCandidate &goal_viewpoint,
    const OccupancyGrid &map, int occupied_threshold, bool block_unknown,
    double path_clearance_m);

std::optional<double> compute_path_cost(
    double start_x, double start_y, double goal_x, double goal_y,
    const OccupancyGrid &map, int occupied_threshold, bool block_unknown,
    double path_clearance_m);

std::vector<ViewpointCandidate> generate_candidates(
    const FrontierCluster &cluster, const OccupancyGrid &map,
    const std::vector<double> &sample_radii_m, int angular_samples,
    double candidate_clearance_m, double max_view_range_m,
    int min_viewpoint_coverage, int max_viewpoints, int occupied_threshold,
    bool block_unknown);

std::vector<std::size_t> solve_tsp_nearest_neighbor(
    const std::vector<std::vector<double>> &cost_matrix);

std::optional<DijkstraResult> solve_viewpoint_dijkstra(
    double robot_x, double robot_y,
    const std::vector<FrontierCluster> &layer_clusters,
    const OccupancyGrid &map, int occupied_threshold, bool block_unknown,
    double path_clearance_m,
    const std::vector<VisitedViewpoint> &visited_viewpoints,
    double revisit_block_radius_m);

}  // namespace fuel_exploration::frontier
