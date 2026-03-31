#pragma once

#include "fuel_exploration/common/occupancy_grid_utils.hpp"
#include "fuel_exploration/frontier/frontier_types.hpp"

#include <set>
#include <vector>

namespace fuel_exploration::frontier {

FrontierSetUpdateResult update_frontier_set(
    const OccupancyGrid &map, const std::vector<int8_t> &prev_map_data,
    std::set<int> &frontier_set);

std::vector<FrontierCluster> build_frontier_clusters(
    const std::set<int> &frontier_set, const OccupancyGrid &map,
    int min_cluster_size, const std::vector<double> &sample_radii_m,
    int angular_samples, double candidate_clearance_m,
    double max_view_range_m, int min_viewpoint_coverage, int max_viewpoints,
    int occupied_threshold, bool block_unknown_in_los);

ClusterStatistics summarize_clusters(
    const std::vector<FrontierCluster> &clusters);

}  // namespace fuel_exploration::frontier
