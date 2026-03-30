#include <climits>
#include <cmath>
#include <cstdint>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

struct FrontierCluster {
  std::vector<int> cells; // 前沿cell的index信息
  double centroid_x;      // 质心的世界坐标
  double centroid_y;      // 质心的世界坐标
  std::pair<double, double> best_viewpoint;
};

const std::vector<std::pair<int, int>> Neighborhood{
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

class FrontierDetector : public ::rclcpp::Node {
public:
  FrontierDetector() : Node("frontier_detector") {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&FrontierDetector::map_callback, this,
                  std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "frontiers", 10);
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  // 维护当前所有 frontier cell，增量增删
  std::set<int> frontier_set_;

  // 对比新旧地图，找变化的 cell
  std::vector<int8_t> prev_map_data_;

  int min_cluster_size_ = 10;
};

// 返回所有的合法的候选视点世界坐标
std::vector<std::pair<double, double>>
genrate_candidates(const FrontierCluster &cluster,
                   const nav_msgs::msg::OccupancyGrid &msg) {
  std::vector<std::pair<double, double>> candidates;
  // 双重循环访问cluster中的每个cell
  // for 每个半径 r : {1.0, 2.0}
  //   for 每个角度 θ : {0, 45°, 90°, ..., 315°}
  //   算世界坐标 (x, y)
  //   转栅格坐标
  //   检查合法性
  //   合法就收集
  for (double r : {1.0, 2.0}) {
    for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
      double x = cluster.centroid_x + r * cos(theta);
      double y = cluster.centroid_y + r * sin(theta);
      int col = (x - msg.info.origin.position.x) / msg.info.resolution;
      int row = (y - msg.info.origin.position.y) / msg.info.resolution;
      if (col >= 0 && col < static_cast<int>(msg.info.width) && row >= 0 &&
          row < static_cast<int>(msg.info.height)) {
        int idx = row * msg.info.width + col;
        if (msg.data[idx] == 0) {
          candidates.push_back({x, y});
        }
      }
    }
  }

  return candidates;
}

bool has_line_of_sight(int r0, int c0, int r1, int c1,
                       const nav_msgs::msg::OccupancyGrid &msg) {
  int width = msg.info.width;
  int dy = std::abs(r1 - r0);
  int dx = std::abs(c1 - c0);
  int sx = (c0 < c1) ? 1 : -1;
  int sy = (r0 < r1) ? 1 : -1;
  int err = dx - dy;
  while (r0 != r1 || c0 != c1) {
    if (msg.data[r0 * width + c0] == 100)
      return false;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      c0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      r0 += sy;
    }
  }
  return true;
}

int score_candidate(const FrontierCluster &cluster,
                    const std::pair<double, double> &candidate,
                    const nav_msgs::msg::OccupancyGrid &msg) {
  int score = 0;
  int width = msg.info.width;
  int col =
      (candidate.first - msg.info.origin.position.x) / msg.info.resolution;
  int row =
      (candidate.second - msg.info.origin.position.y) / msg.info.resolution;
  for (int idx : cluster.cells) {
    int fr = idx / width;
    int fc = idx % width;
    if (has_line_of_sight(row, col, fr, fc, msg)) {

      score++;
    }
  }
  return score;
}

void FrontierDetector::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received map: %d x %d", msg->info.width,
              msg->info.height);
  const auto &map_data = msg->data;
  int width = msg->info.width;
  int height = msg->info.height;

  // 判断一个 cell 是否是 frontier
  auto is_frontier = [&](int idx) -> bool {
    if (map_data[idx] != 0)
      return false;
    int row = idx / width;
    int col = idx % width;
    for (auto &[dr, dc] : Neighborhood) {
      int nr = row + dr;
      int nc = col + dc;
      if (nr >= 0 && nr < height && nc >= 0 && nc < width) {
        if (map_data[nr * width + nc] == -1)
          return true;
      }
    }
    return false;
  };

  if (prev_map_data_.empty()) {
    // 第一帧：全图扫描
    for (int i = 0; i < width * height; i++) {
      if (is_frontier(i))
        frontier_set_.insert(i);
    }
    RCLCPP_INFO(this->get_logger(), "Initial frontier cells: %zu",
                frontier_set_.size());
  } else {
    // 增量更新：只处理变化的 cell 及其邻居
    std::set<int> wait_check;
    for (int i = 0; i < width * height; i++) {
      if (map_data[i] == prev_map_data_[i])
        continue;
      wait_check.insert(i);
      int row = i / width;
      int col = i % width;
      for (auto &[dr, dc] : Neighborhood) {
        int nr = row + dr;
        int nc = col + dc;
        if (nr >= 0 && nr < height && nc >= 0 && nc < width)
          wait_check.insert(nr * width + nc);
      }
    }
    for (int idx : wait_check) {
      if (is_frontier(idx))
        frontier_set_.insert(idx);
      else
        frontier_set_.erase(idx);
    }
    RCLCPP_INFO(this->get_logger(),
                "Incremental update: %zu cells checked, %zu frontiers",
                wait_check.size(), frontier_set_.size());
  }

  // 保存当前 map 用于下次对比
  prev_map_data_ = msg->data;

  // 聚类（基于 frontier_set_）
  std::vector<FrontierCluster> clusters;
  std::set<int> visited;
  for (int idx : frontier_set_) {
    if (visited.count(idx))
      continue;
    std::vector<int> current_cluster;
    std::queue<int> q;
    q.push(idx);
    visited.insert(idx);

    while (!q.empty()) {
      int curr = q.front();
      q.pop();
      current_cluster.push_back(curr);

      int row = curr / width;
      int col = curr % width;
      for (auto &[dr, dc] : Neighborhood) {
        int nr = row + dr;
        int nc = col + dc;
        if (nr >= 0 && nr < height && nc >= 0 && nc < width) {
          int neighbor_idx = nr * width + nc;
          if (frontier_set_.count(neighbor_idx) &&
              !visited.count(neighbor_idx)) {
            visited.insert(neighbor_idx);
            q.push(neighbor_idx);
          }
        }
      }
    }
    if (static_cast<int>(current_cluster.size()) < min_cluster_size_)
      continue;

    double sum_row = 0;
    double sum_col = 0;
    for (int ci : current_cluster) {
      // NOLINTBEGIN(bugprone-integer-division)
      sum_row += ci / width;
      sum_col += ci % width;
      // NOLINTEND(bugprone-integer-division)
    }
    double centroid_x =
        (sum_col / current_cluster.size()) * msg->info.resolution +
        msg->info.origin.position.x;
    double centroid_y =
        (sum_row / current_cluster.size()) * msg->info.resolution +
        msg->info.origin.position.y;
    clusters.push_back({current_cluster, centroid_x, centroid_y, {}});
  }

  for (auto &cluster : clusters) {
    auto candidates = genrate_candidates(cluster, *msg);
    int best_score = INT_MIN;
    std::pair<double, double> best_candidate;
    for (const auto &candidate : candidates) {
      int score = score_candidate(cluster, candidate, *msg);
      if (score > best_score) {
        best_score = score;
        best_candidate = candidate;
      }
    }
    cluster.best_viewpoint = best_candidate;
  }

  // 发布 marker
  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < clusters.size(); i++) {
    // 红色球：簇质心
    visualization_msgs::msg::Marker cm;
    cm.header.frame_id = msg->header.frame_id;
    cm.header.stamp = this->now();
    cm.ns = "centroids";
    cm.id = i;
    cm.type = visualization_msgs::msg::Marker::SPHERE;
    cm.action = visualization_msgs::msg::Marker::ADD;
    cm.pose.position.x = clusters[i].centroid_x;
    cm.pose.position.y = clusters[i].centroid_y;
    cm.pose.position.z = 0.1;
    cm.scale.x = 0.3;
    cm.scale.y = 0.3;
    cm.scale.z = 0.3;
    cm.color.r = 1.0;
    cm.color.a = 1.0;
    marker_array.markers.push_back(cm);

    // 绿色球：最佳视点
    visualization_msgs::msg::Marker vm;
    vm.header.frame_id = msg->header.frame_id;
    vm.header.stamp = this->now();
    vm.ns = "viewpoints";
    vm.id = i;
    vm.type = visualization_msgs::msg::Marker::SPHERE;
    vm.action = visualization_msgs::msg::Marker::ADD;
    vm.pose.position.x = clusters[i].best_viewpoint.first;
    vm.pose.position.y = clusters[i].best_viewpoint.second;
    vm.pose.position.z = 0.1;
    vm.scale.x = 0.25;
    vm.scale.y = 0.25;
    vm.scale.z = 0.25;
    vm.color.g = 1.0;
    vm.color.a = 1.0;
    marker_array.markers.push_back(vm);
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
