#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include <mutex>
#include <algorithm>

enum class State {
  INITIAL_SPIN,
  FIND_FRONTIER,
  PLAN_PATH,
  DRIVE,
  SPIN_SCAN,
  DONE
};

struct Cell {
  int x, y;
  double g, h;
  double f() const { return g + h; }
  bool operator>(const Cell& other) const { return f() > other.f(); }
};

struct Frontier {
  std::vector<std::pair<int, int>> cells;
  double centroid_x, centroid_y;
};

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("navigator")
  {
    spin_speed_ = this->declare_parameter("spin_speed", 0.5);
    drive_speed_ = this->declare_parameter("drive_speed", 0.15);
    refine_distance_ = this->declare_parameter("refine_distance", 1.5);
    obstacle_threshold_ = this->declare_parameter("obstacle_threshold", 0.15);
    waypoint_tolerance_ = this->declare_parameter("waypoint_tolerance", 0.08);
    heading_tolerance_ = this->declare_parameter("heading_tolerance", 0.15);
    inflation_radius_ = this->declare_parameter("inflation_radius", 3);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        pose_received_ = true;
      });

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_ = *msg;
        map_received_ = true;
      });

    std::vector<std::string> sensor_names = {
      "front_tof", "rear_right_tof", "rear_left_tof", "left_tof", "right_tof"
    };
    for (const auto& name : sensor_names) {
      auto sub = this->create_subscription<sensor_msgs::msg::Range>(
        name + "/range", 10,
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->range >= msg->min_range && msg->range <= msg->max_range) {
            min_obstacle_range_ = std::min(min_obstacle_range_,
                                           static_cast<double>(msg->range));
          }
        });
      range_subs_.push_back(sub);
    }

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&Navigator::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Navigator initialized. Waiting for map and pose...");
  }

private:
  void control_loop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pose_received_ || !map_received_) return;

    switch (state_) {
      case State::INITIAL_SPIN: execute_spin(); break;
      case State::FIND_FRONTIER: find_frontier(); break;
      case State::PLAN_PATH: plan_path(); break;
      case State::DRIVE: execute_drive(); break;
      case State::SPIN_SCAN: execute_spin(); break;
      case State::DONE: stop_robot(); break;
    }
  }

  void execute_spin()
  {
    if (!spin_started_) {
      spin_start_yaw_ = robot_yaw_;
      spin_accumulated_ = 0.0;
      last_spin_yaw_ = robot_yaw_;
      spin_started_ = true;
      RCLCPP_INFO(this->get_logger(), "Starting %s at yaw %.2f",
                  state_ == State::INITIAL_SPIN ? "initial scan" : "refine scan",
                  robot_yaw_);
    }

    double delta = robot_yaw_ - last_spin_yaw_;
    if (delta > M_PI) delta -= 2 * M_PI;
    if (delta < -M_PI) delta += 2 * M_PI;
    spin_accumulated_ += delta;
    last_spin_yaw_ = robot_yaw_;

    if (std::abs(spin_accumulated_) < 2 * M_PI) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = spin_speed_;
      cmd_pub_->publish(cmd);
    } else {
      stop_robot();
      spin_started_ = false;
      accumulated_distance_ = 0.0;
      last_x_ = robot_x_;
      last_y_ = robot_y_;

      RCLCPP_INFO(this->get_logger(), "Scan complete (%.1f deg). Finding frontier...",
                  std::abs(spin_accumulated_) * 180.0 / M_PI);
      state_ = State::FIND_FRONTIER;
    }
  }

  void find_frontier()
  {
    std::vector<Frontier> frontiers = detect_frontiers();

    if (frontiers.empty()) {
      RCLCPP_INFO(this->get_logger(), "No frontiers found. Exploration complete!");
      state_ = State::DONE;
      return;
    }

    double best_dist = 1e9;
    bool found_target = false;

    for (auto& frontier : frontiers) {
      for (auto& [gx, gy] : frontier.cells) {
        double wx = map_.info.origin.position.x + (gx + 0.5) * map_.info.resolution;
        double wy = map_.info.origin.position.y + (gy + 0.5) * map_.info.resolution;
        double dx = wx - robot_x_;
        double dy = wy - robot_y_;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 0.3 && dist < best_dist) {
          best_dist = dist;
          target_x_ = wx;
          target_y_ = wy;
          found_target = true;
        }
      }
    }

    if (!found_target) {
      RCLCPP_INFO(this->get_logger(), "No reachable frontier cells. Exploration complete!");
      state_ = State::DONE;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Target frontier cell at (%.2f, %.2f), dist=%.2f",
                target_x_, target_y_, best_dist);
    state_ = State::PLAN_PATH;
  }

  std::vector<Frontier> detect_frontiers()
  {
    int w = map_.info.width;
    int h = map_.info.height;
    double res = map_.info.resolution;
    double ox = map_.info.origin.position.x;
    double oy = map_.info.origin.position.y;

    std::set<int> frontier_indices;

    for (int y = 1; y < h - 1; y++) {
      for (int x = 1; x < w - 1; x++) {
        int idx = y * w + x;
        if (map_.data[idx] < 0 || map_.data[idx] > 40) continue;

        bool has_unknown_neighbor = false;
        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int ni = (y + dy) * w + (x + dx);
            if (map_.data[ni] == -1) {
              has_unknown_neighbor = true;
              break;
            }
          }
          if (has_unknown_neighbor) break;
        }

        if (has_unknown_neighbor) {
          frontier_indices.insert(idx);
        }
      }
    }

    std::vector<Frontier> frontiers;
    std::set<int> visited;

    for (int idx : frontier_indices) {
      if (visited.count(idx)) continue;

      Frontier frontier;
      std::queue<int> bfs;
      bfs.push(idx);
      visited.insert(idx);

      while (!bfs.empty()) {
        int current = bfs.front();
        bfs.pop();

        int cx = current % w;
        int cy = current / w;
        frontier.cells.push_back({cx, cy});

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int ni = (cy + dy) * w + (cx + dx);
            if (frontier_indices.count(ni) && !visited.count(ni)) {
              visited.insert(ni);
              bfs.push(ni);
            }
          }
        }
      }

      if (frontier.cells.size() < 3) continue;

      double sum_x = 0, sum_y = 0;
      for (auto& [gx, gy] : frontier.cells) {
        sum_x += ox + (gx + 0.5) * res;
        sum_y += oy + (gy + 0.5) * res;
      }
      frontier.centroid_x = sum_x / frontier.cells.size();
      frontier.centroid_y = sum_y / frontier.cells.size();

      frontiers.push_back(frontier);
    }

    RCLCPP_INFO(this->get_logger(), "Found %zu frontier clusters", frontiers.size());
    return frontiers;
  }

  void plan_path()
  {
    int w = map_.info.width;
    int h = map_.info.height;
    double res = map_.info.resolution;
    double ox = map_.info.origin.position.x;
    double oy = map_.info.origin.position.y;

    int sx = static_cast<int>((robot_x_ - ox) / res);
    int sy = static_cast<int>((robot_y_ - oy) / res);
    int gx = static_cast<int>((target_x_ - ox) / res);
    int gy = static_cast<int>((target_y_ - oy) / res);

    sx = std::clamp(sx, 0, w - 1);
    sy = std::clamp(sy, 0, h - 1);
    gx = std::clamp(gx, 0, w - 1);
    gy = std::clamp(gy, 0, h - 1);

    std::vector<bool> blocked(w * h, false);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int idx = y * w + x;
        if (map_.data[idx] > 65) {
          for (int dy = -inflation_radius_; dy <= inflation_radius_; dy++) {
            for (int dx = -inflation_radius_; dx <= inflation_radius_; dx++) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                blocked[ny * w + nx] = true;
              }
            }
          }
        }
      }
    }

    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int idx = y * w + x;
        if (map_.data[idx] < 0) {
          blocked[idx] = true;
        }
      }
    }

    blocked[sy * w + sx] = false;
    blocked[gy * w + gx] = false;

    std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> open;
    std::vector<double> g_cost(w * h, 1e9);
    std::vector<bool> closed(w * h, false);
    std::vector<int> parent(w * h, -1);

    auto heuristic = [gx, gy](int x, int y) -> double {
      return std::sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy));
    };

    g_cost[sy * w + sx] = 0;
    open.push({sx, sy, 0.0, heuristic(sx, sy)});

    bool found = false;
    int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    double cost8[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

    while (!open.empty()) {
      Cell current = open.top();
      open.pop();

      int cidx = current.y * w + current.x;
      if (closed[cidx]) continue;
      closed[cidx] = true;

      if (current.x == gx && current.y == gy) {
        found = true;
        break;
      }

      for (int i = 0; i < 8; i++) {
        int nx = current.x + dx8[i];
        int ny = current.y + dy8[i];
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;

        int nidx = ny * w + nx;
        if (closed[nidx] || blocked[nidx]) continue;

        double new_g = g_cost[cidx] + cost8[i];
        if (new_g < g_cost[nidx]) {
          g_cost[nidx] = new_g;
          parent[nidx] = cidx;
          open.push({nx, ny, new_g, heuristic(nx, ny)});
        }
      }
    }

    if (!found) {
      RCLCPP_WARN(this->get_logger(), "No path found to frontier.");
      state_ = State::FIND_FRONTIER;
      return;
    }

    // Extract path
    path_.clear();
    int current_idx = gy * w + gx;
    while (current_idx != -1) {
      int px = current_idx % w;
      int py = current_idx / w;
      double wx = ox + (px + 0.5) * res;
      double wy = oy + (py + 0.5) * res;
      path_.push_back({wx, wy});
      current_idx = parent[current_idx];
    }
    std::reverse(path_.begin(), path_.end());

    // Line-of-sight smoothing
    std::vector<std::pair<double, double>> smooth_path;
    smooth_path.push_back(path_.front());
    size_t i = 0;
    while (i < path_.size() - 1) {
      size_t furthest = i + 1;
      for (size_t j = i + 2; j < path_.size(); j++) {
        int x0 = static_cast<int>((smooth_path.back().first - ox) / res);
        int y0 = static_cast<int>((smooth_path.back().second - oy) / res);
        int x1 = static_cast<int>((path_[j].first - ox) / res);
        int y1 = static_cast<int>((path_[j].second - oy) / res);

        bool clear = true;
        int ddx = std::abs(x1 - x0);
        int ddy = std::abs(y1 - y0);
        int step_x = (x0 < x1) ? 1 : -1;
        int step_y = (y0 < y1) ? 1 : -1;
        int err = ddx - ddy;
        int cx = x0, cy = y0;

        while (cx != x1 || cy != y1) {
          if (cx >= 0 && cx < w && cy >= 0 && cy < h) {
            if (blocked[cy * w + cx]) {
              clear = false;
              break;
            }
          }
          int e2 = 2 * err;
          if (e2 > -ddy) { err -= ddy; cx += step_x; }
          if (e2 < ddx) { err += ddy; cy += step_y; }
        }

        if (clear) furthest = j;
      }
      smooth_path.push_back(path_[furthest]);
      i = furthest;
    }
    path_ = smooth_path;
    path_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "Path planned: %zu waypoints", path_.size());
    state_ = State::DRIVE;
  }

  void execute_drive()
  {
    double current_min_range = min_obstacle_range_;
    min_obstacle_range_ = 999.0;

    if (current_min_range < obstacle_threshold_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2fm! Stopping to rescan.",
                  current_min_range);
      stop_robot();
      spin_started_ = false;
      state_ = State::SPIN_SCAN;
      return;
    }

    if (accumulated_distance_ > refine_distance_) {
      RCLCPP_INFO(this->get_logger(), "Accumulated %.2fm — stopping to refine.",
                  accumulated_distance_);
      stop_robot();
      spin_started_ = false;
      state_ = State::SPIN_SCAN;
      return;
    }

    if (path_index_ >= path_.size()) {
      RCLCPP_INFO(this->get_logger(), "Reached frontier target.");
      stop_robot();
      spin_started_ = false;
      state_ = State::SPIN_SCAN;
      return;
    }

    double wp_x = path_[path_index_].first;
    double wp_y = path_[path_index_].second;
    double dist_to_wp = std::sqrt((wp_x - robot_x_) * (wp_x - robot_x_) +
                                   (wp_y - robot_y_) * (wp_y - robot_y_));

    if (dist_to_wp < waypoint_tolerance_) {
      path_index_++;
      if (path_index_ >= path_.size()) {
        RCLCPP_INFO(this->get_logger(), "Reached frontier target.");
        stop_robot();
        spin_started_ = false;
        state_ = State::SPIN_SCAN;
        return;
      }
      wp_x = path_[path_index_].first;
      wp_y = path_[path_index_].second;
    }

    // Heading — robot forward is +Y
    double target_heading = std::atan2(wp_y - robot_y_, wp_x - robot_x_);
    double heading_error = target_heading - M_PI / 2 - robot_yaw_;

    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;

    if (std::abs(heading_error) > heading_tolerance_) {
      cmd.angular.z = std::copysign(std::min(std::abs(heading_error), spin_speed_),
                                     heading_error);
    } else {
      cmd.linear.y = drive_speed_;
      cmd.angular.z = heading_error * 0.5;
    }

    // Track distance only when driving
    if (cmd.linear.y > 0) {
      double dx = robot_x_ - last_x_;
      double dy = robot_y_ - last_y_;
      accumulated_distance_ += std::sqrt(dx * dx + dy * dy);
    }
    last_x_ = robot_x_;
    last_y_ = robot_y_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "DRIVE: pos(%.2f,%.2f) yaw=%.2f -> wp(%.2f,%.2f) hdg_err=%.2f dist=%.2f",
      robot_x_, robot_y_, robot_yaw_, wp_x, wp_y, heading_error, dist_to_wp);

    cmd_pub_->publish(cmd);
  }

  void stop_robot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  // Member variables
  State state_ = State::INITIAL_SPIN;
  bool spin_started_ = false;
  double spin_start_yaw_ = 0.0;
  double spin_accumulated_ = 0.0;
  double last_spin_yaw_ = 0.0;
  double accumulated_distance_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;

  double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
  bool pose_received_ = false;

  nav_msgs::msg::OccupancyGrid map_;
  bool map_received_ = false;

  std::vector<std::pair<double, double>> path_;
  size_t path_index_ = 0;
  double target_x_ = 0.0, target_y_ = 0.0;

  double min_obstacle_range_ = 999.0;

  double spin_speed_;
  double drive_speed_;
  double refine_distance_;
  double obstacle_threshold_;
  double waypoint_tolerance_;
  double heading_tolerance_;
  int inflation_radius_;

  std::mutex mutex_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}