#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>

class SparseSlam : public rclcpp::Node
{
public:
  SparseSlam() : Node("sparse_slam")
  {
    // Grid parameters
    resolution_ = declare_parameter("resolution", 0.05);       // meters per cell
    grid_width_ = declare_parameter("grid_width", 400);        // cells
    grid_height_ = declare_parameter("grid_height", 400);      // cells
    // Origin of the grid in world coords (bottom-left corner)
    origin_x_ = -(grid_width_ * resolution_) / 2.0;
    origin_y_ = -(grid_height_ * resolution_) / 2.0;

    // Log-odds parameters
    l_free_ = declare_parameter("l_free", -0.4);
    l_occ_ = declare_parameter("l_occ", 0.85);
    l_min_ = -5.0;
    l_max_ = 5.0;

    // Initialize log-odds grid to zero (unknown)
    log_odds_.resize(grid_width_ * grid_height_, 0.0);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
        pose_received_ = true;
      });

    // Subscribe to all 5 ToF sensors
    std::vector<std::string> sensor_names = {
      "front_tof", "rear_right_tof", "rear_left_tof", "left_tof", "right_tof"
    };

    for (const auto& name : sensor_names) {
      auto sub = create_subscription<sensor_msgs::msg::Range>(
        name + "/range", 10,
        [this, name](const sensor_msgs::msg::Range::SharedPtr msg) {
          handle_range(msg, name + "_link");
        });
      range_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Subscribed to %s/range", name.c_str());
    }

    // Publish occupancy grid
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Timer to publish grid at 2 Hz
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SparseSlam::publish_grid, this));

    RCLCPP_INFO(get_logger(), "Sparse SLAM initialized: %dx%d grid, %.2fm resolution",
                grid_width_, grid_height_, resolution_);
  }

private:
  void handle_range(const sensor_msgs::msg::Range::SharedPtr msg,
                    const std::string& frame_id)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_received_) return;

    // Skip invalid readings
    if (msg->range < msg->min_range || msg->range > msg->max_range) return;

    // Get sensor pose in odom frame via TF
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("odom", frame_id,
                                               tf2::TimePointZero,
                                               tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      return;
    }

    double sensor_x = transform.transform.translation.x;
    double sensor_y = transform.transform.translation.y;
    double sensor_yaw = tf2::getYaw(transform.transform.rotation);

    // Endpoint of the ray
    double hit_x = sensor_x + msg->range * std::cos(sensor_yaw);
    double hit_y = sensor_y + msg->range * std::sin(sensor_yaw);

    // Convert to grid coordinates
    int sx = world_to_grid_x(sensor_x);
    int sy = world_to_grid_y(sensor_y);
    int hx = world_to_grid_x(hit_x);
    int hy = world_to_grid_y(hit_y);

    // Bresenham: mark free cells along the ray
    bresenham_free(sx, sy, hx, hy);

    // Mark hit cell as occupied (if within max range)
    if (msg->range < msg->max_range - 0.05) {
      mark_cell(hx, hy, l_occ_);
    }
  }

  void bresenham_free(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
      // Stop before the hit cell — don't mark it as free
      if (x0 == x1 && y0 == y1) break;

      mark_cell(x0, y0, l_free_);

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
  }

  void mark_cell(int gx, int gy, double update)
  {
    if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) return;

    int idx = gy * grid_width_ + gx;
    log_odds_[idx] += update;

    // Clamp
    if (log_odds_[idx] > l_max_) log_odds_[idx] = l_max_;
    if (log_odds_[idx] < l_min_) log_odds_[idx] = l_min_;
  }

  void publish_grid()
  {
    auto grid_msg = nav_msgs::msg::OccupancyGrid();
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "odom";

    grid_msg.info.resolution = resolution_;
    grid_msg.info.width = grid_width_;
    grid_msg.info.height = grid_height_;
    grid_msg.info.origin.position.x = origin_x_;
    grid_msg.info.origin.position.y = origin_y_;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(grid_width_ * grid_height_);

    for (int i = 0; i < grid_width_ * grid_height_; i++) {
      if (std::abs(log_odds_[i]) < 0.01) {
        grid_msg.data[i] = -1;  // Unknown
      } else {
        // Convert log-odds to probability [0, 100]
        double prob = 1.0 / (1.0 + std::exp(-log_odds_[i]));
        grid_msg.data[i] = static_cast<int8_t>(prob * 100.0);
      }
    }

    grid_pub_->publish(grid_msg);
  }

  int world_to_grid_x(double wx) {
    return static_cast<int>((wx - origin_x_) / resolution_);
  }

  int world_to_grid_y(double wy) {
    return static_cast<int>((wy - origin_y_) / resolution_);
  }

  // Grid parameters
  double resolution_;
  int grid_width_, grid_height_;
  double origin_x_, origin_y_;

  // Log-odds
  double l_free_, l_occ_, l_min_, l_max_;
  std::vector<double> log_odds_;

  // Robot pose
  double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
  bool pose_received_ = false;
  std::mutex pose_mutex_;

  // ROS interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SparseSlam>());
  rclcpp::shutdown();
  return 0;
}