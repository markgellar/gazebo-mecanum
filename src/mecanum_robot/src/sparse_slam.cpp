#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <mutex>
#include <filesystem>
#include <fstream>

// One ToF reading as a ray, in the odom frame
// The sensor reports the NEAREST obstacle anywhere in its cone, so a reading means
// "nothing closer than range inside the cone, something on the arc at range".
struct Ray {
  double sx, sy;     // sensor
  double ex, ey;     // centre-line end (range along the sensor axis, or max range)
  bool hit;
  double half_fov;   // cone half-angle, rad
  int sensor = -1;   // index into the sensor list (front, rear_right, rear_left, left, right)
  double stamp = 0;  // reading time, s
};

// Outcome of matching one spin scan against the map
struct MatchResult {
  double dx = 0.0, dy = 0.0, dyaw = 0.0;   // correction: rotate about pivot by dyaw, then shift
  double pivot_x = 0.0, pivot_y = 0.0;
  double score = 0.0, zero_score = 0.0;   // mean field value per hit: best pose vs no correction
  double margin = 0.0;                    // how much the winner beats the best clearly-different alignment
  double peak_rot = 0.0, peak_trans = 0.0; // how far near-best fits reach from the winner
  double fraction = 0.0;                  // share of hits on walls at the best pose
  size_t points = 0;
  bool at_edge = false;
  bool accepted = false;
  std::string reason;                     // set if matching wasn't attempted
  std::string reject;                     // set if the result failed a trust check
  double ms = 0.0;
  double window_rot = 0.0, window_trans = 0.0;  // search window used
};

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

    // Scan matching (correlative search around the current pose estimate)
    // Window only needs to cover drift since the last correction (~1 deg, a few cm per spin)
    // Dry run by default: with 5 single-zone ToF cones, one spin only pins rotation to about +-4 deg and a
    // false match corrupts the map (see slam_records analysis). Matches are still logged and recorded.
    match_apply_ = declare_parameter("match_apply", false);               // true = apply accepted corrections
    match_max_rot_ = declare_parameter("match_max_rot_deg", 4.0) * M_PI / 180.0;
    match_rot_step_ = declare_parameter("match_rot_step_deg", 0.5) * M_PI / 180.0;
    match_max_trans_ = declare_parameter("match_max_trans", 0.2);         // m
    match_prior_weight_ = declare_parameter("match_prior_weight", 0.005); // tie-breaker toward small corrections; real fit gains are only ~0.02-0.1
    // Ambiguity: near-best fits (within match_min_distinct of the best) must stay within this of the winner.
    // A good match is one compact peak (flat-topped: arcs span the cone); corridors and repeated boxes spread out.
    match_ambiguity_rot_ = declare_parameter("match_ambiguity_deg", 3.0) * M_PI / 180.0;
    match_ambiguity_trans_ = declare_parameter("match_ambiguity_trans", 0.15);  // m
    match_min_distinct_ = declare_parameter("match_min_distinct", 0.01);
    // After each spin without an applied correction, drift keeps growing: widen the window to keep up
    // Off by default: wide windows let false matches through and stall this node for seconds
    match_widen_rot_ = declare_parameter("match_widen_rot_deg", 0.0) * M_PI / 180.0;
    match_widen_trans_ = declare_parameter("match_widen_trans", 0.0);
    match_widen_max_rot_ = declare_parameter("match_widen_max_rot_deg", 12.0) * M_PI / 180.0;
    match_widen_max_trans_ = declare_parameter("match_widen_max_trans", 0.5);

    // Spin recording for offline analysis (tools/slam_replay.py). Empty = off.
    record_dir_ = declare_parameter("record_dir", std::string(""));
    if (!record_dir_.empty()) {
      std::filesystem::create_directories(record_dir_);
      RCLCPP_INFO(get_logger(), "Recording spin scans to %s", record_dir_.c_str());
    }
    match_sigma_ = declare_parameter("match_sigma", 0.08);                // m; how quickly the score falls off away from a wall
    match_occ_threshold_ = declare_parameter("match_occ_threshold", 1.0); // log-odds; cells counted as wall
    match_min_points_ = declare_parameter("match_min_points", 100);       // hits needed to attempt a match
    match_arc_samples_ = declare_parameter("match_arc_samples", 7);       // points per reading's arc when scoring
    match_free_weight_ = declare_parameter("match_free_weight", 1.0);     // penalty for walls inside a reading's cone
    match_free_margin_ = declare_parameter("match_free_margin", 0.2);     // m; only look this far short of the reading
    default_half_fov_ = declare_parameter("default_fov", 0.44) / 2.0;     // used if a Range msg has no field_of_view
    match_min_fraction_ = declare_parameter("match_min_fraction", 0.4);   // share of hits that must land on walls
    match_min_gain_ = declare_parameter("match_min_gain", 0.02);          // score improvement over no correction

    // Initialize log-odds grid to zero (unknown)
    log_odds_.resize(grid_width_ * grid_height_, 0.0);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
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

    for (size_t i = 0; i < sensor_names.size(); i++) {
      const std::string name = sensor_names[i];
      auto sub = create_subscription<sensor_msgs::msg::Range>(
        name + "/range", 10,
        [this, name, i](const sensor_msgs::msg::Range::SharedPtr msg) {
          handle_range(msg, name + "_link", static_cast<int>(i));
        });
      range_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Subscribed to %s/range", name.c_str());
    }

    // Navigator signals the start (true) and end (false) of each 360° spin
    spin_scan_sub_ = create_subscription<std_msgs::msg::Bool>(
      "spin_scan", 10,
      std::bind(&SparseSlam::handle_spin_scan, this, std::placeholders::_1));

    // Publish occupancy grid
    grid_pub_ =create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Timer to publish grid at 2 Hz
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SparseSlam::publish_grid, this));

    // Broadcast map→odom at 20 Hz
    map_odom_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SparseSlam::publish_map_to_odom, this));

    RCLCPP_INFO(get_logger(), "Sparse SLAM initialized: %dx%d grid, %.2fm resolution",
                grid_width_, grid_height_, resolution_);
  }

private:
  void handle_range(const sensor_msgs::msg::Range::SharedPtr msg,
                    const std::string& frame_id, int sensor)
  {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "ToF %s: range=%.3f min=%.3f max=%.3f",
      frame_id.c_str(), msg->range, msg->min_range, msg->max_range);

    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_received_) return;

    // Get sensor pose in odom (EKF) frame via TF, at the moment the reading was taken.
    // Rays are kept in odom so a buffered scan can be re-placed under any map→odom correction.
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("odom", frame_id,
                                               msg->header.stamp,
                                               tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Dropping %s reading, no TF: %s", frame_id.c_str(), ex.what());
      return;
    }

    double sensor_x = transform.transform.translation.x;
    double sensor_y = transform.transform.translation.y;
    double sensor_yaw = tf2::getYaw(transform.transform.rotation);

    // Clamp to max range for free-space tracing
    double range = msg->range;
    bool hit_obstacle = true;

    if (range >= msg->max_range || std::isinf(range)) {
      range = msg->max_range;
      hit_obstacle = false;
    } else if (range < msg->min_range) {
        return;  // Too close, invalid
    }

    // Endpoint of the ray
    Ray ray;
    ray.sx = sensor_x;
    ray.sy = sensor_y;
    ray.ex = sensor_x + range * std::cos(sensor_yaw);
    ray.ey = sensor_y + range * std::sin(sensor_yaw);
    ray.hit = hit_obstacle;
    ray.half_fov = msg->field_of_view > 0.0f ? msg->field_of_view / 2.0 : default_half_fov_;
    ray.sensor = sensor;
    ray.stamp = rclcpp::Time(msg->header.stamp).seconds();

    // During a spin, hold readings back so the scan can be matched before it enters the map
    if (scanning_) {
      scan_buffer_.push_back(ray);
      return;
    }
    insert_ray(ray);
  }

  // Spin start: begin buffering. Spin end: insert the whole scan, then publish right away.
  void handle_spin_scan(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (msg->data) {
      scan_buffer_.clear();
      scanning_ = true;
      RCLCPP_INFO(get_logger(), "Spin scan started, buffering readings");
      return;
    }
    if (!scanning_) return;
    scanning_ = false;
    spin_end_time_ = this->now();   // before matching, which can take a while

    // Match against the map as it was before this scan; if trusted, correct map→odom first
    // so the scan goes into the map at the corrected pose
    MatchResult m = match_scan(scan_buffer_);
    if (!record_dir_.empty()) record_spin(m);   // before any correction or insertion
    if (m.reason.empty()) {
      bool apply = m.accepted && match_apply_;
      if (apply) apply_correction(m);
      spins_since_correction_ = apply ? 0 : spins_since_correction_ + 1;
      RCLCPP_INFO(get_logger(),
        "Scan match%s: %s dx=%+.3f dy=%+.3f dyaw=%+.2f deg | "
        "score %.2f (uncorrected %.2f), margin %.3f, peak +-%.1f deg/%.2f m, %.0f%% of %zu hits on walls, %.0f ms, "
        "window +-%.1f deg/%.2f m | %s",
        match_apply_ ? "" : " (dry run)", apply ? "corrected" : "would correct",
        m.dx, m.dy, m.dyaw * 180.0 / M_PI, m.score, m.zero_score, m.margin,
        m.peak_rot * 180.0 / M_PI, m.peak_trans, m.fraction * 100.0,
        m.points, m.ms, m.window_rot * 180.0 / M_PI, m.window_trans,
        m.accepted ? "ACCEPT" : ("REJECT: " + m.reject).c_str());
    } else {
      spins_since_correction_++;
      RCLCPP_INFO(get_logger(), "Scan match skipped: %s", m.reason.c_str());
    }

    size_t hits = 0;
    for (const auto& ray : scan_buffer_) {
      insert_ray(ray);
      if (ray.hit) hits++;
    }
    RCLCPP_INFO(get_logger(), "Spin scan ended: inserted %zu readings (%zu hits)",
                scan_buffer_.size(), hits);

    last_scan_insert_time_ = this->now();
    publish_grid();
  }

  // Place an odom-frame ray into the map using the current map→odom correction
  // Cone sensor model: every cell inside the cone closer than the range is free; cells on the arc
  // at the range are (possibly) occupied. Wrong arc cells get carved back to free by overlapping
  // readings that see past them, so only real surfaces keep accumulating hits.
  void insert_ray(const Ray& ray)
  {
    double sx, sy, ex, ey;
    odom_to_map(ray.sx, ray.sy, sx, sy);
    odom_to_map(ray.ex, ray.ey, ex, ey);
    double range = std::hypot(ex - sx, ey - sy);
    double heading = std::atan2(ey - sy, ex - sx);
    double half_cell = resolution_ / 2.0;

    // Bounding box of the sector: sensor, both arc ends and the arc's middle
    double min_x = sx, max_x = sx, min_y = sy, max_y = sy;
    for (double a : {heading - ray.half_fov, heading, heading + ray.half_fov}) {
      double px = sx + range * std::cos(a), py = sy + range * std::sin(a);
      min_x = std::min(min_x, px); max_x = std::max(max_x, px);
      min_y = std::min(min_y, py); max_y = std::max(max_y, py);
    }
    int x0 = world_to_grid_x(min_x) - 1, x1 = world_to_grid_x(max_x) + 1;
    int y0 = world_to_grid_y(min_y) - 1, y1 = world_to_grid_y(max_y) + 1;

    for (int gy = y0; gy <= y1; gy++) {
      for (int gx = x0; gx <= x1; gx++) {
        double cx = origin_x_ + (gx + 0.5) * resolution_;
        double cy = origin_y_ + (gy + 0.5) * resolution_;
        double d = std::hypot(cx - sx, cy - sy);
        if (d > range + half_cell) continue;
        double off = std::abs(std::atan2(std::sin(std::atan2(cy - sy, cx - sx) - heading),
                                         std::cos(std::atan2(cy - sy, cx - sx) - heading)));
        if (off > ray.half_fov && d > resolution_) continue;   // outside the cone (sensor's own cell always counts)

        if (d < range - half_cell) {
          mark_cell(gx, gy, l_free_);
        } else if (ray.hit) {
          mark_cell(gx, gy, l_occ_);
        }
      }
    }
  }

  void odom_to_map(double ox, double oy, double& mx, double& my) const
  {
    double c = std::cos(map_odom_yaw_);
    double s = std::sin(map_odom_yaw_);
    mx = map_odom_x_ + c * ox - s * oy;
    my = map_odom_y_ + s * ox + c * oy;
  }

  // Write a float array as a .npy file (numpy's simple binary format) so Python can np.load() it
  static void write_npy(const std::string& path, const char* descr, size_t elem_size,
                        const void* data, size_t rows, size_t cols)
  {
    std::string header = "{'descr': '" + std::string(descr) + "', 'fortran_order': False, 'shape': (" +
                         std::to_string(rows) + ", " + std::to_string(cols) + "), }";
    while ((10 + header.size() + 1) % 64 != 0) header += ' ';   // pad so data is 64-byte aligned
    header += '\n';
    uint16_t len = static_cast<uint16_t>(header.size());
    std::ofstream f(path, std::ios::binary);
    f.write("\x93NUMPY\x01\x00", 8);
    f.write(reinterpret_cast<const char*>(&len), 2);
    f.write(header.data(), header.size());
    f.write(static_cast<const char*>(data), rows * cols * elem_size);
  }

  // Save this spin as it was before matching changed anything: the rays (odom frame), the map
  // around the robot, the map→odom correction in use, and what the matcher decided.
  void record_spin(const MatchResult& m)
  {
    char base[64];
    snprintf(base, sizeof base, "spin_%03d", ++record_count_);
    std::string prefix = record_dir_ + "/" + base;

    std::vector<double> rays;
    rays.reserve(scan_buffer_.size() * 8);
    double cx = 0.0, cy = 0.0;
    for (const auto& r : scan_buffer_) {
      rays.insert(rays.end(), {r.sx, r.sy, r.ex, r.ey, r.hit ? 1.0 : 0.0, r.half_fov,
                               static_cast<double>(r.sensor), r.stamp});
      double mx, my;
      odom_to_map(r.sx, r.sy, mx, my);
      cx += mx;
      cy += my;
    }
    if (!scan_buffer_.empty()) {
      cx /= scan_buffer_.size();
      cy /= scan_buffer_.size();
    }
    write_npy(prefix + "_rays.npy", "<f8", sizeof(double), rays.data(), scan_buffer_.size(), 8);

    // Map crop (log-odds) centred on the robot, big enough for sensor range + the widest search
    int half = static_cast<int>(std::ceil(3.5 / resolution_));
    int x0 = world_to_grid_x(cx) - half;
    int y0 = world_to_grid_y(cy) - half;
    int size = 2 * half + 1;
    std::vector<float> crop(size * size, 0.0f);
    for (int y = 0; y < size; y++) {
      for (int x = 0; x < size; x++) {
        int gx = x0 + x, gy = y0 + y;
        if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_) {
          crop[y * size + x] = static_cast<float>(log_odds_[gy * grid_width_ + gx]);
        }
      }
    }
    write_npy(prefix + "_map.npy", "<f4", sizeof(float), crop.data(), size, size);

    std::ofstream meta(prefix + ".json");
    meta << std::fixed;
    meta << "{\n"
         << "  \"sim_time\": " << spin_end_time_.seconds() << ",\n"
         << "  \"resolution\": " << resolution_ << ",\n"
         << "  \"crop_origin\": [" << origin_x_ + x0 * resolution_ << ", " << origin_y_ + y0 * resolution_ << "],\n"
         << "  \"map_odom\": [" << map_odom_x_ << ", " << map_odom_y_ << ", " << map_odom_yaw_ << "],\n"
         << "  \"params\": {\"sigma\": " << match_sigma_ << ", \"occ_threshold\": " << match_occ_threshold_
         << ", \"arc_samples\": " << match_arc_samples_ << ", \"rot_step\": " << match_rot_step_ << ", \"min_fraction\": " << match_min_fraction_
         << ", \"min_gain\": " << match_min_gain_ << ", \"min_distinct\": " << match_min_distinct_ << "},\n"
         << "  \"match\": {\"attempted\": " << (m.reason.empty() ? "true" : "false")
         << ", \"accepted\": " << (m.accepted ? "true" : "false")
         << ", \"applied\": " << (m.accepted && match_apply_ ? "true" : "false")
         << ", \"dx\": " << m.dx << ", \"dy\": " << m.dy << ", \"dyaw\": " << m.dyaw
         << ", \"pivot\": [" << m.pivot_x << ", " << m.pivot_y << "]"
         << ", \"score\": " << m.score << ", \"zero_score\": " << m.zero_score << ", \"margin\": " << m.margin
         << ", \"peak_rot\": " << m.peak_rot << ", \"peak_trans\": " << m.peak_trans
         << ", \"window_rot\": " << m.window_rot << ", \"window_trans\": " << m.window_trans
         << ", \"reject\": \"" << (m.reason.empty() ? m.reject : m.reason) << "\"}\n"
         << "}\n";
  }

  // Fold a match into map→odom. The match moves map points by p' = P + R(dyaw)(p - P) + t
  // (rotate about the pivot P, then shift). Applied on top of the old map→odom (R(yaw), o):
  //   p' = R(dyaw + yaw) p_odom + [P + R(dyaw)(o - P) + t]
  void apply_correction(const MatchResult& m)
  {
    double c = std::cos(m.dyaw), s = std::sin(m.dyaw);
    double ox = map_odom_x_ - m.pivot_x;
    double oy = map_odom_y_ - m.pivot_y;
    map_odom_x_ = m.pivot_x + c * ox - s * oy + m.dx;
    map_odom_y_ = m.pivot_y + s * ox + c * oy + m.dy;
    map_odom_yaw_ = std::atan2(std::sin(map_odom_yaw_ + m.dyaw), std::cos(map_odom_yaw_ + m.dyaw));
    publish_map_to_odom();
  }

  // Correlative scan matching: find the rotation (about the robot) + shift that best
  // lines the scan's hits up with walls already in the map.
  MatchResult match_scan(const std::vector<Ray>& scan)
  {
    auto t0 = std::chrono::steady_clock::now();
    MatchResult r;

    // Each hit becomes K points along its arc across the cone (map frame); the reading scores the best
    // of them. Pivot: the robot centre, i.e. the mean sensor position.
    const int K = std::max(1, match_arc_samples_);
    std::vector<std::pair<double, double>> pts;   // readings * K, one arc after another
    std::vector<std::pair<double, double>> inner; // points inside each cone, well short of its reading
    std::vector<size_t> inner_reading;            // which reading each inner point belongs to
    double px = 0.0, py = 0.0;
    for (const auto& ray : scan) {
      double sx, sy;
      odom_to_map(ray.sx, ray.sy, sx, sy);
      px += sx;
      py += sy;
      if (ray.hit) {
        double ex, ey;
        odom_to_map(ray.ex, ray.ey, ex, ey);
        double range = std::hypot(ex - sx, ey - sy);
        double heading = std::atan2(ey - sy, ex - sx);
        size_t reading = pts.size() / K;
        for (int k = 0; k < K; k++) {
          double a = heading + (K == 1 ? 0.0 : ray.half_fov * (2.0 * k / (K - 1) - 1.0));
          pts.push_back({sx + range * std::cos(a), sy + range * std::sin(a)});
          if (range > match_free_margin_ + 0.05) {
            for (double frac : {0.25, 0.5, 0.75}) {
              double rr = (range - match_free_margin_) * frac;
              inner.push_back({sx + rr * std::cos(a), sy + rr * std::sin(a)});
              inner_reading.push_back(reading);
            }
          }
        }
      }
    }
    size_t readings = pts.size() / K;
    r.points = readings;
    if (scan.empty() || static_cast<int>(readings) < match_min_points_) {
      r.reason = "only " + std::to_string(readings) + " hits";
      return r;
    }
    px /= scan.size();
    py /= scan.size();
    r.pivot_x = px;
    r.pivot_y = py;

    // Likelihood field in a window around the pivot: 1.0 on a wall cell, Gaussian falloff around it
    double reach = 0.0;
    for (const auto& [x, y] : pts) reach = std::max(reach, std::hypot(x - px, y - py));
    // Search window: the base size, widened for each spin since the last applied correction
    double max_rot = std::min(match_max_rot_ + spins_since_correction_ * match_widen_rot_,
                              std::max(match_max_rot_, match_widen_max_rot_));
    double max_trans = std::min(match_max_trans_ + spins_since_correction_ * match_widen_trans_,
                                std::max(match_max_trans_, match_widen_max_trans_));
    r.window_rot = max_rot;
    r.window_trans = max_trans;

    int half = static_cast<int>(std::ceil((reach + max_trans + 3.0 * match_sigma_) / resolution_));
    int size = 2 * half + 1;
    int win_x0 = world_to_grid_x(px) - half;   // grid cell at window (0, 0)
    int win_y0 = world_to_grid_y(py) - half;
    int k = static_cast<int>(std::ceil(3.0 * match_sigma_ / resolution_));
    double two_sigma2 = 2.0 * match_sigma_ * match_sigma_;

    std::vector<float> field(size * size, 0.0f);
    size_t wall_cells = 0;
    for (int wy = 0; wy < size; wy++) {
      for (int wx = 0; wx < size; wx++) {
        int gx = win_x0 + wx;
        int gy = win_y0 + wy;
        if (gx < 0 || gx >= grid_width_ || gy < 0 || gy >= grid_height_) continue;
        if (log_odds_[gy * grid_width_ + gx] <= match_occ_threshold_) continue;
        wall_cells++;
        for (int dy = -k; dy <= k; dy++) {
          for (int dx = -k; dx <= k; dx++) {
            int fx = wx + dx;
            int fy = wy + dy;
            if (fx < 0 || fx >= size || fy < 0 || fy >= size) continue;
            double d2 = (dx * dx + dy * dy) * resolution_ * resolution_;
            float v = static_cast<float>(std::exp(-d2 / two_sigma2));
            float& f = field[fy * size + fx];
            if (v > f) f = v;
          }
        }
      }
    }
    if (wall_cells < 20) {
      r.reason = "no mapped walls nearby yet";
      return r;
    }

    // Brute-force search. Rotate the points once per angle; each shift is then an integer cell offset.
    int n_rot = static_cast<int>(std::round(max_rot / match_rot_step_));
    int n_trans = static_cast<int>(std::round(max_trans / resolution_));
    // Per point: the field cell just below/left of it and how far it sits toward the next one,
    // so the score can be blended between cells (bilinear) and moves smoothly below cell size
    std::vector<int> cx(pts.size()), cy(pts.size());
    std::vector<float> wx(pts.size()), wy(pts.size());
    auto field_at = [&](int x, int y) -> float {
      return (x >= 0 && x < size && y >= 0 && y < size) ? field[y * size + x] : 0.0f;
    };
    double n = static_cast<double>(readings);

    // Every candidate's fit (mean field value per hit) and its value after the motion prior:
    // odometry is roughly right between spins, so bigger corrections must earn a better fit.
    struct Candidate { int a, tx, ty; double fit, value; };
    std::vector<Candidate> cands;
    cands.reserve((2 * n_rot + 1) * (2 * n_trans + 1) * (2 * n_trans + 1));

    for (int a = -n_rot; a <= n_rot; a++) {
      double c = std::cos(a * match_rot_step_);
      double s = std::sin(a * match_rot_step_);
      for (size_t i = 0; i < pts.size(); i++) {
        double rx = px + c * (pts[i].first - px) - s * (pts[i].second - py);
        double ry = py + s * (pts[i].first - px) + c * (pts[i].second - py);
        // Window coordinates measured from cell centres (field values live at centres)
        double fx = (rx - origin_x_) / resolution_ - win_x0 - 0.5;
        double fy = (ry - origin_y_) / resolution_ - win_y0 - 0.5;
        cx[i] = static_cast<int>(std::floor(fx));
        cy[i] = static_cast<int>(std::floor(fy));
        wx[i] = static_cast<float>(fx - cx[i]);
        wy[i] = static_cast<float>(fy - cy[i]);
      }
      for (int ty = -n_trans; ty <= n_trans; ty++) {
        for (int tx = -n_trans; tx <= n_trans; tx++) {
          double sum = 0.0;
          for (size_t j = 0; j < readings; j++) {
            float arc_best = 0.0f;   // the obstacle is somewhere on the arc: take its best point
            for (int k = 0; k < K; k++) {
              size_t i = j * K + k;
              int x = cx[i] + tx;
              int y = cy[i] + ty;
              float v = (1 - wx[i]) * (1 - wy[i]) * field_at(x, y) + wx[i] * (1 - wy[i]) * field_at(x + 1, y) +
                        (1 - wx[i]) * wy[i] * field_at(x, y + 1) + wx[i] * wy[i] * field_at(x + 1, y + 1);
              arc_best = std::max(arc_best, v);
            }
            sum += arc_best;
          }
          double fit = sum / n;
          double size_of_correction = std::abs(a * match_rot_step_) / match_max_rot_ +
                                      std::hypot(tx, ty) * resolution_ / match_max_trans_;
          cands.push_back({a, tx, ty, fit, fit - match_prior_weight_ * size_of_correction});
        }
      }
    }

    // Stage 2, free space: a reading also says nothing is closer than its range anywhere in the cone.
    // Rotating or shifting the scan the wrong way makes cones overlap walls short of their readings;
    // penalise that. Only near-best candidates can be affected, so only they are evaluated.
    auto sample = [&](double x, double y) -> float {
      double fx = (x - origin_x_) / resolution_ - win_x0 - 0.5;
      double fy = (y - origin_y_) / resolution_ - win_y0 - 0.5;
      int ix = static_cast<int>(std::floor(fx)), iy = static_cast<int>(std::floor(fy));
      float ax = static_cast<float>(fx - ix), ay = static_cast<float>(fy - iy);
      return (1 - ax) * (1 - ay) * field_at(ix, iy) + ax * (1 - ay) * field_at(ix + 1, iy) +
             (1 - ax) * ay * field_at(ix, iy + 1) + ax * ay * field_at(ix + 1, iy + 1);
    };
    auto violation = [&](const Candidate& cand) {
      double c = std::cos(cand.a * match_rot_step_), s = std::sin(cand.a * match_rot_step_);
      std::vector<float> worst(readings, 0.0f);
      for (size_t i = 0; i < inner.size(); i++) {
        double rx = px + c * (inner[i].first - px) - s * (inner[i].second - py) + cand.tx * resolution_;
        double ry = py + s * (inner[i].first - px) + c * (inner[i].second - py) + cand.ty * resolution_;
        float& w = worst[inner_reading[i]];
        w = std::max(w, sample(rx, ry));
      }
      double total = 0.0;
      for (float w : worst) total += w;
      return total / n;
    };
    double arc_best = 0.0;
    for (const auto& cand : cands) arc_best = std::max(arc_best, cand.fit);
    const double stage2_band = 0.05;   // candidates further below the best arc fit can't win
    for (auto& cand : cands) {
      bool is_zero = cand.a == 0 && cand.tx == 0 && cand.ty == 0;
      if (cand.fit < arc_best - stage2_band && !is_zero) {
        cand.fit = -1.0;
        cand.value = -1e9;
        continue;
      }
      double penalty = match_free_weight_ * violation(cand);
      cand.fit -= penalty;
      cand.value -= penalty;
    }

    const Candidate* best = &cands.front();
    const Candidate* zero = nullptr;
    for (const auto& cand : cands) {
      if (cand.value > best->value) best = &cand;
      if (cand.a == 0 && cand.tx == 0 && cand.ty == 0) zero = &cand;
    }

    // Ambiguity: how far from the winner do near-best fits reach? Raw fit, not value, so the prior
    // can't hide a tie (e.g. sliding along a corridor). Also the best fit clearly away from that region.
    double best_fit = 0.0;
    for (const auto& cand : cands) best_fit = std::max(best_fit, cand.fit);
    double rot_reach = 0.0, trans_reach = 0.0, runner_up = -1.0;
    for (const auto& cand : cands) {
      double rot_off = std::abs(cand.a - best->a) * match_rot_step_;
      double trans_off = std::hypot(cand.tx - best->tx, cand.ty - best->ty) * resolution_;
      if (cand.fit >= best_fit - match_min_distinct_) {
        rot_reach = std::max(rot_reach, rot_off);
        trans_reach = std::max(trans_reach, trans_off);
      }
      if (rot_off > match_ambiguity_rot_ || trans_off > match_ambiguity_trans_) runner_up = std::max(runner_up, cand.fit);
    }
    r.peak_rot = rot_reach;
    r.peak_trans = trans_reach;

    r.dyaw = best->a * match_rot_step_;
    r.dx = best->tx * resolution_;
    r.dy = best->ty * resolution_;
    r.score = best->fit;
    r.zero_score = zero->fit;
    r.margin = best->fit - runner_up;
    r.at_edge = std::abs(best->a) == n_rot || std::abs(best->tx) == n_trans || std::abs(best->ty) == n_trans;

    // Share of readings whose arc touches (or nearly touches) a wall at the best pose
    double c = std::cos(r.dyaw), s = std::sin(r.dyaw);
    size_t on_wall = 0;
    for (size_t j = 0; j < readings; j++) {
      for (int k = 0; k < K; k++) {
        const auto& [x, y] = pts[j * K + k];
        double rx = px + c * (x - px) - s * (y - py) + r.dx;
        double ry = py + s * (x - px) + c * (y - py) + r.dy;
        int fx = static_cast<int>(std::floor((rx - origin_x_) / resolution_)) - win_x0;
        int fy = static_cast<int>(std::floor((ry - origin_y_) / resolution_)) - win_y0;
        if (fx >= 0 && fx < size && fy >= 0 && fy < size && field[fy * size + fx] > 0.5f) {
          on_wall++;
          break;
        }
      }
    }
    r.fraction = static_cast<double>(on_wall) / readings;

    // Trust checks
    if (r.at_edge) r.reject = "best fit at edge of search window";
    else if (r.fraction < match_min_fraction_) r.reject = "too few hits on mapped walls";
    else if (r.score - r.zero_score < match_min_gain_ && (r.dx != 0.0 || r.dy != 0.0 || r.dyaw != 0.0))
      r.reject = "not clearly better than no correction";
    else if (r.peak_rot > match_ambiguity_rot_ || r.peak_trans > match_ambiguity_trans_)
      r.reject = "ambiguous: near-best fits spread too far";
    r.accepted = r.reject.empty();

    r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
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
    grid_msg.header.frame_id = "map";

    // Repurposed: when the last spin scan was inserted, so the navigator can wait for it
    grid_msg.info.map_load_time = last_scan_insert_time_;
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

  // map→odom: SLAM's correction on top of the EKF pose. Identity until scan matching updates it.
  void publish_map_to_odom()
  {
    geometry_msgs::msg::TransformStamped tf;
    // Stamped slightly ahead so lookups at the newest EKF/sensor times never need to extrapolate
    tf.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.translation.x = map_odom_x_;
    tf.transform.translation.y = map_odom_y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, map_odom_yaw_);
    tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf);
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

  // map→odom correction
  double map_odom_x_ = 0.0, map_odom_y_ = 0.0, map_odom_yaw_ = 0.0;

  // Scan matching
  double match_max_rot_, match_rot_step_, match_max_trans_, match_sigma_;
  double match_occ_threshold_, match_min_fraction_, match_min_gain_;
  double match_prior_weight_, match_ambiguity_rot_, match_ambiguity_trans_, match_min_distinct_;
  int match_min_points_;
  int match_arc_samples_;
  double match_free_weight_, match_free_margin_;
  double default_half_fov_;
  bool match_apply_;
  double match_widen_rot_, match_widen_trans_, match_widen_max_rot_, match_widen_max_trans_;
  int spins_since_correction_ = 0;

  // Recording
  std::string record_dir_;
  int record_count_ = 0;
  rclcpp::Time spin_end_time_{0, 0, RCL_ROS_TIME};

  // Spin scan buffering
  bool scanning_ = false;
  std::vector<Ray> scan_buffer_;
  rclcpp::Time last_scan_insert_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> range_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spin_scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr map_odom_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SparseSlam>());
  rclcpp::shutdown();
  return 0;
}