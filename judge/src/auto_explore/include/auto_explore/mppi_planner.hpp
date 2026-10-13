// mppi_planner.hpp — lightweight Model Predictive Path Integral (MPPI)
// local planner for the auto_explore node.
//
// Works in the BODY frame of the current control cycle:
//   x  = forward (m)
//   y  = left (m)
//   yaw = 0 at t0
// The global goal is transformed into this body frame once per cycle, and a
// local obstacle distance field is built from the latest filtered 2D scan
// (also body frame). Rollouts are integrated with a unicycle model, so the
// planner directly outputs body-frame forward speed v and yaw rate w.
//
// The implementation is deliberately small and dependency-free (C++11):
//   - K random constant-control rollouts over a horizon of H steps
//   - sample around the current commanded (v, w)
//   - score obstacle proximity, goal progress, yaw rate, and speed tracking
//   - select the lowest-cost feasible rollout and return its first control
//
// Typical use (20 Hz control loop, 300 samples, 20 steps @ dt=0.1 -> 2 s):
//   mppi::Planner p;
//   mppi::Params prm; ... p.configure(prm);
//   p.setGoal(tx_body, ty_body);
//   p.setObstaclePoints(scan_ranges, scan_angle_min, scan_angle_inc, ...);
//   mppi::Control c = p.plan(current_v, current_w);
//   cmd.linear.x  = c.v;
//   cmd.angular.z = c.w;

#ifndef AUTO_EXPLORE_MPPI_PLANNER_HPP
#define AUTO_EXPLORE_MPPI_PLANNER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace mppi {

// 期望路径点（机体坐标系），由调用方通过 Planner::setPath() 注入。
struct PathPoint {
  double x = 0.0;
  double y = 0.0;
};

struct Params {
  // Sampling budget.
  int num_samples = 1500;    // K candidate routes per planning cycle
  int horizon = 20;          // H steps
  double dt = 0.10;          // rollout step (s) -> 2.0 s preview

  // Control limits.
  double max_v = 5.0;        // max forward speed (m/s)
  double min_v = 0.0;        // min forward speed (m/s, 0 = allowed to hover)
  double max_w = 1.2;        // max yaw rate magnitude (rad/s)

  // Sampling noise (standard deviation of the control perturbation).
  double sigma_v = 0.8;
  double sigma_w = 0.6;

  // MPPI temperature. Smaller = greedier, larger = more exploratory.
  double lambda = 0.15;

  // Desired cruise speed: the planner is pulled toward this speed when safe.
  double v_des = 3.0;

  // Cost weights.
  double goal_weight = 1.2;          // terminal distance-to-goal
  double obstacle_weight = 40.0;     // running obstacle proximity
  double yaw_weight = 0.05;          // running yaw-rate penalty
  double speed_weight = 0.4;         // running (v - v_des)^2 penalty

  // DWB-style trajectory critics (dominant path-alignment + path-distance).
  // path_align_weight：每步 rollout 末端与"局部期望路径方向"的对齐代价——
  //                     rollout 末态切向与目标方向的夹角越大代价越高（cos 加权）。
  // path_dist_weight : rollout 末端到"局部期望路径"的有符号横向偏移——
  //                     沿 path 右手坐标系，路径左侧为正、右侧为负，偏离路径的代价越高。
  // 两者共同把 rollout 推向"快到目标 + 沿路"的双主导，与 ROS DWB 的 PathAlign/PathDist 一致。
  double path_align_weight = 3.0;     // 与期望切向对齐（越大越压弯）
  double path_dist_weight  = 2.5;     // 与期望横向偏差（越大越收紧）
  // 期望路径长度：用 setPath() 注入；若未注入则按直线到 goal 评估（保持向后兼容）。
  std::vector<PathPoint> path_;

  // Obstacle field geometry.
  double field_radius = 18.0;        // half-size of the local distance field (m)
  double field_res = 0.15;           // cell size (m)
  double safe_radius = 2.5;          // distance below which obstacle cost grows (m)
  // 自身物理半径（m）：从所有"距离障碍"的实测值中减去，让 MPPI 把无人机当成
  // 一个真实大小的圆盘而不是质点，从而撞墙/卡墙更早出现代价上升。
  // depth_obstacle_cost 也按 (dd - robot_radius) 重新分段。
  // 默认 0.5 m 适配 typhoon_h480；换机型改 yaml mppi_robot_radius 即可。
  double robot_radius = 0.5;

  // Independent compact depth obstacle grid. It is deliberately smaller than
  // the laser field so depth updates remain bounded and cheap per cycle.
  double depth_grid_radius = 8.0;
  double depth_grid_res = 0.25;
  double depth_cost_cutoff = 4.0;       // depth obstacle query radius (m)
  double depth_obstacle_weight = 40.0;  // quadratic weight when depth < depth_soft_start
  double depth_soft_start = 2.5;        // soft-zone lower bound (m): depth in
                                        // [soft_start, cutoff] uses a small linear
                                        // penalty (allows passing narrow gaps);
                                        // depth < soft_start keeps the original
                                        // quadratic penalty using depth_obstacle_weight.
  double depth_soft_weight = 3.0;       // max soft-zone penalty at depth ==
                                        // depth_soft_start; linear taper to 0 at
                                        // depth_cost_cutoff.

  // Rollout stops early if the state leaves the field.
  bool stop_outside_field = true;
};

struct State {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

struct Control {
  double v = 0.0, w = 0.0;
};

class Planner {
 public:
  Planner() : rng_(20260902u) {}

  void configure(const Params& p) {
    p_ = p;
    if (p_.field_res <= 0.0 || p_.field_radius <= 0.0 || p_.dt <= 0.0 ||
        p_.lambda <= 0.0 || p_.num_samples <= 0 || p_.horizon <= 0) {
      field_.clear();
      field_valid_ = false;
      return;
    }
    const int n = static_cast<int>(std::ceil(2.0 * p_.field_radius / p_.field_res));
    field_w_ = n;
    field_h_ = n;
    field_.assign(static_cast<size_t>(field_w_) * field_h_, 0.0f);
    const int depth_n = (p_.depth_grid_res > 0.0 && p_.depth_grid_radius > 0.0)
        ? static_cast<int>(std::ceil(2.0 * p_.depth_grid_radius / p_.depth_grid_res)) : 0;
    depth_grid_w_ = depth_n;
    depth_grid_h_ = depth_n;
    depth_grid_.assign(static_cast<size_t>(depth_grid_w_) * depth_grid_h_, 0.0f);
    depth_observed_.assign(static_cast<size_t>(depth_grid_w_) * depth_grid_h_, 0);
    candidate_costs_.assign(static_cast<size_t>(p_.num_samples),
                            std::numeric_limits<double>::infinity());
    candidate_route_.reserve(static_cast<size_t>(p_.horizon) + 1);
    best_route_.reserve(static_cast<size_t>(p_.horizon) + 1);
    path_.clear();   // path_ 由 setPath() 显式注入；configure 不保留旧路径
    depth_grid_valid_ = false;
    last_depth_update_sec_ = 0.0;
    field_valid_ = false;
    valid_hits_ = 0;
    scan_valid_ = false;
    evaluated_samples_ = 0;
    best_route_index_ = -1;
    best_cost_ = std::numeric_limits<double>::infinity();
    last_plan_duration_ms_ = 0.0;
    v_prev_ = 0.0;
    w_prev_ = 0.0;
  }

  // Goal in the CURRENT body frame (x forward, y left).
  void setGoal(double gx, double gy) {
    goal_x_ = gx;
    goal_y_ = gy;
  }

  // Optional DWB-style path for PathAlign/PathDist critics. Each point is in
  // the body frame (x forward, y left). The path is followed sequentially:
  // for each rollout step the critic projects the state onto the closest
  // segment of this path. Empty path keeps backward-compatible behavior
  // (goal-only terminal cost).
  void setPath(const std::vector<std::pair<double, double>>& body_path) {
    path_.clear();
    path_.reserve(body_path.size());
    for (const auto& p : body_path) path_.push_back({p.first, p.second});
  }
  void clearPath() { path_.clear(); }

  // Build the body-frame obstacle distance field from a laser scan.
  // `ranges` are already filtered (self-hits removed). Points are assumed to
  // be in the scan frame == body frame (x forward, y left).
  void setScan(const std::vector<float>& ranges, double angle_min,
               double angle_inc, double range_max, double range_min = 0.05,
               bool scan_usable = true) {
    if (field_.empty() || ranges.empty() || !scan_usable) {
      scan_valid_ = false;
      field_valid_ = false;
      return;
    }
    const float kInf = 1.0e9f;
    std::fill(field_.begin(), field_.end(), kInf);
    int valid_hits = 0;
    scan_valid_ = false;
    field_valid_ = false;
    if (!(std::isfinite(range_min) && std::isfinite(range_max) &&
          range_max > range_min && range_min >= 0.0)) {
      return;
    }

    const int half_x = field_w_ / 2;
    const int half_y = field_h_ / 2;
    const double inv_res = 1.0 / p_.field_res;
    const double R = p_.field_radius;
    const int n = field_w_ * field_h_;

    for (size_t i = 0; i < ranges.size(); ++i) {
      const float r = ranges[i];
      if (!(r >= static_cast<float>(range_min) &&
            r < static_cast<float>(range_max) - 0.05f) || !std::isfinite(r))
        continue;
      ++valid_hits;
      const double a = angle_min + i * angle_inc;
      const double px = r * std::cos(a);
      const double py = r * std::sin(a);
      if (std::fabs(px) > R || std::fabs(py) > R) continue;
      const int cx = half_x + static_cast<int>(std::floor(px * inv_res));
      const int cy = half_y + static_cast<int>(std::floor(py * inv_res));
      if (cx >= 0 && cx < field_w_ && cy >= 0 && cy < field_h_)
        field_[cy * field_w_ + cx] = 0.0f;  // obstacle seed
    }

    // Two-pass 3x3 chamfer distance transform (approximate Euclidean).
    // Forward pass (top-left -> bottom-right).
    for (int y = 0; y < field_h_; ++y) {
      for (int x = 0; x < field_w_; ++x) {
        const int idx = y * field_w_ + x;
        float v = field_[idx];
        if (y > 0) {
          v = std::min(v, field_[(y - 1) * field_w_ + x] + 1.0f);
          if (x > 0) v = std::min(v, field_[(y - 1) * field_w_ + x - 1] + 1.41421f);
          if (x + 1 < field_w_) v = std::min(v, field_[(y - 1) * field_w_ + x + 1] + 1.41421f);
        }
        if (x > 0) v = std::min(v, field_[y * field_w_ + x - 1] + 1.0f);
        field_[idx] = v;
      }
    }
    // Backward pass (bottom-right -> top-left).
    for (int y = field_h_ - 1; y >= 0; --y) {
      for (int x = field_w_ - 1; x >= 0; --x) {
        const int idx = y * field_w_ + x;
        float v = field_[idx];
        if (y + 1 < field_h_) {
          v = std::min(v, field_[(y + 1) * field_w_ + x] + 1.0f);
          if (x > 0) v = std::min(v, field_[(y + 1) * field_w_ + x - 1] + 1.41421f);
          if (x + 1 < field_w_) v = std::min(v, field_[(y + 1) * field_w_ + x + 1] + 1.41421f);
        }
        if (x + 1 < field_w_) v = std::min(v, field_[y * field_w_ + x + 1] + 1.0f);
        field_[idx] = v;
      }
    }
    // Convert cell distance to meters.
    for (int i = 0; i < n; ++i) field_[i] *= static_cast<float>(p_.field_res);
    field_valid_ = valid_hits > 0;
    scan_valid_ = valid_hits >= 3;
    valid_hits_ = valid_hits;
  }

  // Score K constant-control rollouts and return the first control of the
  // minimum-total-cost route. Replanning each cycle makes this receding-horizon
  // route selection; callers may hold the result between planner ticks.
  Control plan(double v_now, double w_now) {
    const std::chrono::steady_clock::time_point started =
        std::chrono::steady_clock::now();
    Control out;
    out.v = clamp(v_now, p_.min_v, p_.max_v);
    out.w = clamp(w_now, -p_.max_w, p_.max_w);
    evaluated_samples_ = 0;
    best_route_index_ = -1;
    best_cost_ = std::numeric_limits<double>::infinity();
    best_route_.clear();
    if (!field_valid_ || !scan_valid_) {
      v_prev_ = 0.0;
      w_prev_ = 0.0;
      last_plan_duration_ms_ = elapsedMs(started);
      return Control();
    }

    const int K = p_.num_samples;
    const int H = p_.horizon;
    const double dt = p_.dt;
    std::normal_distribution<double> nv(0.0, p_.sigma_v);
    std::normal_distribution<double> nw(0.0, p_.sigma_w);

    const double v0 = clamp(v_now, p_.min_v, p_.max_v);
    const double w0 = clamp(w_now, -p_.max_w, p_.max_w);
    int best_index = -1;
    best_route_.clear();

    for (int k = 0; k < K; ++k) {
      const double cv = clamp(v0 + nv(rng_), p_.min_v, p_.max_v);
      const double cw = clamp(w0 + nw(rng_), -p_.max_w, p_.max_w);
      State s;
      candidate_route_.clear();
      candidate_route_.push_back(s);
      double total_cost = 0.0;
      bool left_local_field = false;

      for (int t = 0; t < H; ++t) {
        total_cost += runningCost(s, cv, cw);
        s = integrate(s, cv, cw, dt);
        candidate_route_.push_back(s);
        if (p_.stop_outside_field && !insideField(s)) {
          total_cost += 500.0;
          left_local_field = true;
          break;
        }
      }
      if (!left_local_field) total_cost += terminalCost(s);
      candidate_costs_[static_cast<size_t>(k)] = total_cost;
      ++evaluated_samples_;

      if (total_cost < best_cost_) {
        best_cost_ = total_cost;
        best_index = k;
        out.v = cv;
        out.w = cw;
        best_route_ = candidate_route_;
      }
    }

    best_route_index_ = best_index;
    v_prev_ = out.v;
    w_prev_ = out.w;
    last_plan_duration_ms_ = elapsedMs(started);
    return out;
  }

  bool valid() const { return scan_valid_; }
  // plan() also rejects scans with fewer than three valid returns.
  bool canPlan() const { return field_valid_ && scan_valid_; }
  int validHits() const { return valid_hits_; }
  int evaluatedSamples() const { return evaluated_samples_; }
  int bestRouteIndex() const { return best_route_index_; }
  double bestCost() const { return best_cost_; }
  double lastPlanDurationMs() const { return last_plan_duration_ms_; }
  const std::vector<double>& candidateCosts() const { return candidate_costs_; }
  const std::vector<State>& bestRoute() const { return best_route_; }
  void resetWarmStart(double v, double w) {
    v_prev_ = clamp(v, p_.min_v, p_.max_v);
    w_prev_ = clamp(w, -p_.max_w, p_.max_w);
  }

  // Invalidate the cached depth obstacle grid when the depth source becomes
  // stale (no fresh depth callback within `age_s`). setDepthPoints will set a
  // fresh grid on the next callback; this guard prevents the planner from
  // reacting to a depth field that may no longer reflect the scene.
  // Does NOT touch setDepthPoints' existing logic.
  void expireIfStale(double age_s) {
    if (!depth_grid_valid_) return;
    const double now = nowSec();
    if (last_depth_update_sec_ <= 0.0 ||
        now - last_depth_update_sec_ > age_s) {
      depth_grid_valid_ = false;
    }
  }

  double obstacleDistance(double x, double y) const {
    if (!field_valid_) return std::numeric_limits<double>::max();
    const int cx = field_w_ / 2 + static_cast<int>(std::floor(x / p_.field_res));
    const int cy = field_h_ / 2 + static_cast<int>(std::floor(y / p_.field_res));
    if (cx < 0 || cx >= field_w_ || cy < 0 || cy >= field_h_)
      return std::numeric_limits<double>::max();
    return field_[cy * field_w_ + cx];
  }

  void setDepthPoints(const std::vector<float>& points_xy) {
    if (depth_grid_.empty()) { depth_grid_valid_ = false; return; }
    const float kInf = 1.0e9f;
    std::fill(depth_grid_.begin(), depth_grid_.end(), kInf);
    std::fill(depth_observed_.begin(), depth_observed_.end(), 0);
    const int half = depth_grid_w_ / 2;
    const double inv_res = 1.0 / p_.depth_grid_res;
    int hits = 0;
    for (size_t i = 0; i + 1 < points_xy.size(); i += 2) {
      const double x = points_xy[i], y = points_xy[i + 1];
      if (!std::isfinite(x) || !std::isfinite(y) ||
          std::fabs(x) > p_.depth_grid_radius || std::fabs(y) > p_.depth_grid_radius)
        continue;
      const int cx = half + static_cast<int>(std::floor(x * inv_res));
      const int cy = half + static_cast<int>(std::floor(y * inv_res));
      if (cx >= 0 && cx < depth_grid_w_ && cy >= 0 && cy < depth_grid_h_) {
        depth_grid_[cy * depth_grid_w_ + cx] = 0.0f;
        depth_observed_[cy * depth_grid_w_ + cx] = 1;
        ++hits;
      }
    }
    if (hits == 0) { depth_grid_valid_ = false; return; }
    for (int y = 0; y < depth_grid_h_; ++y)
      for (int x = 0; x < depth_grid_w_; ++x) {
        const int idx = y * depth_grid_w_ + x;
        if (!depth_observed_[idx]) continue;
        float v = depth_grid_[idx];
        if (y > 0 && depth_observed_[(y - 1) * depth_grid_w_ + x])
          v = std::min(v, depth_grid_[(y - 1) * depth_grid_w_ + x] + 1.0f);
        if (x > 0 && depth_observed_[y * depth_grid_w_ + x - 1])
          v = std::min(v, depth_grid_[y * depth_grid_w_ + x - 1] + 1.0f);
        depth_grid_[idx] = v;
      }
    for (int y = depth_grid_h_ - 1; y >= 0; --y)
      for (int x = depth_grid_w_ - 1; x >= 0; --x) {
        const int idx = y * depth_grid_w_ + x;
        if (!depth_observed_[idx]) continue;
        float v = depth_grid_[idx];
        if (y + 1 < depth_grid_h_ && depth_observed_[(y + 1) * depth_grid_w_ + x])
          v = std::min(v, depth_grid_[(y + 1) * depth_grid_w_ + x] + 1.0f);
        if (x + 1 < depth_grid_w_ && depth_observed_[y * depth_grid_w_ + x + 1])
          v = std::min(v, depth_grid_[y * depth_grid_w_ + x + 1] + 1.0f);
        depth_grid_[idx] = v;
      }
    for (size_t i = 0; i < depth_grid_.size(); ++i)
      if (depth_observed_[i]) depth_grid_[i] *= static_cast<float>(p_.depth_grid_res);
    depth_grid_valid_ = true;
    last_depth_update_sec_ = nowSec();
  }

 private:
  static double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
  }

  static double nowSec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
               steady_clock::now().time_since_epoch()).count();
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point& started) {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  }

  State integrate(const State& s, double v, double w, double dt) const {
    State n;
    n.x = s.x + v * std::cos(s.yaw) * dt;
    n.y = s.y + v * std::sin(s.yaw) * dt;
    n.yaw = s.yaw + w * dt;
    return n;
  }

  bool insideField(const State& s) const {
    return std::fabs(s.x) < p_.field_radius && std::fabs(s.y) < p_.field_radius;
  }

  // MPPI depth cost (units: obstacle-weight * 1, weight * normalized-distance).
  //   dd >= depth_cost_cutoff        -> 0 (free space)
  //   dd in [depth_soft_start, ...)  -> linear soft penalty, scales with
  //                                     (depth_cost_cutoff - dd) / span, weight
  //                                     depth_soft_weight. Monotonic from 0 at
  //                                     cutoff to depth_soft_weight at soft_start.
  //   dd <  depth_soft_start         -> quadratic penalty using the original
  //                                     (depth_cost_cutoff - dd)/depth_cost_cutoff
  //                                     formula and depth_obstacle_weight. The
  //                                     existing depth_cost_cutoff / depth_obstacle_
  //                                     weight semantics are preserved.
  double depthObstacleCost(double dd) const {
    // 自身半径：从深度距离 dd 中减去后得到净空再分段计算代价。
    // robot_radius=0.5 时，距离障碍 2.5 m 平整即可命中硬区阈值（< 2.5 m），
    // 体现"无人机更早开始减速/绕行"的物理行为。
    const double d = dd - p_.robot_radius;
    if (d >= p_.depth_cost_cutoff) return 0.0;
    if (d < p_.depth_soft_start) {
      const double q = (p_.depth_cost_cutoff - d) / p_.depth_cost_cutoff;
      return p_.depth_obstacle_weight * q * q;
    }
    const double span = p_.depth_cost_cutoff - p_.depth_soft_start;
    if (span <= 0.0) return 0.0;
    const double r = (p_.depth_cost_cutoff - d) / span;   // 0..1
    return p_.depth_soft_weight * r;
  }

  // 把 rollout 末态投影到 path_ 上最近的有符号横向偏移 + 期望切向；
// path_ 为空时返回直线到 (goal_x_, goal_y_) 的退化行为（保持向后兼容）。
  // 返回：{signed_lateral, tangent_x, tangent_y}；lateral 单位为 m，>0 表示在路径左侧。
  struct PathProjection {
    double lateral = 0.0;
    double tangent_x = 1.0;
    double tangent_y = 0.0;
    double s_along = 0.0;
    bool valid = false;
  };
  PathProjection projectOntoPath(const State& s) const
  {
    PathProjection out;
    if (path_.size() < 2) {
      // 退化：把目标点直接当路径终点，期望切向为 (goal - state) 方向。
      const double gdx = goal_x_ - s.x;
      const double gdy = goal_y_ - s.y;
      const double len = std::hypot(gdx, gdy);
      if (len > 1e-6) {
        out.tangent_x = gdx / len;
        out.tangent_y = gdy / len;
        out.valid = true;
      }
      out.lateral = 0.0;
      return out;
    }
    double best_d2 = std::numeric_limits<double>::infinity();
    size_t best_seg = 0;
    double best_t = 0.0;
    for (size_t i = 0; i + 1 < path_.size(); ++i)
    {
      const double ax = path_[i].x,     ay = path_[i].y;
      const double bx = path_[i + 1].x, by = path_[i + 1].y;
      const double dx = bx - ax, dy = by - ay;
      const double seg_len2 = dx * dx + dy * dy;
      if (seg_len2 < 1e-9) continue;
      double t = ((s.x - ax) * dx + (s.y - ay) * dy) / seg_len2;
      t = std::max(0.0, std::min(1.0, t));
      const double px = ax + t * dx;
      const double py = ay + t * dy;
      const double ddx = s.x - px, ddy = s.y - py;
      const double d2 = ddx * ddx + ddy * ddy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_seg = i;
        best_t = t;
      }
    }
    const double ax = path_[best_seg].x,     ay = path_[best_seg].y;
    const double bx = path_[best_seg + 1].x, by = path_[best_seg + 1].y;
    const double dx = bx - ax, dy = by - ay;
    const double seg_len = std::hypot(dx, dy);
    if (seg_len > 1e-6) {
      out.tangent_x = dx / seg_len;
      out.tangent_y = dy / seg_len;
    }
    // 路径左侧 = 切向按 CCW 旋转 90° = (-ty, tx)；rollout 在该方向为正则 lateral > 0。
    const double px = ax + best_t * dx;
    const double py = ay + best_t * dy;
    const double offset_x = s.x - px;
    const double offset_y = s.y - py;
    out.lateral = -out.tangent_y * offset_x + out.tangent_x * offset_y;
    out.valid = true;
    return out;
  }

  // DWB 风格：每步 rollout 末态与期望路径的横向偏差 + 切向对齐代价。
  // 末端对齐：rollout.yaw 与 path tangent 越一致，cos 越大、惩罚越小。
  // 横向偏差：abs(lateral) 越大，代价越大。
  void addPathCritics(double& cost, const State& s) const
  {
    if (path_.empty() && p_.path_align_weight <= 0.0 && p_.path_dist_weight <= 0.0) return;
    if (p_.path_align_weight <= 0.0 && p_.path_dist_weight <= 0.0) return;
    const PathProjection proj = projectOntoPath(s);
    if (!proj.valid) return;
    if (p_.path_dist_weight > 0.0) {
      cost += p_.path_dist_weight * std::fabs(proj.lateral);
    }
    if (p_.path_align_weight > 0.0) {
      // cos(rollout.yaw - path_tangent)：对齐时 ≈ 1；偏移越大代价越高。
      // 引入 (1 - cos) 比直接 |sin| 数值更稳（避免分段）。
      const double sin_yaw = std::sin(s.yaw), cos_yaw = std::cos(s.yaw);
      const double cos_align = sin_yaw * proj.tangent_y + cos_yaw * proj.tangent_x;
      const double misalign = 1.0 - std::max(-1.0, std::min(1.0, cos_align));
      cost += p_.path_align_weight * misalign;
    }
  }

  double runningCost(const State& s, double v, double w) const {
    double c = 0.0;
    // 自身半径：从所有"距离障碍"的实测值中减去，让 MPPI 把无人机当成真实大小的圆盘。
    // 例如 robot_radius=0.5 → 距离障碍 0.6 m 时净空只剩 0.1 m → 已逼近代价上升。
    const double d_raw = obstacleDistance(s.x, s.y);
    const double d = std::max(0.0, d_raw - p_.robot_radius);
    // 动态安全半径：速度越快，刹车/避障所需的提前量越大。
    // v=0 时用静态 safe_radius；v=5m/s 时安全半径至少 4m。
    const double dyn_safe = std::max(p_.safe_radius, std::fabs(v) * 0.8);
    if (d < dyn_safe) {
      const double p = (dyn_safe - d) / dyn_safe;  // 0..1
      c += p_.obstacle_weight * p * p;              // quadratic
    }
    if (depth_grid_valid_) {
      const int cx = depth_grid_w_ / 2 + static_cast<int>(std::floor(s.x / p_.depth_grid_res));
      const int cy = depth_grid_h_ / 2 + static_cast<int>(std::floor(s.y / p_.depth_grid_res));
      if (cx >= 0 && cx < depth_grid_w_ && cy >= 0 && cy < depth_grid_h_) {
        const int idx = cy * depth_grid_w_ + cx;
        if (depth_observed_[idx]) {
          c += depthObstacleCost(static_cast<double>(depth_grid_[idx]));
        }
      }
    }
    c += p_.yaw_weight * w * w;
    const double dv = v - p_.v_des;
    c += p_.speed_weight * dv * dv;
    addPathCritics(c, s);
    return c;
  }

  double terminalCost(const State& s) const {
    const double dx = s.x - goal_x_;
    const double dy = s.y - goal_y_;
    double c = p_.goal_weight * std::hypot(dx, dy);
    if (depth_grid_valid_) {
      const int cx = depth_grid_w_ / 2 + static_cast<int>(std::floor(s.x / p_.depth_grid_res));
      const int cy = depth_grid_h_ / 2 + static_cast<int>(std::floor(s.y / p_.depth_grid_res));
      if (cx >= 0 && cx < depth_grid_w_ && cy >= 0 && cy < depth_grid_h_) {
        const int idx = cy * depth_grid_w_ + cx;
        if (depth_observed_[idx]) {
          c += depthObstacleCost(static_cast<double>(depth_grid_[idx]));
        }
      }
    }
    addPathCritics(c, s);
    return c;
  }

  Params p_;
  std::vector<float> field_;
  int field_w_ = 0, field_h_ = 0;
  std::vector<float> depth_grid_;
  std::vector<unsigned char> depth_observed_;
  int depth_grid_w_ = 0, depth_grid_h_ = 0;
  bool depth_grid_valid_ = false;
  bool field_valid_ = false;
  int valid_hits_ = 0;
  bool scan_valid_ = false;
  int evaluated_samples_ = 0;
  int best_route_index_ = -1;
  double best_cost_ = std::numeric_limits<double>::infinity();
  double last_plan_duration_ms_ = 0.0;
  std::vector<double> candidate_costs_;
  std::vector<State> candidate_route_, best_route_;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  std::vector<PathPoint> path_;
  double v_prev_ = 0.0, w_prev_ = 0.0;
  double last_depth_update_sec_ = 0.0;
  std::mt19937 rng_;
};

}  // namespace mppi

#endif  // AUTO_EXPLORE_MPPI_PLANNER_HPP
