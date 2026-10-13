// astar.hpp — header-only A* grid path planner.
//
// ROS-free and dependency-light, so it can be reused by any node. Works on an
// 8-connected 2D occupancy grid (row-major). With the octile heuristic it is
// admissible and consistent, so it is guaranteed to return a shortest path
// when one exists, and an empty path otherwise.
//
// Occupancy convention (same as the validated unit test):
//   0          -> traversable (free)
//   anything else -> obstacle (occupied *or* unknown)
//
// To use with a nav_msgs/OccupancyGrid (e.g. slam_toolbox's /map), the caller
// converts cells first. A sensible mapping for exploration is:
//   value >= occupied_threshold -> obstacle,  else -> free (unknown included).
// The caller decides whether -1 (unknown) is traversable; for frontier
// exploration it is usually treated as free so paths may extend into it.

#ifndef AUTO_EXPLORE_ASTAR_HPP
#define AUTO_EXPLORE_ASTAR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace astar {

struct Cell {
  int x, y;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
};

// ---------------------------------------------------------------------------
// Plan a path from start to goal on a row-major grid of width w / height h.
//   grid : occupancy buffer, 0 = free, anything else = obstacle.
// Returns the shortest path inclusive of both endpoints, or an empty vector
// when no path exists (out of bounds, occupied endpoints, or fully blocked).
// Diagonal moves cost sqrt(2); corner-cutting between two diagonal obstacles
// is prevented so the path never clips a corner.
// ---------------------------------------------------------------------------
inline std::vector<Cell> plan(const std::vector<int8_t>& grid, int w, int h,
                              Cell start, Cell goal) {
  const auto idx = [&](int x, int y) { return y * w + x; };
  const auto in_bounds = [&](int x, int y) {
    return x >= 0 && x < w && y >= 0 && y < h;
  };

  if (!in_bounds(start.x, start.y) || !in_bounds(goal.x, goal.y) ||
      grid[idx(start.x, start.y)] != 0 || grid[idx(goal.x, goal.y)] != 0)
    return {};

  const double SQRT2 = std::sqrt(2.0);
  // Octile distance: admissible & consistent for 8-connected grids with
  // orthogonal cost 1 and diagonal cost sqrt(2).
  const auto octile = [&](int dx, int dy) {
    const int adx = std::abs(dx), ady = std::abs(dy);
    return std::max(adx, ady) + (SQRT2 - 1.0) * std::min(adx, ady);
  };

  std::vector<double> g(w * h, std::numeric_limits<double>::infinity());
  std::vector<int> came(w * h, -1);
  std::vector<bool> closed(w * h, false);

  using Node = std::pair<double, int>;  // (f = g + h, cell index)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  const int s = idx(start.x, start.y);
  const int t = idx(goal.x, goal.y);
  g[s] = 0.0;
  open.push({octile(start.x - goal.x, start.y - goal.y), s});

  static const int DX[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int DY[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();
    const int c = cur.second;
    if (closed[c]) continue;  // stale entry in the priority queue
    closed[c] = true;
    if (c == t) break;

    const int cx = c % w, cy = c / w;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + DX[k], ny = cy + DY[k];
      if (!in_bounds(nx, ny)) continue;
      const int n = idx(nx, ny);
      if (closed[n] || grid[n] != 0) continue;

      const bool diag = (DX[k] != 0 && DY[k] != 0);
      if (diag && (grid[idx(cx + DX[k], cy)] != 0 ||
                   grid[idx(cx, cy + DY[k])] != 0))
        continue;  // do not cut a corner between two obstacles

      const double step = diag ? SQRT2 : 1.0;
      const double ng = g[c] + step;
      if (ng < g[n]) {
        g[n] = ng;
        came[n] = c;
        open.push({ng + octile(nx - goal.x, ny - goal.y), n});
      }
    }
  }

  if (std::isinf(g[t])) return {};  // unreachable

  std::vector<Cell> path;
  for (int c = t; c != -1; c = came[c]) path.push_back({c % w, c / w});
  std::reverse(path.begin(), path.end());
  return path;
}

// Total path cost using the same move costs as plan() (sqrt(2) diagonal).
inline double pathCost(const std::vector<Cell>& path) {
  if (path.size() < 2) return 0.0;
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const int dx = std::abs(path[i].x - path[i - 1].x);
    const int dy = std::abs(path[i].y - path[i - 1].y);
    cost += (dx && dy) ? std::sqrt(2.0) : 1.0;
  }
  return cost;
}

}  // namespace astar

#endif  // AUTO_EXPLORE_ASTAR_HPP
