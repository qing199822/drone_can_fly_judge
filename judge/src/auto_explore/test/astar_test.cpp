// astar_test.cpp — self-contained A* (8-connected grid) + correctness tests.
//
// No ROS / catkin dependency. Build & run standalone:
//   g++ -std=c++14 -O2 astar_test.cpp -o astar_test && ./astar_test
//
// The A* below is the same core that will later be reused by the exploration
// node as its global planner. It is deterministic: with an admissible &
// consistent (octile) heuristic it is guaranteed to return an optimal path when
// one exists, and an empty path otherwise.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

namespace astar {

struct Cell {
  int x, y;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
};

// ---------------------------------------------------------------------------
// A* on an 8-connected 2D grid.
//   grid : row-major occupancy, 0 = free, anything else = obstacle.
//   w, h : grid width / height in cells.
//   start, goal : cell coordinates.
// Returns the shortest path inclusive of start and goal, or an empty vector
// when no path exists (blocked, out of bounds, or start/goal occupied).
// ---------------------------------------------------------------------------
std::vector<Cell> plan(const std::vector<int8_t>& grid, int w, int h,
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
    if (closed[c]) continue;  // stale entry, skip
    closed[c] = true;
    if (c == t) break;

    const int cx = c % w, cy = c / w;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + DX[k], ny = cy + DY[k];
      if (!in_bounds(nx, ny)) continue;
      const int n = idx(nx, ny);
      if (closed[n] || grid[n] != 0) continue;

      const bool diag = (DX[k] != 0 && DY[k] != 0);
      // Prevent cutting a corner between two diagonal obstacles.
      if (diag && (grid[idx(cx + DX[k], cy)] != 0 ||
                   grid[idx(cx, cy + DY[k])] != 0))
        continue;

      const double step = diag ? SQRT2 : 1.0;
      const double ng = g[c] + step;
      if (ng < g[n]) {
        g[n] = ng;
        came[n] = c;
        open.push({ng + octile(nx - goal.x, ny - goal.y), n});
      }
    }
  }

  if (std::isinf(g[t])) return {};  // no path

  std::vector<Cell> path;
  for (int c = t; c != -1; c = came[c]) path.push_back({c % w, c / w});
  std::reverse(path.begin(), path.end());
  return path;
}

// Total cost of a path using the same move costs as the planner.
double pathCost(const std::vector<Cell>& path) {
  if (path.size() < 2) return 0.0;
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const int dx = std::abs(path[i].x - path[i - 1].x);
    const int dy = std::abs(path[i].y - path[i - 1].y);
    cost += (dx && dy) ? std::sqrt(2.0) : 1.0;
  }
  return cost;
}

// Validate a path: every cell free, every step is 8-adjacent, no corner cuts.
bool pathValid(const std::vector<Cell>& path, const std::vector<int8_t>& grid,
               int w, int h) {
  if (path.empty()) return true;
  const auto idx = [&](int x, int y) { return y * w + x; };
  for (size_t i = 0; i < path.size(); ++i) {
    const int x = path[i].x, y = path[i].y;
    if (x < 0 || x >= w || y < 0 || y >= h || grid[idx(x, y)] != 0) return false;
  }
  for (size_t i = 1; i < path.size(); ++i) {
    const int dx = std::abs(path[i].x - path[i - 1].x);
    const int dy = std::abs(path[i].y - path[i - 1].y);
    if (dx > 1 || dy > 1 || (dx == 0 && dy == 0)) return false;
    if (dx && dy) {  // diagonal: no corner cut
      if (grid[idx(path[i].x, path[i - 1].y)] != 0 ||
          grid[idx(path[i - 1].x, path[i].y)] != 0)
        return false;
    }
  }
  return true;
}

}  // namespace astar

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
static int failures = 0;

static void check(bool ok, const std::string& msg) {
  std::cout << (ok ? "  [PASS] " : "  [FAIL] ") << msg << "\n";
  if (!ok) ++failures;
}

static void printGrid(const std::vector<int8_t>& grid, int w, int h,
                      const std::vector<astar::Cell>& path,
                      astar::Cell start, astar::Cell goal) {
  for (int y = 0; y < h; ++y) {
    std::cout << "    ";
    for (int x = 0; x < w; ++x) {
      astar::Cell c{x, y};
      if (c == start) std::cout << 'S';
      else if (c == goal) std::cout << 'G';
      else if (std::find(path.begin(), path.end(), c) != path.end()) std::cout << '*';
      else if (grid[y * w + x] != 0) std::cout << '#';
      else std::cout << '.';
    }
    std::cout << "\n";
  }
}

int main() {
  using astar::Cell;
  const double SQRT2 = std::sqrt(2.0);

  std::cout << "=== A* correctness tests ===\n\n";

  // --- Test 1: open grid, straight diagonal, optimal cost ----------------
  {
    const int w = 10, h = 10;
    std::vector<int8_t> grid(w * h, 0);
    Cell start{0, 0}, goal{9, 9};
    auto path = astar::plan(grid, w, h, start, goal);
    check(!path.empty(), "open grid: path found");
    check(path.front() == start && path.back() == goal,
          "open grid: start/goal endpoints correct");
    check(astar::pathValid(path, grid, w, h), "open grid: path is valid");
    const double c = astar::pathCost(path);
    check(std::fabs(c - 9.0 * SQRT2) < 1e-6,
          "open grid: cost optimal (9*sqrt2 = " + std::to_string(9.0 * SQRT2) +
              ", got " + std::to_string(c) + ")");
  }

  // --- Test 2: vertical wall with gaps, must detour ----------------------
  {
    const int w = 11, h = 11;
    std::vector<int8_t> grid(w * h, 0);
    for (int y = 2; y <= 8; ++y) grid[y * w + 5] = 1;  // wall x=5, y in [2,8]
    Cell start{1, 5}, goal{9, 5};
    auto path = astar::plan(grid, w, h, start, goal);
    check(!path.empty(), "wall detour: path found");
    check(astar::pathValid(path, grid, w, h), "wall detour: path is valid");
    // Direct horizontal distance would be 8; a detour must be longer.
    check(astar::pathCost(path) > 8.0 + 1e-9,
          "wall detour: path detours around wall (cost " +
              std::to_string(astar::pathCost(path)) + " > 8)");
    std::cout << "    wall-detour map (S=start G=goal *=path #=wall):\n";
    printGrid(grid, w, h, path, start, goal);
  }

  // --- Test 3: enclosed goal -> no path -------------------------------
  {
    const int w = 7, h = 7;
    std::vector<int8_t> grid(w * h, 0);
    for (int x = 2; x <= 4; ++x)
      for (int y = 2; y <= 4; ++y)
        if (!(x == 3 && y == 3)) grid[y * w + x] = 1;  // ring around (3,3)
    Cell start{0, 0}, goal{3, 3};
    auto path = astar::plan(grid, w, h, start, goal);
    check(path.empty(), "enclosed goal: correctly returns no path");
  }

  // --- Test 4: start or goal occupied -> no path ------------------------
  {
    const int w = 5, h = 5;
    std::vector<int8_t> grid(w * h, 0);
    grid[0] = 1;  // start cell (0,0) is an obstacle
    check(astar::plan(grid, w, h, {0, 0}, {4, 4}).empty(),
          "occupied start: correctly returns no path");
    std::vector<int8_t> grid2(w * h, 0);
    grid2[h * w - 1] = 1;  // goal cell (4,4) is an obstacle
    check(astar::plan(grid2, w, h, {0, 0}, {4, 4}).empty(),
          "occupied goal: correctly returns no path");
  }

  // --- Test 5: diagonal must not cut a corner ---------------------------
  {
    const int w = 2, h = 2;
    std::vector<int8_t> grid(w * h, 0);
    grid[1 * w + 0] = 1;  // (1,0) obstacle
    grid[0 * w + 1] = 1;  // (0,1) obstacle
    // (0,0)->(1,1) is only reachable by the diagonal, which would cut the
    // corner between the two obstacles -> must be rejected.
    auto path = astar::plan(grid, w, h, {0, 0}, {1, 1});
    check(path.empty(), "corner cut: diagonal through blocked corner rejected");
  }

  // --- Test 6: snake maze (three staggered walls), sanity + validity ----
  {
    // Three horizontal walls with gaps alternating left/right/left force a
    // "snake" path. Guaranteed solvable.
    const int w = 21, h = 21;
    std::vector<int8_t> grid(w * h, 0);
    for (int x = 0; x < 19; ++x) grid[5 * w + x] = 1;    // row 5, gap at x=19
    for (int x = 1; x < 21; ++x) grid[10 * w + x] = 1;   // row 10, gap at x=0
    for (int x = 0; x < 19; ++x) grid[15 * w + x] = 1;   // row 15, gap at x=19
    Cell start{1, 1}, goal{19, 19};
    auto path = astar::plan(grid, w, h, start, goal);
    check(!path.empty(), "snake maze: path found");
    check(astar::pathValid(path, grid, w, h), "snake maze: path is valid");
    // The snake must cross three walls, so cost >> direct diagonal (~25.5).
    check(astar::pathCost(path) > 40.0,
          "snake maze: cost reflects full snake detour (cost " +
              std::to_string(astar::pathCost(path)) + " > 40)");
    std::cout << "    snake maze map:\n";
    printGrid(grid, w, h, path, start, goal);
  }

  // --- Test 7: UAV4 transit around a randomized building ----------------
  {
    const int w = 180, h = 100;
    std::vector<int8_t> grid(w * h, 0);
    // World origin (-50,-50), 1 m cells. Inflate the observed building
    // [15,31] x [-30,-18] by 1 m for vehicle clearance.
    for (int y = 19; y <= 33; ++y)
      for (int x = 64; x <= 82; ++x)
        grid[y * w + x] = 1;
    const Cell start{33, 53};  // (-17,3)
    const Cell goal{93, 7};    // (43,-43)
    const auto path = astar::plan(grid, w, h, start, goal);
    check(!path.empty(), "UAV4 transit: path found around randomized building");
    check(astar::pathValid(path, grid, w, h),
          "UAV4 transit: path stays outside inflated building");
    check(astar::pathCost(path) > std::hypot(60.0, 46.0),
          "UAV4 transit: direct line through building is rejected");
  }

  std::cout << "\n=== " << (failures == 0 ? "ALL TESTS PASSED" : "FAILURES: " +
                              std::to_string(failures)) << " ===\n";
  return failures == 0 ? 0 : 1;
}
