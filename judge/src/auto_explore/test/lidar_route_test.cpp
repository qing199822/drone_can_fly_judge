#include "auto_explore/lidar_route.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

int main() {
  const int w = 2300, h = 1200;
  const double ox = -90.0, oy = -60.0, res = 0.1;
  std::vector<int8_t> fine(static_cast<size_t>(w) * h, -1);
  const auto fill = [&](double xmin, double xmax, double ymin, double ymax) {
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const double wx = ox + (x + 0.5) * res;
        const double wy = oy + (y + 0.5) * res;
        if (wx >= xmin && wx <= xmax && wy >= ymin && wy <= ymax)
          fine[static_cast<size_t>(y) * w + x] = 100;
      }
  };
  // Synthetic lidar map: only observed wall cells are occupied; all other
  // cells remain unknown. The geometry is a test fixture, not navigation input.
  fill(12.4, 43.6, -41.1, -14.9);
  const auto grid = lidar_route::build(fine, w, h, res, ox, oy, 50);
  const auto route = lidar_route::plan(grid, -17.0, 3.0, 43.0, -43.0);
  assert(route.size() > 80);  // Full route, not a 40 m rolling-window fragment.
  for (const auto& p : route)
    assert(!(p.first >= 12.4 && p.first <= 43.6 &&
             p.second >= -41.1 && p.second <= -14.9));
  for (size_t i = 1; i < route.size(); ++i) {
    for (int j = 0; j <= 10; ++j) {
      const double t = j / 10.0;
      const double x = route[i - 1].first + t * (route[i].first - route[i - 1].first);
      const double y = route[i - 1].second + t * (route[i].second - route[i - 1].second);
      assert(!(x >= 12.4 && x <= 43.6 && y >= -41.1 && y <= -14.9));
    }
  }

  std::fill(fine.begin(), fine.end(), -1);
  const auto unseen = lidar_route::build(fine, w, h, res, ox, oy, 50);
  const auto direct = lidar_route::plan(unseen, -17.0, 3.0, 43.0, -43.0);
  assert(!direct.empty() && direct.size() < route.size());
  std::cout << "PASS: observed lidar obstacle changes UAV4's complete transit route\n";
}
