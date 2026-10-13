#ifndef AUTO_EXPLORE_LIDAR_ROUTE_HPP
#define AUTO_EXPLORE_LIDAR_ROUTE_HPP

#include "auto_explore/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace lidar_route {

struct Grid {
  double origin_x = 0.0, origin_y = 0.0, resolution = 0.5;
  int width = 0, height = 0;
  std::vector<int8_t> occupied;
};

inline Grid build(const std::vector<int8_t>& fine, int width, int height,
                  double fine_resolution, double origin_x, double origin_y,
                  int occupied_threshold, double resolution = 0.5,
                  double clearance = 0.5) {
  Grid out;
  if (width <= 0 || height <= 0 || fine_resolution <= 0.0 || resolution <= 0.0 ||
      fine.size() != static_cast<size_t>(width) * height)
    return out;
  out.origin_x = origin_x;
  out.origin_y = origin_y;
  out.resolution = resolution;
  out.width = static_cast<int>(std::ceil(width * fine_resolution / resolution));
  out.height = static_cast<int>(std::ceil(height * fine_resolution / resolution));
  out.occupied.assign(static_cast<size_t>(out.width) * out.height, 0);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x) {
      if (fine[static_cast<size_t>(y) * width + x] < occupied_threshold) continue;
      const int cx = std::min(out.width - 1,
          static_cast<int>((x + 0.5) * fine_resolution / resolution));
      const int cy = std::min(out.height - 1,
          static_cast<int>((y + 0.5) * fine_resolution / resolution));
      out.occupied[static_cast<size_t>(cy) * out.width + cx] = 1;
    }
  const int radius = std::max(0, static_cast<int>(std::round(clearance / resolution)));
  if (radius > 0) {
    const auto base = out.occupied;
    for (int y = 0; y < out.height; ++y)
      for (int x = 0; x < out.width; ++x) {
        if (!base[static_cast<size_t>(y) * out.width + x]) continue;
        for (int dy = -radius; dy <= radius; ++dy)
          for (int dx = -radius; dx <= radius; ++dx) {
            const int nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < out.width && ny >= 0 && ny < out.height)
              out.occupied[static_cast<size_t>(ny) * out.width + nx] = 1;
          }
      }
  }
  return out;
}

inline std::vector<std::pair<double, double>> plan(const Grid& grid,
                                                    double sx, double sy,
                                                    double gx, double gy) {
  if (grid.occupied.empty()) return {};
  const auto cell = [&](double x, double y) {
    return astar::Cell{static_cast<int>(std::floor((x - grid.origin_x) / grid.resolution)),
                       static_cast<int>(std::floor((y - grid.origin_y) / grid.resolution))};
  };
  const auto path = astar::plan(grid.occupied, grid.width, grid.height,
                                cell(sx, sy), cell(gx, gy));
  if (path.size() < 2) return {};
  std::vector<std::pair<double, double>> world;
  world.reserve(path.size());
  for (const auto& c : path)
    world.emplace_back(grid.origin_x + (c.x + 0.5) * grid.resolution,
                       grid.origin_y + (c.y + 0.5) * grid.resolution);
  return world;
}

}  // namespace lidar_route

#endif
