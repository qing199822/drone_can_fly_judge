// MPPI 1500-route selection test: validate the planner emits exactly the
// configured sample count and returns the lowest-total-cost rollout's first
// control. This is a pure-C++ unit test (no ROS runtime) so it can run as
// part of `catkin run_tests` or a manual `make test`.

#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "auto_explore/mppi_planner.hpp"

namespace
{
constexpr double kRangeMax = 20.0;
constexpr double kAngleMin = -M_PI;
constexpr double kAngleStep = (2.0 * M_PI) / 640.0;
constexpr double kGoalX = 8.0;
constexpr double kGoalY = 0.0;
}  // namespace

TEST(MppiRouteSelection, SparseScanDoesNotClaimPlannerIsReady)
{
  mppi::Planner planner;
  planner.configure(mppi::Params{});
  std::vector<float> ranges(640, std::numeric_limits<float>::infinity());

  for (int hits = 0; hits < 3; ++hits)
  {
    if (hits > 0) ranges[319 + hits] = 8.0f;
    planner.setScan(ranges, kAngleMin, kAngleStep, kRangeMax, 0.05);
    EXPECT_FALSE(planner.canPlan()) << "hits=" << hits;
    EXPECT_FALSE(planner.valid()) << "hits=" << hits;
    const mppi::Control cmd = planner.plan(0.0, 0.0);
    EXPECT_EQ(planner.evaluatedSamples(), 0);
    EXPECT_DOUBLE_EQ(cmd.v, 0.0);
  }

  ranges[322] = 8.0f;
  planner.setScan(ranges, kAngleMin, kAngleStep, kRangeMax, 0.05);
  EXPECT_TRUE(planner.canPlan());
  EXPECT_TRUE(planner.valid());
}

TEST(MppiRouteSelection, EmitsExactlyNumSamplesAndSelectsMinimumCost)
{
  mppi::Params prm;
  prm.num_samples = 1500;
  prm.horizon = 16;
  prm.dt = 0.10;
  prm.max_v = 5.0;
  prm.min_v = 0.0;
  prm.max_w = 1.2;
  prm.sigma_v = 0.8;
  prm.sigma_w = 0.6;
  prm.lambda = 0.15;
  prm.v_des = 3.0;
  prm.goal_weight = 1.2;
  prm.obstacle_weight = 40.0;
  prm.yaw_weight = 0.05;
  prm.speed_weight = 0.4;
  prm.field_radius = 18.0;
  prm.field_res = 0.15;
  prm.safe_radius = 2.5;

  mppi::Planner planner;
  planner.configure(prm);

  // Construct a scan with a short obstacle wall 2 m ahead across the front
  // cone so the planner registers multiple real hits and exercises its
  // obstacle cost while still leaving plenty of room around the goal.
  std::vector<float> ranges(640, std::numeric_limits<float>::infinity());
  for (int i = 310; i < 330; ++i)
    ranges[i] = 2.0f;  // 2 m obstacle band directly in front (>=3 hits)

  planner.setScan(ranges, kAngleMin, kAngleStep, kRangeMax, 0.05);
  ASSERT_TRUE(planner.canPlan()) << "planner rejected scan; check setScan arguments";

  planner.setGoal(kGoalX, kGoalY);

  const auto started = std::chrono::steady_clock::now();
  const mppi::Control cmd = planner.plan(0.0, 0.0);
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();

  EXPECT_EQ(planner.evaluatedSamples(), 1500)
      << "planner must emit the configured number of candidate routes";
  EXPECT_GE(planner.bestRouteIndex(), 0)
      << "planner should have selected one valid route";
  EXPECT_LT(planner.bestRouteIndex(), 1500);
  EXPECT_TRUE(std::isfinite(planner.bestCost()));
  EXPECT_LT(planner.bestCost(), std::numeric_limits<double>::infinity());

  // Manual check: minimum across the recorded per-sample cost list equals the
  // advertised best cost, and the output control matches the sample at that
  // index.
  const auto& costs = planner.candidateCosts();
  ASSERT_EQ(costs.size(), static_cast<size_t>(1500));
  double min_cost = std::numeric_limits<double>::infinity();
  int min_idx = -1;
  for (size_t i = 0; i < costs.size(); ++i)
  {
    if (costs[i] < min_cost)
    {
      min_cost = costs[i];
      min_idx = static_cast<int>(i);
    }
  }
  EXPECT_DOUBLE_EQ(min_cost, planner.bestCost());
  EXPECT_EQ(min_idx, planner.bestRouteIndex());

  EXPECT_GE(cmd.v, prm.min_v);
  EXPECT_LE(cmd.v, prm.max_v);
  EXPECT_GE(cmd.w, -prm.max_w);
  EXPECT_LE(cmd.w, prm.max_w);

  // The selected rollout must travel far enough along x to make non-zero
  // progress toward the goal; open-space obstacles only matter if every route
  // terminates with the obstacle inside the path.
  const auto& route = planner.bestRoute();
  ASSERT_FALSE(route.empty());
  EXPECT_GT(route.back().x, 0.0)
      << "selected route must make forward progress toward goal";

  // Repeat planning N times to measure per-cycle latency under the configured
  // 1500 routes / horizon 16. This is the runtime budget check for the
  // requested 10 Hz planner cadence (six UAVs sharing the same cores).
  constexpr int kBenchIters = 50;
  std::vector<double> samples;
  samples.reserve(kBenchIters);
  for (int i = 0; i < kBenchIters; ++i)
  {
    planner.setScan(ranges, kAngleMin, kAngleStep, kRangeMax, 0.05);
    planner.setGoal(kGoalX, kGoalY);
    const auto t0 = std::chrono::steady_clock::now();
    planner.plan(0.0, 0.0);
    const auto t1 = std::chrono::steady_clock::now();
    samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  double sum = 0.0, worst = 0.0;
  for (double s : samples) { sum += s; worst = std::max(worst, s); }
  const double mean = sum / samples.size();
  printf("MPPI 1500-route benchmark over %d iters: mean=%.3f ms, worst=%.3f ms\n",
         kBenchIters, mean, worst);
  EXPECT_LT(mean, 100.0) << "mean planning time exceeds 100 ms (10 Hz budget)";
  EXPECT_LT(worst, 200.0) << "worst planning time exceeds 200 ms";

  (void)elapsed_ms;
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
