// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Benchmarks every planner configuration on the SAME map and the same set of
// queries, and writes the table Phase 7 asks for.
//
// It takes the map from the running stack rather than a synthetic one, because
// a benchmark on an empty grid measures the heuristic's best case and nothing
// else: A* with octile expands a corridor there and Dijkstra expands a disc,
// so the ratio is whatever the grid size happens to make it. A real map with
// walls, doorways and dead ends is where the numbers mean something.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arb_planning/grid_planner.hpp"

namespace arb_planning
{

struct Query
{
  std::string name;
  double sx, sy, gx, gy;
};

struct Row
{
  std::string algorithm;
  std::string heuristic;
  double weight{1.0};
  int solved{0};
  double length_m{0.0};
  double time_ms{0.0};
  double expanded{0.0};
  double generated{0.0};
  double peak_open{0.0};
  double excess_pct{0.0};     ///< path length above the optimal reference
};

class BenchmarkNode : public rclcpp::Node
{
public:
  BenchmarkNode()
  : rclcpp::Node("planner_benchmark")
  {
    output_ = declare_parameter(
      "output_csv", std::string("results/phase7/planner_benchmark.csv"));
    repeats_ = static_cast<int>(declare_parameter("repeats", 5));
    inflation_ = declare_parameter("inflation_radius_m", 0.28);
    decay_ = declare_parameter("cost_decay_radius_m", 0.55);
    unknown_lethal_ = declare_parameter("unknown_is_lethal", true);

    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/arb/map")), qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {onMap(m);});
  }

  bool done() const {return done_;}

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    if (done_) {return;}
    run(*msg);
    done_ = true;
  }

  static double toCellX(const nav_msgs::msg::OccupancyGrid & m, double x)
  {
    return std::floor((x - m.info.origin.position.x) / m.info.resolution);
  }
  static double toCellY(const nav_msgs::msg::OccupancyGrid & m, double y)
  {
    return std::floor((y - m.info.origin.position.y) / m.info.resolution);
  }

  void run(const nav_msgs::msg::OccupancyGrid & map)
  {
    PlannerConfig probe_cfg;
    probe_cfg.inflation_radius_m = inflation_;
    probe_cfg.cost_decay_radius_m = decay_;
    probe_cfg.unknown_is_lethal = unknown_lethal_;
    GridPlanner probe(probe_cfg);
    probe.setMap(map.data, static_cast<int>(map.info.width),
                 static_cast<int>(map.info.height), map.info.resolution);

    // Queries are GENERATED from the map rather than hard-coded.
    //
    // A fixed list of world coordinates is fragile: the robot only ever maps
    // part of the world, and with unknown cells treated as lethal a query into
    // unexplored space is unsolvable through no fault of the planner. Half the
    // first version's queries failed that way, which says something about
    // coverage and nothing about A*.
    //
    // Sampling free cells from the map that the planner is actually given
    // makes the benchmark measure the planner. The sampling is deterministic
    // so the numbers are reproducible.
    std::vector<GridIndex> free_cells;
    for (int y = 0; y < probe.height(); y += 3) {
      for (int x = 0; x < probe.width(); x += 3) {
        if (!probe.isLethal({x, y})) {free_cells.push_back({x, y});}
      }
    }
    if (free_cells.size() < 20) {
      RCLCPP_ERROR(get_logger(),
                   "only %zu free cells in the map; explore more first",
                   free_cells.size());
      return;
    }

    std::vector<Query> queries;
    std::mt19937 rng(20260923u);
    std::uniform_int_distribution<std::size_t> pick(0, free_cells.size() - 1);
    const double min_separation_cells = 60.0;
    int attempts = 0;
    while (queries.size() < 12 && attempts < 20000) {
      ++attempts;
      const GridIndex a = free_cells[pick(rng)];
      const GridIndex b = free_cells[pick(rng)];
      const double d = std::hypot(
        static_cast<double>(a.x - b.x), static_cast<double>(a.y - b.y));
      if (d < min_separation_cells) {continue;}
      // Only keep a pair the reference planner can actually connect, so every
      // row of the table is the same set of problems.
      if (!probe.plan(a, b).success) {continue;}
      Query q;
      q.name = "q" + std::to_string(queries.size());
      q.sx = map.info.origin.position.x + (a.x + 0.5) * map.info.resolution;
      q.sy = map.info.origin.position.y + (a.y + 0.5) * map.info.resolution;
      q.gx = map.info.origin.position.x + (b.x + 0.5) * map.info.resolution;
      q.gy = map.info.origin.position.y + (b.y + 0.5) * map.info.resolution;
      queries.push_back(q);
    }
    RCLCPP_INFO(get_logger(),
                "generated %zu solvable queries from %zu free cells "
                "(min separation %.1f m)",
                queries.size(), free_cells.size(),
                min_separation_cells * map.info.resolution);

    struct Setup
    {
      std::string algorithm;
      Heuristic heuristic;
      double weight;
    };
    const std::vector<Setup> setups = {
      {"dijkstra", Heuristic::kZero, 1.0},
      {"astar", Heuristic::kEuclidean, 1.0},
      {"astar", Heuristic::kOctile, 1.0},
      {"astar", Heuristic::kManhattan, 1.0},
      {"astar", Heuristic::kOctile, 2.0},
      {"astar", Heuristic::kOctile, 5.0},
    };

    PlannerConfig base;
    base.inflation_radius_m = inflation_;
    base.cost_decay_radius_m = decay_;
    base.unknown_is_lethal = unknown_lethal_;

    // The optimal reference. Dijkstra needs no heuristic to be right, so its
    // cost is the ground truth every other row is scored against.
    GridPlanner reference(base);
    reference.setMap(map.data, static_cast<int>(map.info.width),
                     static_cast<int>(map.info.height), map.info.resolution);
    reference.mutableConfig().heuristic = Heuristic::kZero;

    std::vector<double> optimal(queries.size(), 0.0);
    std::vector<bool> solvable(queries.size(), false);
    for (std::size_t i = 0; i < queries.size(); ++i) {
      const auto & q = queries[i];
      const PlanResult r = reference.plan(
        {static_cast<int>(toCellX(map, q.sx)), static_cast<int>(toCellY(map, q.sy))},
        {static_cast<int>(toCellX(map, q.gx)), static_cast<int>(toCellY(map, q.gy))});
      solvable[i] = r.success;
      optimal[i] = r.cost;
      if (!r.success) {
        RCLCPP_WARN(get_logger(), "query '%s' unsolvable: %s",
                    q.name.c_str(), r.failure_reason.c_str());
      }
    }

    std::vector<Row> rows;
    std::vector<std::string> per_query_lines;

    for (const auto & s : setups) {
      GridPlanner planner(base);
      planner.setMap(map.data, static_cast<int>(map.info.width),
                     static_cast<int>(map.info.height), map.info.resolution);
      planner.mutableConfig().heuristic = s.heuristic;
      planner.mutableConfig().heuristic_weight = s.weight;

      Row row;
      row.algorithm = s.algorithm;
      row.heuristic = toString(s.heuristic);
      row.weight = s.weight;

      for (std::size_t i = 0; i < queries.size(); ++i) {
        if (!solvable[i]) {continue;}
        const auto & q = queries[i];
        const GridIndex start{static_cast<int>(toCellX(map, q.sx)),
          static_cast<int>(toCellY(map, q.sy))};
        const GridIndex goal{static_cast<int>(toCellX(map, q.gx)),
          static_cast<int>(toCellY(map, q.gy))};

        // Repeat and take the MEDIAN, not the mean. A single run is dominated
        // by cache state and whatever else the machine is doing; the mean is
        // dragged by one slow outlier while the median is not.
        std::vector<double> times;
        PlanResult r;
        for (int k = 0; k < repeats_; ++k) {
          r = planner.plan(start, goal);
          times.push_back(r.planning_time_ms);
        }
        std::sort(times.begin(), times.end());
        const double median = times[times.size() / 2];
        if (!r.success) {continue;}

        ++row.solved;
        row.length_m += r.length_cells * map.info.resolution;
        row.time_ms += median;
        row.expanded += r.nodes_expanded;
        row.generated += r.nodes_generated;
        row.peak_open += r.open_set_peak;
        row.excess_pct += 100.0 * (r.cost / optimal[i] - 1.0);

        char line[512];
        std::snprintf(
          line, sizeof(line), "%s,%s,%.1f,%s,%.4f,%.4f,%d,%d,%d,%.4f",
          s.algorithm.c_str(), toString(s.heuristic).c_str(), s.weight,
          q.name.c_str(), r.length_cells * map.info.resolution, median,
          r.nodes_expanded, r.nodes_generated, r.open_set_peak,
          100.0 * (r.cost / optimal[i] - 1.0));
        per_query_lines.push_back(line);
      }

      if (row.solved > 0) {
        const double n = row.solved;
        row.length_m /= n;
        row.time_ms /= n;
        row.expanded /= n;
        row.generated /= n;
        row.peak_open /= n;
        row.excess_pct /= n;
      }
      rows.push_back(row);
    }

    // --- console table ----------------------------------------------------
    std::printf("\n");
    std::printf("================================================================"
                "========================\n");
    std::printf("PHASE 7 -- GLOBAL PLANNER BENCHMARK\n");
    std::printf("  map %u x %u at %.3f m, %d queries, median of %d runs each\n",
                map.info.width, map.info.height, map.info.resolution,
                static_cast<int>(queries.size()), repeats_);
    std::printf("================================================================"
                "========================\n");
    std::printf("%-10s %-11s %6s %7s %10s %10s %11s %9s %9s\n",
                "algorithm", "heuristic", "weight", "solved", "length[m]",
                "time[ms]", "expanded", "generated", "excess%");
    std::printf("----------------------------------------------------------------"
                "------------------------\n");
    for (const auto & r : rows) {
      std::printf("%-10s %-11s %6.1f %4d/%-2zu %10.3f %10.3f %11.0f %9.0f %9.2f\n",
                  r.algorithm.c_str(), r.heuristic.c_str(), r.weight,
                  r.solved, queries.size(), r.length_m, r.time_ms,
                  r.expanded, r.generated, r.excess_pct);
    }
    std::printf("\n");

    // --- CSV --------------------------------------------------------------
    std::ofstream out(output_);
    if (out) {
      out << "algorithm,heuristic,weight,query,length_m,time_ms,"
             "nodes_expanded,nodes_generated,open_peak,excess_pct\n";
      for (const auto & l : per_query_lines) {out << l << "\n";}
      std::printf("  wrote %s\n\n", output_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "could not write %s", output_.c_str());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::string output_;
  int repeats_{5};
  double inflation_{0.28};
  double decay_{0.55};
  bool unknown_lethal_{true};
  bool done_{false};
};

}  // namespace arb_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arb_planning::BenchmarkNode>();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (rclcpp::ok() && !node->done() &&
         std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!node->done()) {
    RCLCPP_ERROR(node->get_logger(), "no map received; is arb_mapping running?");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
