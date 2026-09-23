// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Plans a global path from the robot's current pose to a commanded goal, on
// the occupancy grid the robot has built.
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <arb_msgs/msg/planner_metrics.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arb_planning/grid_planner.hpp"

namespace arb_planning
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : rclcpp::Node("global_planner")
  {
    PlannerConfig cfg;
    cfg.lethal_threshold = static_cast<int>(declare_parameter("lethal_threshold", 65));
    cfg.unknown_is_lethal = declare_parameter("unknown_is_lethal", true);
    cfg.inflation_radius_m = declare_parameter("inflation_radius_m", 0.28);
    cfg.cost_decay_radius_m = declare_parameter("cost_decay_radius_m", 0.55);
    cfg.cost_scaling = declare_parameter("cost_scaling", 12.0);
    cfg.max_soft_cost = declare_parameter("max_soft_cost", 2.5);
    cfg.allow_diagonal = declare_parameter("allow_diagonal", true);
    cfg.heuristic_weight = declare_parameter("heuristic_weight", 1.0);
    cfg.max_expansions = static_cast<int>(declare_parameter("max_expansions", 2000000));
    const std::string h = declare_parameter("heuristic", std::string("octile"));
    cfg.heuristic = parseHeuristic(h);

    planner_ = std::make_unique<GridPlanner>(cfg);
    replan_period_ = declare_parameter("replan_period_s", 1.0);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    goal_tolerance_ = declare_parameter("goal_tolerance_m", 0.25);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/arb/map")), map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {onMap(m);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter("goal_topic", std::string("/arb/goal")), 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {onGoal(*m);});

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      declare_parameter("path_topic", std::string("/arb/global_path")), 10);
    metrics_pub_ = create_publisher<arb_msgs::msg::PlannerMetrics>(
      declare_parameter("metrics_topic", std::string("/arb/planner_metrics")), 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(replan_period_), [this]() {replan();});

    RCLCPP_INFO(
      get_logger(),
      "Global planner: %s heuristic (weight %.2f), inflation %.2f m, "
      "decay %.2f m, unknown is %s, replanning every %.1f s",
      toString(cfg.heuristic).c_str(), cfg.heuristic_weight,
      cfg.inflation_radius_m, cfg.cost_decay_radius_m,
      cfg.unknown_is_lethal ? "lethal" : "free", replan_period_);
  }

private:
  static Heuristic parseHeuristic(const std::string & name)
  {
    if (name == "euclidean") {return Heuristic::kEuclidean;}
    if (name == "manhattan") {return Heuristic::kManhattan;}
    if (name == "none" || name == "dijkstra") {return Heuristic::kZero;}
    return Heuristic::kOctile;
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    // Rebuilding the costmap is the expensive part (a full BFS distance
    // transform), so it happens on map arrival rather than per plan.
    planner_->setMap(
      msg->data, static_cast<int>(msg->info.width),
      static_cast<int>(msg->info.height), msg->info.resolution);
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    resolution_ = msg->info.resolution;
    have_map_ = true;
    ++maps_received_;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped & msg)
  {
    goal_x_ = msg.pose.position.x;
    goal_y_ = msg.pose.position.y;
    const auto & q = msg.pose.orientation;
    goal_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    have_goal_ = true;
    RCLCPP_INFO(get_logger(), "new goal: (%.2f, %.2f)", goal_x_, goal_y_);
    replan();
  }

  std::optional<GridIndex> toCell(double x, double y) const
  {
    const int cx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int cy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    const GridIndex c{cx, cy};
    if (!planner_->inBounds(c)) {return std::nullopt;}
    return c;
  }

  /// Nearest free cell within `radius` cells, for when a start or goal lands
  /// inside the inflation band.
  ///
  /// The robot genuinely can be inside the inflated region -- the inflation is
  /// a safety margin, not a physical wall -- and refusing to plan from there
  /// leaves it stuck against a wall with no way out. Nudging the start to the
  /// nearest free cell is the standard remedy.
  std::optional<GridIndex> nearestFree(const GridIndex & c, int radius) const
  {
    if (!planner_->isLethal(c)) {return c;}
    for (int r = 1; r <= radius; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {continue;}
          const GridIndex n{c.x + dx, c.y + dy};
          if (planner_->inBounds(n) && !planner_->isLethal(n)) {return n;}
        }
      }
    }
    return std::nullopt;
  }

  void replan()
  {
    if (!have_map_ || !have_goal_) {return;}

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no robot pose: %s", ex.what());
      return;
    }
    const double rx = tf.transform.translation.x;
    const double ry = tf.transform.translation.y;

    if (std::hypot(goal_x_ - rx, goal_y_ - ry) < goal_tolerance_) {
      if (!goal_reached_) {
        RCLCPP_INFO(get_logger(), "goal reached");
        goal_reached_ = true;
        nav_msgs::msg::Path empty;
        empty.header.stamp = get_clock()->now();
        empty.header.frame_id = map_frame_;
        path_pub_->publish(empty);
      }
      return;
    }
    goal_reached_ = false;

    const auto start_cell = toCell(rx, ry);
    const auto goal_cell = toCell(goal_x_, goal_y_);
    if (!start_cell || !goal_cell) {
      publishFailure("start or goal outside the map");
      return;
    }
    const auto start_free = nearestFree(*start_cell, 8);
    const auto goal_free = nearestFree(*goal_cell, 8);
    if (!start_free) {publishFailure("no free cell near the robot"); return;}
    if (!goal_free) {publishFailure("no free cell near the goal"); return;}

    const PlanResult r = planner_->plan(*start_free, *goal_free);
    ++plans_attempted_;
    if (!r.success) {
      ++plans_failed_;
      publishFailure(r.failure_reason);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "plan failed: %s", r.failure_reason.c_str());
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = map_frame_;
    path.poses.reserve(r.path.size());
    for (const auto & c : r.path) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = origin_x_ + (c.x + 0.5) * resolution_;
      p.pose.position.y = origin_y_ + (c.y + 0.5) * resolution_;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    // Give each waypoint the heading towards the next one, so a controller can
    // use the path's orientation instead of re-deriving it.
    for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const double dx = path.poses[i + 1].pose.position.x -
        path.poses[i].pose.position.x;
      const double dy = path.poses[i + 1].pose.position.y -
        path.poses[i].pose.position.y;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, std::atan2(dy, dx));
      path.poses[i].pose.orientation.x = q.x();
      path.poses[i].pose.orientation.y = q.y();
      path.poses[i].pose.orientation.z = q.z();
      path.poses[i].pose.orientation.w = q.w();
    }
    if (!path.poses.empty()) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, goal_yaw_);
      path.poses.back().pose.orientation.x = q.x();
      path.poses.back().pose.orientation.y = q.y();
      path.poses.back().pose.orientation.z = q.z();
      path.poses.back().pose.orientation.w = q.w();
    }
    path_pub_->publish(path);

    arb_msgs::msg::PlannerMetrics m;
    m.header = path.header;
    m.algorithm = (planner_->config().heuristic == Heuristic::kZero)
      ? "dijkstra" : "astar";
    m.heuristic = toString(planner_->config().heuristic);
    m.success = true;
    m.path_length_m = r.length_cells * resolution_;
    m.path_waypoints = static_cast<std::uint32_t>(r.path.size());
    m.planning_time_ms = r.planning_time_ms;
    m.nodes_expanded = static_cast<std::uint32_t>(r.nodes_expanded);
    m.nodes_generated = static_cast<std::uint32_t>(r.nodes_generated);
    m.open_set_peak_size = r.open_set_peak;
    m.start.x = rx;
    m.start.y = ry;
    m.goal.x = goal_x_;
    m.goal.y = goal_y_;
    metrics_pub_->publish(m);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "plan: %.2f m, %u waypoints, %.2f ms, %d expanded, %d generated",
      m.path_length_m, m.path_waypoints, m.planning_time_ms,
      r.nodes_expanded, r.nodes_generated);
  }

  void publishFailure(const std::string & reason)
  {
    arb_msgs::msg::PlannerMetrics m;
    m.header.stamp = get_clock()->now();
    m.header.frame_id = map_frame_;
    m.algorithm = (planner_->config().heuristic == Heuristic::kZero)
      ? "dijkstra" : "astar";
    m.heuristic = toString(planner_->config().heuristic);
    m.success = false;
    m.failure_reason = reason;
    metrics_pub_->publish(m);
  }

  std::unique_ptr<GridPlanner> planner_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<arb_msgs::msg::PlannerMetrics>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_map_{false}, have_goal_{false}, goal_reached_{false};
  double origin_x_{0.0}, origin_y_{0.0}, resolution_{0.05};
  double goal_x_{0.0}, goal_y_{0.0}, goal_yaw_{0.0};
  double replan_period_{1.0}, goal_tolerance_{0.25};
  std::string map_frame_{"map"}, base_frame_{"base_link"};
  unsigned long maps_received_{0}, plans_attempted_{0}, plans_failed_{0};
};

}  // namespace arb_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_planning::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
