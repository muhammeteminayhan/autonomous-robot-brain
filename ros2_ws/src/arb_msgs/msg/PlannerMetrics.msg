# One planning query, measured. Written by arb_planning for the Phase 7
# benchmark table (algorithm | path length | planning time | nodes expanded).

std_msgs/Header header

string  algorithm               # "astar" | "dijkstra"
string  heuristic               # "euclidean" | "octile" | "manhattan" | "none"

bool    success
string  failure_reason          # "" when success

float64 path_length_m
uint32  path_waypoints
float64 planning_time_ms
uint32  nodes_expanded          # popped from the open set
uint32  nodes_generated         # pushed into the open set
float64 open_set_peak_size

geometry_msgs/Pose2D start
geometry_msgs/Pose2D goal
