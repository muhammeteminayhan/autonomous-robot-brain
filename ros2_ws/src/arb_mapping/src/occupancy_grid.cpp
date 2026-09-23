// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_mapping/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace arb_mapping
{

OccupancyGrid::OccupancyGrid(const GridConfig & config)
: config_(config)
{
  width_ = std::max(1, static_cast<int>(std::round(config_.width_m / config_.resolution_m)));
  height_ = std::max(1, static_cast<int>(std::round(config_.height_m / config_.resolution_m)));
  l_hit_ = probabilityToLogOdds(config_.p_hit);
  l_miss_ = probabilityToLogOdds(config_.p_miss);
  l_prior_ = probabilityToLogOdds(config_.p_prior);
  clear();
}

double OccupancyGrid::probabilityToLogOdds(double p)
{
  // Clamped away from 0 and 1: log-odds of a certainty is infinite, and an
  // infinite cell can never be revised by any amount of later evidence.
  const double clamped = std::min(std::max(p, 1e-6), 1.0 - 1e-6);
  return std::log(clamped / (1.0 - clamped));
}

double OccupancyGrid::logOddsToProbability(double l)
{
  return 1.0 - 1.0 / (1.0 + std::exp(l));
}

void OccupancyGrid::clear()
{
  // The initial value is the PRIOR, not zero, so that a non-uniform prior is
  // representable. With p_prior = 0.5 they coincide, which is why the
  // distinction is easy to miss and easy to get wrong later.
  log_odds_.assign(static_cast<std::size_t>(width_) * height_,
                   static_cast<float>(l_prior_));
}

bool OccupancyGrid::worldToCell(double x, double y, int & cx, int & cy) const
{
  if (!std::isfinite(x) || !std::isfinite(y)) {return false;}
  // std::floor, NOT a cast to int. A cast truncates towards zero, so cells at
  // negative coordinates are shifted by one and the grid is subtly asymmetric
  // about the origin -- a bug that only appears on one side of the map.
  cx = static_cast<int>(std::floor((x - config_.origin_x_m) / config_.resolution_m));
  cy = static_cast<int>(std::floor((y - config_.origin_y_m) / config_.resolution_m));
  return inBounds(cx, cy);
}

void OccupancyGrid::cellToWorld(int cx, int cy, double & x, double & y) const
{
  x = config_.origin_x_m + (cx + 0.5) * config_.resolution_m;
  y = config_.origin_y_m + (cy + 0.5) * config_.resolution_m;
}

double OccupancyGrid::logOdds(int cx, int cy) const
{
  if (!inBounds(cx, cy)) {return l_prior_;}
  return log_odds_[static_cast<std::size_t>(cy) * width_ + cx];
}

double OccupancyGrid::probability(int cx, int cy) const
{
  return logOddsToProbability(logOdds(cx, cy));
}

CellState OccupancyGrid::state(int cx, int cy) const
{
  const double l = logOdds(cx, cy);
  if (l >= config_.occupied_threshold) {return CellState::kOccupied;}
  if (l <= config_.free_threshold) {return CellState::kFree;}
  return CellState::kUnknown;
}

void OccupancyGrid::updateCell(int cx, int cy, double delta)
{
  if (!inBounds(cx, cy)) {return;}
  const std::size_t i = static_cast<std::size_t>(cy) * width_ + cx;
  const double updated = static_cast<double>(log_odds_[i]) + delta;
  log_odds_[i] = static_cast<float>(
    std::min(std::max(updated, config_.log_odds_min), config_.log_odds_max));
}

std::vector<std::pair<int, int>> OccupancyGrid::bresenham(
  int x0, int y0, int x1, int y1)
{
  if (x0 == x1 && y0 == y1) {return {};}

  // The traversal is CANONICALISED before running, then reversed if needed.
  //
  // Plain Bresenham breaks exact diagonal ties in favour of whichever end it
  // started from, so the same physical segment traversed in the two directions
  // visits slightly different cells. For an occupancy grid that means a wall
  // scanned from the left and from the right clears marginally different
  // cells, and the map develops direction-dependent seams along diagonals.
  //
  // Sorting the endpoints makes the cell set a property of the SEGMENT rather
  // than of the direction of travel, which costs one comparison.
  const bool swapped = std::make_pair(x1, y1) < std::make_pair(x0, y0);
  const int ax = swapped ? x1 : x0;
  const int ay = swapped ? y1 : y0;
  const int bx = swapped ? x0 : x1;
  const int by = swapped ? y0 : y1;

  // Integer-only Bresenham over the canonical segment, BOTH endpoints
  // included. The error term tracks twice the distance from the ideal line,
  // which is what keeps everything in integers and makes the traversal exact.
  std::vector<std::pair<int, int>> cells;
  const int dx = std::abs(bx - ax);
  const int dy = -std::abs(by - ay);
  const int sx = (ax < bx) ? 1 : -1;
  const int sy = (ay < by) ? 1 : -1;
  int err = dx + dy;

  int x = ax;
  int y = ay;
  // A generous bound: no line inside a sane grid needs more steps, and it
  // turns a hypothetical infinite loop into a truncated ray.
  const int limit = dx - dy + 2;
  cells.reserve(static_cast<std::size_t>(limit));
  for (int guard = 0; guard <= limit; ++guard) {
    cells.emplace_back(x, y);
    if (x == bx && y == by) {break;}
    const int e2 = 2 * err;
    if (e2 >= dy) {err += dy; x += sx;}
    if (e2 <= dx) {err += dx; y += sy;}
  }

  // Restore the caller's direction, then drop the caller's ENDPOINT: that is
  // the cell the beam hit, and marking it free before marking it occupied
  // would have the two updates fight each other.
  if (swapped) {std::reverse(cells.begin(), cells.end());}
  if (!cells.empty()) {cells.pop_back();}
  return cells;
}

bool OccupancyGrid::integrateBeam(
  const arb_core::Pose2D & sensor, double bearing, double range)
{
  int sx = 0;
  int sy = 0;
  if (!worldToCell(sensor.x, sensor.y, sx, sy)) {
    // The sensor is outside the grid; nothing about this beam is representable.
    return false;
  }
  if (!std::isfinite(bearing)) {return false;}

  const bool no_return = !std::isfinite(range) || range > config_.max_usable_range_m;
  if (!no_return && range < config_.min_usable_range_m) {
    // Too close to be anything but the robot's own structure.
    return false;
  }

  // A beam that returned nothing still carries information: the space up to
  // the usable range was empty. Clearing it is what turns open floor from
  // unknown into free, and skipping these beams leaves large voids that a
  // planner cannot cross.
  const double effective = no_return ? config_.max_usable_range_m : range;

  const double ex = sensor.x + effective * std::cos(bearing);
  const double ey = sensor.y + effective * std::sin(bearing);

  int cx = 0;
  int cy = 0;
  const bool endpoint_inside = worldToCell(ex, ey, cx, cy);
  if (!endpoint_inside) {
    // Clip to the grid edge so the visible part of the ray still clears cells.
    cx = std::min(std::max(cx, 0), width_ - 1);
    cy = std::min(std::max(cy, 0), height_ - 1);
  }

  for (const auto & [fx, fy] : bresenham(sx, sy, cx, cy)) {
    updateCell(fx, fy, l_miss_ - l_prior_);
  }

  // Only a genuine return marks an obstacle. Marking the endpoint of a
  // max-range beam occupied would paint a phantom wall at exactly the sensor's
  // range limit, in a perfect arc, all the way around the robot.
  if (!no_return && endpoint_inside) {
    updateCell(cx, cy, l_hit_ - l_prior_);
  }
  return true;
}

int OccupancyGrid::integrateScan(
  const arb_core::Pose2D & sensor, const std::vector<float> & ranges,
  double angle_min, double angle_increment, double range_min, double range_max)
{
  int used = 0;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double bearing = sensor.theta + angle_min +
      static_cast<double>(i) * angle_increment;
    double r = static_cast<double>(ranges[i]);
    // Normalise everything unmeasurable to one sentinel before deciding.
    if (!std::isfinite(r) || r > range_max) {
      r = std::numeric_limits<double>::infinity();
    } else if (r < range_min) {
      continue;
    }
    if (integrateBeam(sensor, bearing, r)) {++used;}
  }
  return used;
}

std::vector<std::int8_t> OccupancyGrid::toOccupancyBytes() const
{
  std::vector<std::int8_t> out(log_odds_.size());
  for (std::size_t i = 0; i < log_odds_.size(); ++i) {
    const double l = log_odds_[i];
    if (l > config_.free_threshold && l < config_.occupied_threshold) {
      out[i] = -1;   // unknown, which is NOT the same as 50 % occupied
    } else {
      out[i] = static_cast<std::int8_t>(
        std::lround(100.0 * logOddsToProbability(l)));
    }
  }
  return out;
}

OccupancyGrid::Counts OccupancyGrid::counts() const
{
  Counts c;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      switch (state(x, y)) {
        case CellState::kFree: ++c.free; break;
        case CellState::kOccupied: ++c.occupied; break;
        default: ++c.unknown; break;
      }
    }
  }
  return c;
}

}  // namespace arb_mapping
