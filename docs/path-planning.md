# Path Planning

> Phase 7 complete and benchmarked. Phase 8 (local obstacle avoidance) follows.

## 1. A* is Dijkstra told where the goal is

Every cell carries three numbers:

```
g(n)  cost of the best path found so far from the start to n
h(n)  an estimate of the remaining cost from n to the goal
f(n)  = g(n) + h(n), the estimated cost of the best path THROUGH n
```

The **open set** holds cells discovered but not yet examined, ordered by `f`.
The **closed set** holds cells already examined. The loop is:

```
pop the lowest-f cell from open
if it is the goal, done
move it to closed
for each neighbour, compute a tentative g
if that beats the neighbour's current g, record it and push it into open
```

Set `h = 0` and `f = g`: the open set is ordered by distance from the start and
the algorithm is exactly Dijkstra. A* is not a different algorithm.

## 2. Admissibility is the whole guarantee

A* returns an **optimal** path if and only if `h` never overestimates the true
remaining cost. Losing that does not raise an error — it returns a plausible,
slightly wrong path.

On an 8-connected grid a diagonal step costs `sqrt(2)` and covers one cell in
both axes, so the true minimum cost is the **octile distance**:

```
octile(dx, dy) = (sqrt(2) - 1) * min(dx, dy) + max(dx, dy)
```

| Heuristic | Admissible? | Notes |
|---|---|---|
| **Octile** | yes, and **tight** | equals the true cost on an empty grid, so A* expands almost nothing |
| **Euclidean** | yes, looser | `sqrt(dx^2+dy^2)`; underestimates in between, so more expansions for the same answer |
| **Manhattan** | **no** | claims `2d` for a pure diagonal whose true cost is `1.41d` |
| **Zero** | yes, useless | this is Dijkstra |

Measured over an 18-instance sweep, Manhattan was suboptimal on **10 of them**,
worst case **8.9 %** longer than necessary. Most of the time it finds the
optimum anyway, which is exactly what makes it dangerous.

## 3. Benchmark

Twelve start/goal pairs generated from the map the robot built, minimum 3 m
apart, median of five runs each. The 480 x 360 grid at 5 cm resolution is
172 800 cells.

| Algorithm | Heuristic | Weight | Length [m] | Time [ms] | Expanded | Generated | Excess |
|---|---|---|---|---|---|---|---|
| Dijkstra | none | 1.0 | 13.211 | 6.764 | 45 836 | 46 977 | 0.00 % |
| A* | euclidean | 1.0 | 13.211 | 2.691 | 13 171 | 20 312 | 0.00 % |
| **A\*** | **octile** | **1.0** | **13.211** | **2.812** | **10 947** | **22 365** | **0.00 %** |
| A* | manhattan | 1.0 | 13.174 | 1.658 | 6 573 | 14 463 | **+1.30 %** |
| A* | octile | 2.0 | 13.322 | 0.959 | 3 896 | 8 996 | **+13.99 %** |
| A* | octile | 5.0 | 13.544 | 0.669 | 2 847 | 6 643 | **+47.00 %** |

Reading it:

* **A\* expands 4.2x fewer nodes than Dijkstra** for an identical path. That is
  the entire value of a heuristic.
* **Octile expands 17 % fewer nodes than Euclidean**, as the tightness
  predicts — but it *generates* more and its wall-clock time is a wash.
  Expansions are the algorithmic metric; time is the engineering one, and they
  do not always agree because a tighter heuristic reorders the queue more.
* **Manhattan is the fastest admissible-looking option and is not admissible.**
  1.30 % longer paths on average, for free, silently.
* **Weighted A* is an honest trade**: at weight 5 it is 10x faster than
  Dijkstra and 47 % off optimal. Worth having, worth labelling.

## 4. The costmap

Obstacles are **inflated** by the robot's radius so the planner can treat the
robot as a point. Planning for a point in an inflated world is exactly
equivalent to planning for a disc in the real one, and far cheaper than
checking a footprint at every cell.

The robot's circumscribed radius is `sqrt(0.40^2 + 0.30^2)/2 = 0.25 m`;
the configured inflation is 0.28 m. The narrowest doorway in the world is
0.90 m, leaving 0.34 m of free corridor through it.

Beyond the lethal band a **decaying soft cost** keeps paths in the middle of a
corridor. Without it the shortest path hugs every wall, which leaves the local
planner no room and turns any control error into a collision.

The distance transform is a multi-source BFS from every obstacle — one pass
over the map, rather than the `O(obstacles * radius^2)` splat a naive
implementation uses. On this grid that is milliseconds against a second.

## 5. Details that are not details

**Diagonal moves must not cut corners.** Two obstacles meeting at a diagonal
leave a gap a *point* can slip through and a disc cannot. Allowing it produces
paths that are geometrically shortest and physically impossible.

**Tie-break on larger `g`.** Two nodes with equal `f` are equally promising,
but the one further from the start is closer to the goal. Expanding it first
drives towards the goal instead of fanning out through the whole ring of
equal-`f` cells.

**Stale queue entries.** `std::priority_queue` has no decrease-key, so a cell
reached by a better path is pushed again. The outdated copy must be discarded
when it surfaces, and checking the closed set alone is not enough — the check
on `g` is what catches a duplicate that has not been expanded yet.

**Unknown is lethal, but does not radiate.** Unknown cells are impassable, yet
they must not push soft cost into the known free space beside them, or the
frontier of the map becomes a wall and the robot refuses to approach anywhere
it has not already been.
