---
description: "Phase 3 — Navigation & Exploration: frontier explorer (Canny + cost-utility), waypoint manager, autonomous mode"
---

Execute Phase 3 of the autonomous-explorer-ros2 project. This is the **showcase phase** — the code produced here is what recruiters will open first on the GitHub repo. Quality must reflect that.

## Context

Phase 1 (foundations) and Phase 2 (perception + Nav2) are complete and validated. The robot can navigate to manually-set goals via Nav2 (NavFn planner + DWB controller) with live SLAM. Phase 3 adds **autonomy**: the robot must decide where to go on its own.

## Architectural Principles (NON-NEGOTIABLE)

These rules apply to **every file produced in this phase**:

1. **Separation of concerns.** Core algorithms (frontier detection, clustering, utility scoring) MUST be implemented as **pure functions** in a dedicated module (e.g. `frontier_detection.py`) that has **zero ROS dependencies**. The ROS node (`frontier_explorer_node.py`) only handles I/O: subscriptions, action clients, publishing. This enables unit tests without ROS runtime.

2. **Type hints everywhere.** Every function signature uses Python type hints. Use `numpy.typing.NDArray` for arrays. No `Any` unless unavoidable.

3. **Docstrings in NumPy style** on every public function and class. Include `Parameters`, `Returns`, and a short `Notes` section explaining algorithmic choices where relevant.

4. **YAML-first configuration.** No magic numbers in code. Every threshold, weight, or timeout is a ROS parameter declared in `exploration_params.yaml` with a comment explaining its role.

5. **Logging with levels.** Use `self.get_logger().debug/info/warn/error` appropriately. INFO for state transitions, DEBUG for per-cycle data, WARN for recoverable issues, ERROR for failures.

## Deliverables

### 1. `frontier_detection.py` (pure Python, NO ROS imports)

This is **the showcase module**. Recruiters will read this file line by line.

**Required functions:**

```python
def occupancy_grid_to_image(grid: NDArray[np.int8], width: int, height: int) -> NDArray[np.uint8]:
    """Convert ROS OccupancyGrid data to a 3-level grayscale image.

    Mapping: -1 (unknown) → 127, 0 (free) → 255, 100 (occupied) → 0.
    """

def detect_frontier_cells(image: NDArray[np.uint8]) -> NDArray[np.uint8]:
    """Detect frontier pixels using Canny edge detection on free space.

    Pipeline:
        1. Extract free-space mask (pixel == 255).
        2. Apply Canny edge detector on the free mask to find boundaries.
        3. Dilate the unknown-space mask (pixel == 127) by 1 pixel.
        4. Intersect: frontier = edges_of_free ∩ dilated_unknown.
        5. Morphological closing to consolidate thin frontier strips.

    Returns a binary image where 255 marks frontier cells.

    Notes
    -----
    Using Canny on the occupancy grid (treating the map as a grayscale image)
    rather than the classic Wavefront Frontier Detection (WFD, Yamauchi 1997)
    leverages mature, fast image-processing primitives and integrates naturally
    with the camera-based obstacle_detector from Phase 2.
    """

def cluster_frontiers(
    frontier_image: NDArray[np.uint8],
    min_cluster_size: int,
) -> list[FrontierCluster]:
    """Group connected frontier pixels into clusters with centroids.

    Uses cv2.connectedComponentsWithStats. Discards clusters smaller than
    min_cluster_size (in pixels) to filter noise.
    """

def select_best_frontier(
    clusters: list[FrontierCluster],
    robot_position_px: tuple[int, int],
    info_gain_weight: float,
    distance_weight: float,
    blacklist: set[tuple[int, int]],
) -> FrontierCluster | None:
    """Select the frontier maximizing utility = α·info_gain − β·distance.

    info_gain is normalized cluster size (0-1).
    distance is normalized Euclidean distance to centroid (0-1).
    Blacklisted centroids are excluded. Returns None if no valid frontier remains.
    """
```

**Required dataclass:**

```python
@dataclass(frozen=True)
class FrontierCluster:
    centroid_px: tuple[int, int]  # pixel coords in map frame
    size: int                      # number of frontier pixels
    bbox: tuple[int, int, int, int]  # x, y, w, h
```

**Grid ↔ world coordinate helpers:**

```python
def grid_to_world(px: tuple[int, int], origin: Pose2D, resolution: float) -> tuple[float, float]: ...
def world_to_grid(xy: tuple[float, float], origin: Pose2D, resolution: float) -> tuple[int, int]: ...
```

### 2. `frontier_explorer_node.py` (ROS2 node, thin wrapper)

- Subscribes to `/map` (nav_msgs/OccupancyGrid) — QoS transient local.
- Uses `tf2_ros` buffer/listener to get current robot pose in map frame.
- On each map update (throttled to max 1 Hz): runs `detect → cluster → select` pipeline from `frontier_detection.py`.
- Sends the selected frontier centroid as a goal via **action client** `NavigateToPose` (not topic publish — action client gives you feedback and result).
- Publishes `visualization_msgs/MarkerArray` on `/frontiers` to visualize clusters in rviz2 (use cube list, color-graded by utility score).
- Publishes exploration progress (% of known-vs-unknown cells) on `/exploration/progress` (std_msgs/Float32) at 1 Hz.
- Stops when `select_best_frontier` returns None for 3 consecutive iterations (map complete).

### 3. `waypoint_manager_node.py`

Keep it simple — this is supporting infrastructure, not the vitrine:
- Subscribes to `/waypoints` (geometry_msgs/PoseArray) — one-shot queue reload.
- Sends goals sequentially via `NavigateToPose` action client.
- On goal failure: logs warning, **retries once**, then skips to next waypoint.
- Publishes `/waypoint_status` (std_msgs/String) with JSON `{current_index, total, status}` for debugging.

### 4. Recovery logic (inside `frontier_explorer_node.py`)

- **Stuck detection**: if the robot's position changes by less than `stuck_threshold_m` (default 0.1) over `stuck_timeout_s` (default 30s), cancel current goal and call Nav2 `BackUp` + `Spin` behaviors via action clients.
- **Unreachable frontier**: if Nav2 returns `ABORTED` twice for the same frontier centroid, add it to a blacklist (set of grid cells with radius-based matching).
- **Blacklist reset**: when no non-blacklisted frontier remains but unknown space > threshold, clear blacklist and log a warning.

### 5. `exploration_params.yaml`

All parameters with inline comments:

```yaml
frontier_explorer:
  ros__parameters:
    # Detection
    min_cluster_size_px: 10          # reject clusters smaller than this (noise filter)
    map_update_rate_hz: 1.0          # throttle replanning to avoid thrashing

    # Selection (utility = α·info_gain − β·distance)
    info_gain_weight: 1.0            # α: preference for large unexplored regions
    distance_weight: 0.5             # β: penalty for far-away frontiers

    # Recovery
    stuck_threshold_m: 0.1
    stuck_timeout_s: 30.0
    max_retries_per_frontier: 2

    # Termination
    completion_threshold: 0.95       # stop when 95% of reachable map is known
    empty_iterations_before_stop: 3  # consecutive no-frontier results to confirm done
```

### 6. `full_system.launch.py`

Composes `simulation.launch.py` + `navigation.launch.py` + `frontier_explorer` + `waypoint_manager`. Use `IncludeLaunchDescription` — no code duplication.

### 7. Unit tests

In `test/test_frontier_detection.py`, using pytest:
- `test_occupancy_grid_to_image`: verify the three-level mapping on a synthetic 5×5 grid.
- `test_detect_frontier_cells_simple_case`: a hand-crafted grid with one known frontier — assert it's detected.
- `test_detect_frontier_cells_no_frontier`: fully known map — assert empty result.
- `test_cluster_frontiers_filters_small`: assert noise below min_cluster_size is rejected.
- `test_select_best_frontier_utility_tradeoff`: two clusters, one close-small, one far-large — verify that α/β weights produce the expected ranking.
- `test_select_best_frontier_respects_blacklist`: blacklisted cluster is never selected.

Tests must run standalone: `pytest test/test_frontier_detection.py` with no ROS runtime needed.

## Success Criteria

`ros2 launch autonomous_explorer full_system.launch.py` must:
1. Start robot + Gazebo + Nav2 + SLAM + frontier_explorer with zero errors.
2. Robot autonomously explores the warehouse without human input.
3. rviz2 shows frontier markers appearing and disappearing as exploration progresses.
4. Exploration progress topic publishes monotonically increasing values.
5. Robot recovers from at least one dead end or blocked path during a test run.
6. Exploration terminates cleanly when no frontiers remain.
7. All unit tests in `test/test_frontier_detection.py` pass.

## Code Quality Gates (MANDATORY)

Before declaring Phase 3 complete, verify:

- [ ] `frontier_detection.py` has zero `import rclpy` or any ROS imports.
- [ ] Every public function has a NumPy-style docstring including `Notes` for algorithmic choices.
- [ ] `mypy --strict frontier_detection.py` passes.
- [ ] No magic numbers in `frontier_explorer_node.py` — all tunables are ROS params.
- [ ] README section added explaining the Canny + cost-utility choice with a screenshot of rviz2 showing frontier markers.
- [ ] Git commit messages follow Conventional Commits format (`feat(phase3): ...`, `test(phase3): ...`).

## Non-Goals

- Do NOT implement Wavefront Frontier Detection as a fallback — commit to the Canny approach.
- Do NOT add multi-robot exploration — single robot only.
- Do NOT implement custom BT nodes for Nav2 — use the default behavior tree.
