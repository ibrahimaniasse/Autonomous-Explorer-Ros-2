#!/usr/bin/env python3
"""frontier_explorer_node.py — Autonomous frontier-based exploration node.

This node subscribes to the SLAM-generated occupancy grid, detects frontiers
at the boundary of free and unknown space, and drives the robot toward the
most promising frontier using Nav2's ``NavigateToPose`` action.

All algorithmic heavy-lifting is delegated to :mod:`frontier_detection`
(a pure-Python module with zero ROS dependencies).  This node is
deliberately kept thin: its only responsibility is I/O (subscriptions,
action clients, TF lookups, marker publishing).
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import BackUp, NavigateToPose
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

from autonomous_explorer.frontier_detection import (
    FrontierCluster,
    Pose2D,
    cluster_frontiers,
    detect_frontier_cells,
    grid_to_world,
    occupancy_grid_to_image,
    select_best_frontier,
    world_to_grid,
)


class FrontierExplorer(Node):
    """Explores unknown space by iteratively navigating to frontier goals.

    Lifecycle
    ---------
    1. Wait for the first ``/map`` message.
    2. On each map update (throttled to ``map_update_rate_hz``):
       a. Convert grid → image.
       b. Detect frontier cells.
       c. Cluster and rank frontiers.
       d. Send the best frontier as a ``NavigateToPose`` goal.
    3. Stop when no valid frontier is found for
       ``empty_iterations_before_stop`` consecutive cycles.
    """

    def __init__(self) -> None:
        """Declare parameters, set up subscribers / publishers / actions."""
        super().__init__("frontier_explorer")

        # ------------------------------------------------------------------
        # Parameters (all from exploration_params.yaml — no magic numbers)
        # ------------------------------------------------------------------
        self.declare_parameter("min_cluster_size_px", 10)
        self.declare_parameter("map_update_rate_hz", 1.0)
        self.declare_parameter("canny_low_threshold", 100)
        self.declare_parameter("canny_high_threshold", 200)
        self.declare_parameter("info_gain_weight", 1.0)
        self.declare_parameter("distance_weight", 0.5)
        self.declare_parameter("stuck_threshold_m", 0.1)
        self.declare_parameter("stuck_timeout_s", 30.0)
        self.declare_parameter("max_retries_per_frontier", 2)
        self.declare_parameter("completion_threshold", 0.95)
        self.declare_parameter("empty_iterations_before_stop", 3)
        self.declare_parameter("min_frontier_distance_m", 0.5)
        self.declare_parameter("recovery_spin_speed_rad_s", 1.0)

        self._min_cluster_size: int = self.get_parameter("min_cluster_size_px").value
        self._map_update_period: float = (
            1.0 / self.get_parameter("map_update_rate_hz").value
        )
        self._canny_low: int = self.get_parameter("canny_low_threshold").value
        self._canny_high: int = self.get_parameter("canny_high_threshold").value
        self._info_gain_weight: float = self.get_parameter("info_gain_weight").value
        self._distance_weight: float = self.get_parameter("distance_weight").value
        self._stuck_threshold: float = self.get_parameter("stuck_threshold_m").value
        self._stuck_timeout: float = self.get_parameter("stuck_timeout_s").value
        self._max_retries: int = self.get_parameter("max_retries_per_frontier").value
        self._completion_threshold: float = self.get_parameter(
            "completion_threshold"
        ).value
        self._empty_iters_limit: int = self.get_parameter(
            "empty_iterations_before_stop"
        ).value
        self._min_frontier_distance_m: float = self.get_parameter(
            "min_frontier_distance_m"
        ).value
        self._recovery_spin_speed: float = self.get_parameter(
            "recovery_spin_speed_rad_s"
        ).value

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )

        # ------------------------------------------------------------------
        # TF
        # ------------------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # Action clients
        # ------------------------------------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._backup_client = ActionClient(self, BackUp, "backup")

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._marker_pub = self.create_publisher(MarkerArray, "/frontiers", 10)
        self._progress_pub = self.create_publisher(Float32, "/exploration/progress", 10)
        self._status_pub = self.create_publisher(String, "/exploration/status", 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._last_plan_time: float = 0.0
        self._latest_map: Optional[OccupancyGrid] = None
        self._navigating: bool = False
        self._current_goal_handle = None

        # Stuck detection
        self._last_pose_time: float = time.time()
        self._last_pose_x: float = 0.0
        self._last_pose_y: float = 0.0

        # Frontier retry / blacklist
        self._blacklist: set[tuple[int, int]] = set()
        self._current_target_px: Optional[tuple[int, int]] = None
        self._retry_count: int = 0

        # Termination
        self._empty_iterations: int = 0

        # Anti-loop: track last goal centroid to detect stale re-selections
        self._last_goal_centroid_px: Optional[tuple[int, int]] = None
        self._last_goal_robot_x: float = 0.0
        self._last_goal_robot_y: float = 0.0

        # Progress timer (1 Hz)
        self._progress_timer = self.create_timer(1.0, self._publish_progress)

        self.get_logger().info("FrontierExplorer node initialised.")

    # ------------------------------------------------------------------
    # Map callback (throttled)
    # ------------------------------------------------------------------

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Store the latest map and trigger planning if rate allows."""
        self._latest_map = msg

        now = time.time()
        if now - self._last_plan_time < self._map_update_period:
            return

        if self._navigating:
            self._check_stuck()
            return

        self._last_plan_time = now
        self._plan_and_navigate()

    # ------------------------------------------------------------------
    # Core planning loop
    # ------------------------------------------------------------------

    def _plan_and_navigate(self) -> None:
        """Run the detect → cluster → select pipeline and send a goal."""
        grid_msg = self._latest_map
        if grid_msg is None:
            return

        # Robot pose in map frame
        robot_px = self._get_robot_pixel(grid_msg)
        if robot_px is None:
            self.get_logger().warn(
                "Cannot look up robot pose in map frame — skipping cycle."
            )
            return

        # Convert grid to image
        grid_data = np.array(grid_msg.data, dtype=np.int8)
        width = grid_msg.info.width
        height = grid_msg.info.height
        image = occupancy_grid_to_image(grid_data, width, height)

        # Detect frontiers
        frontier_img = detect_frontier_cells(
            image,
            canny_low_threshold=self._canny_low,
            canny_high_threshold=self._canny_high,
        )

        # Cluster
        clusters = cluster_frontiers(frontier_img, self._min_cluster_size)
        self.get_logger().debug(f"Detected {len(clusters)} frontier clusters.")

        # Publish markers for all clusters
        self._publish_frontier_markers(clusters, grid_msg)

        # Select best frontier
        min_dist_px = int(self._min_frontier_distance_m / grid_msg.info.resolution)
        best = select_best_frontier(
            clusters=clusters,
            robot_position_px=robot_px,
            info_gain_weight=self._info_gain_weight,
            distance_weight=self._distance_weight,
            blacklist=self._blacklist,
            min_frontier_distance_px=min_dist_px,
        )

        if best is None:
            self._empty_iterations += 1
            self.get_logger().info(
                f"No valid frontier ({self._empty_iterations}/"
                f"{self._empty_iters_limit})."
            )
            if self._empty_iterations >= self._empty_iters_limit:
                known_ratio = self._compute_known_ratio(grid_data)

                # Only declare completion when the map is actually explored
                if known_ratio >= self._completion_threshold:
                    self.get_logger().info(
                        f"Exploration complete ({known_ratio:.1%} known). "
                        "Shutting down."
                    )
                    self._publish_status("complete", known_ratio)
                    raise SystemExit(0)

                # Map is NOT complete — try to recover
                self.get_logger().warn(
                    f"No frontiers but only {known_ratio:.1%} known — "
                    "attempting recovery."
                )
                if self._blacklist:
                    self.get_logger().info("Recovery: clearing blacklist.")
                    self._blacklist.clear()
                else:
                    # No blacklist to clear — spin to reveal new frontiers
                    self.get_logger().info("Recovery: spinning to scan new area.")
                    self._run_spin()

                self._empty_iterations = 0
            return

        self._empty_iterations = 0

        # If targeting a new frontier, reset retry counter (with 2px tolerance)
        if (
            self._current_target_px is None
            or abs(best.centroid_px[0] - self._current_target_px[0]) > 2
            or abs(best.centroid_px[1] - self._current_target_px[1]) > 2
        ):
            self._current_target_px = best.centroid_px
            self._retry_count = 0

        # Anti-loop: skip if same centroid (±2 px) and robot hasn't moved
        if self._last_goal_centroid_px is not None:
            dx_px = abs(best.centroid_px[0] - self._last_goal_centroid_px[0])
            dy_px = abs(best.centroid_px[1] - self._last_goal_centroid_px[1])
            if dx_px <= 2 and dy_px <= 2:
                try:
                    t = self._tf_buffer.lookup_transform(
                        "map", "base_footprint", rclpy.time.Time()
                    )
                    dx_m = abs(t.transform.translation.x - self._last_goal_robot_x)
                    dy_m = abs(t.transform.translation.y - self._last_goal_robot_y)
                    if math.hypot(dx_m, dy_m) < 0.1:
                        self.get_logger().warn(
                            "Stale frontier selection detected, "
                            "waiting for map update."
                        )
                        return
                except tf2_ros.TransformException:
                    pass

        # Record this goal for stale detection
        self._last_goal_centroid_px = best.centroid_px
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            self._last_goal_robot_x = t.transform.translation.x
            self._last_goal_robot_y = t.transform.translation.y
        except tf2_ros.TransformException:
            pass

        self.get_logger().info(
            f"Navigating to frontier at px {best.centroid_px} " f"(size={best.size})."
        )
        self._send_nav_goal(best, grid_msg)

    # ------------------------------------------------------------------
    # Navigation action
    # ------------------------------------------------------------------

    def _send_nav_goal(self, cluster: FrontierCluster, grid_msg: OccupancyGrid) -> None:
        """Send a NavigateToPose goal for the cluster centroid."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        origin = Pose2D(
            x=grid_msg.info.origin.position.x,
            y=grid_msg.info.origin.position.y,
        )
        resolution = grid_msg.info.resolution
        wx, wy = grid_to_world(cluster.centroid_px, origin, resolution)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wx
        goal.pose.pose.position.y = wy
        goal.pose.pose.orientation.w = 1.0

        self._navigating = True
        self._record_pose_for_stuck()

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        """Handle the response when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._retry_count += 1
            self.get_logger().warn(
                f"Navigation goal was rejected (retry {self._retry_count}/{self._max_retries})."
            )
            self._navigating = False
            self._last_goal_centroid_px = None  # Allow immediate retry

            if self._retry_count >= self._max_retries:
                if self._current_target_px is not None:
                    self.get_logger().warn(
                        f"Blacklisting rejected frontier at {self._current_target_px}."
                    )
                    self._blacklist.add(self._current_target_px)
                self._current_target_px = None
                self._retry_count = 0
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        """Handle the final result of a NavigateToPose action."""
        result = future.result()
        status = result.status
        self._navigating = False
        self._current_goal_handle = None

        # status 4 = SUCCEEDED, 6 = ABORTED in action_msgs
        if status == 4:
            self.get_logger().info("Reached frontier goal successfully.")
            self._retry_count = 0
        else:
            self._retry_count += 1
            self.get_logger().warn(
                f"Navigation to frontier failed (status={status}, "
                f"retry {self._retry_count}/{self._max_retries})."
            )
            self._last_goal_centroid_px = None  # Allow immediate retry

            if self._retry_count >= self._max_retries:
                if self._current_target_px is not None:
                    self.get_logger().warn(
                        f"Blacklisting failed frontier at {self._current_target_px}."
                    )
                    self._blacklist.add(self._current_target_px)
                self._current_target_px = None
                self._retry_count = 0

    # ------------------------------------------------------------------
    # Stuck detection & recovery
    # ------------------------------------------------------------------

    def _record_pose_for_stuck(self) -> None:
        """Snapshot current robot pose and time for stuck monitoring."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            self._last_pose_x = t.transform.translation.x
            self._last_pose_y = t.transform.translation.y
            self._last_pose_time = time.time()
        except tf2_ros.TransformException:
            pass

    def _check_stuck(self) -> None:
        """Detect if the robot has been stuck and trigger recovery."""
        elapsed = time.time() - self._last_pose_time
        if elapsed < self._stuck_timeout:
            return

        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return

        dx = t.transform.translation.x - self._last_pose_x
        dy = t.transform.translation.y - self._last_pose_y
        distance_moved = math.hypot(dx, dy)

        if distance_moved < self._stuck_threshold:
            self.get_logger().warn(
                f"Stuck detected (moved {distance_moved:.3f} m in "
                f"{elapsed:.0f} s). Triggering recovery."
            )
            self._cancel_and_recover()
        else:
            self._record_pose_for_stuck()

    def _cancel_and_recover(self) -> None:
        """Cancel the current goal and run BackUp + Spin recovery."""
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
        self._navigating = False
        self._current_goal_handle = None

        self._run_backup()
        self._run_spin()

    def _run_backup(self) -> None:
        """Drive backward 0.3 m via the BackUp action."""
        if not self._backup_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("BackUp action server not available.")
            return
        goal = BackUp.Goal()
        goal.target.x = -0.3
        goal.speed = 0.1
        goal.time_allowance.sec = 10
        self.get_logger().info("Recovery: backing up 0.3 m.")
        self._backup_client.send_goal_async(goal)

    def _run_spin(self) -> None:
        """Execute a ~90° rotation via direct cmd_vel publishing.

        Used when no valid frontier is found: rotating in place lets the
        LiDAR scan unexplored angular sectors, expanding the SLAM map
        without requiring Nav2 behavior server or a local costmap.

        Notes
        -----
        This is a simple open-loop rotation — no feedback control.
        Accuracy is not critical: the goal is just to scan new territory,
        not to precisely reach a target heading.
        """
        angular_speed = self._recovery_spin_speed
        duration_s = 1.57 / angular_speed  # ~90° at this speed

        twist = Twist()
        twist.angular.z = angular_speed

        self.get_logger().info(
            f"Recovery: spinning at {angular_speed:.2f} rad/s for "
            f"{duration_s:.1f}s (~90°)."
        )

        # Publish at 20 Hz for the duration
        rate_hz = 20.0
        num_iterations = int(duration_s * rate_hz)
        for _ in range(num_iterations):
            self._cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate_hz)

        # Stop
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info("Recovery spin complete.")

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_robot_pixel(self, grid_msg: OccupancyGrid) -> Optional[tuple[int, int]]:
        """Get the robot's current position as a pixel in the map grid."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().debug(f"TF lookup failed: {ex}")
            return None

        origin = Pose2D(
            x=grid_msg.info.origin.position.x,
            y=grid_msg.info.origin.position.y,
        )
        robot_world = (
            t.transform.translation.x,
            t.transform.translation.y,
        )
        return world_to_grid(robot_world, origin, grid_msg.info.resolution)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def _publish_frontier_markers(
        self,
        clusters: list[FrontierCluster],
        grid_msg: OccupancyGrid,
    ) -> None:
        """Publish a MarkerArray showing all frontier clusters in rviz2."""
        marker_array = MarkerArray()

        if not clusters:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)
            self._marker_pub.publish(marker_array)
            return

        origin = Pose2D(
            x=grid_msg.info.origin.position.x,
            y=grid_msg.info.origin.position.y,
        )
        resolution = grid_msg.info.resolution
        max_size = max(c.size for c in clusters)
        stamp = self.get_clock().now().to_msg()

        for idx, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "frontiers"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            wx, wy = grid_to_world(cluster.centroid_px, origin, resolution)
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0

            scale = max(0.15, (cluster.size / max_size) * 0.6)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = 0.1

            # Color gradient: green (high utility) → red (low utility)
            ratio = cluster.size / max_size
            marker.color.r = 1.0 - ratio
            marker.color.g = ratio
            marker.color.b = 0.2
            marker.color.a = 0.85

            marker.lifetime.sec = 5

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    # ------------------------------------------------------------------
    # Progress tracking
    # ------------------------------------------------------------------

    def _compute_known_ratio(self, grid_data: np.ndarray) -> float:
        """Compute the fraction of known cells (free + occupied)."""
        total = len(grid_data)
        if total == 0:
            return 0.0
        known = int(np.count_nonzero(grid_data != -1))
        return known / total

    def _publish_progress(self) -> None:
        """Publish the exploration progress (% known) at 1 Hz."""
        if self._latest_map is None:
            return
        grid_data = np.array(self._latest_map.data, dtype=np.int8)
        ratio = self._compute_known_ratio(grid_data)

        msg = Float32()
        msg.data = float(ratio)
        self._progress_pub.publish(msg)

    def _publish_status(self, state: str, progress: float) -> None:
        """Publish a JSON status message."""
        msg = String()
        msg.data = json.dumps(
            {
                "state": state,
                "progress": round(progress, 4),
                "blacklist_size": len(self._blacklist),
            }
        )
        self._status_pub.publish(msg)


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    """Spin up the FrontierExplorer node."""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info("FrontierExplorer shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
