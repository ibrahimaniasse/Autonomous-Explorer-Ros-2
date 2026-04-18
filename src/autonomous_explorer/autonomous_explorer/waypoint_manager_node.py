#!/usr/bin/env python3
"""waypoint_manager_node.py — Sequential waypoint follower.

Subscribes to a ``PoseArray`` on ``/waypoints``, then sends each pose as
a ``NavigateToPose`` goal one at a time.  On failure the goal is retried
once; if the retry also fails the waypoint is skipped and the next one
is attempted.

This node is intentionally simple supporting infrastructure — the
autonomous exploration showcase lives in ``frontier_explorer_node.py``.
"""

from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class WaypointManager(Node):
    """Send a queue of waypoints to Nav2 sequentially.

    Parameters
    ----------
    ~/max_retries : int
        Maximum number of retries per waypoint before skipping (default 1).
    """

    def __init__(self) -> None:
        """Initialise subscribers, publishers and the Nav2 action client."""
        super().__init__("waypoint_manager")

        self.declare_parameter("max_retries", 1)
        self._max_retries: int = self.get_parameter("max_retries").value

        # Waypoint queue
        self._waypoints: list[PoseStamped] = []
        self._current_index: int = 0
        self._retry_count: int = 0
        self._active: bool = False

        # Subscriber — one-shot queue reload on each message
        self._wp_sub = self.create_subscription(
            PoseArray, "/waypoints", self._waypoints_callback, 10
        )

        # Action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Status publisher
        self._status_pub = self.create_publisher(String, "/waypoint_status", 10)

        self.get_logger().info("WaypointManager initialised — waiting for /waypoints.")

    # ------------------------------------------------------------------
    # Waypoint reception
    # ------------------------------------------------------------------

    def _waypoints_callback(self, msg: PoseArray) -> None:
        """Load a new waypoint queue from a PoseArray message."""
        self._waypoints = []
        for pose in msg.poses:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = pose
            self._waypoints.append(ps)

        self._current_index = 0
        self._retry_count = 0
        self._active = True

        self.get_logger().info(
            f"Received {len(self._waypoints)} waypoints. Starting sequence."
        )
        self._publish_status("received")
        self._send_next_goal()

    # ------------------------------------------------------------------
    # Goal management
    # ------------------------------------------------------------------

    def _send_next_goal(self) -> None:
        """Send the next waypoint in the queue to Nav2."""
        if self._current_index >= len(self._waypoints):
            self.get_logger().info("All waypoints completed.")
            self._active = False
            self._publish_status("all_done")
            return

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        wp = self._waypoints[self._current_index]
        wp.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = wp

        self.get_logger().info(
            f"Sending waypoint {self._current_index + 1}/"
            f"{len(self._waypoints)} "
            f"({wp.pose.position.x:.2f}, {wp.pose.position.y:.2f})."
        )
        self._publish_status("navigating")

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        """Handle accept / reject from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Waypoint {self._current_index + 1} rejected by Nav2."
            )
            self._advance()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        """Handle the final result of a NavigateToPose action."""
        result = future.result()
        status = result.status

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"Waypoint {self._current_index + 1} reached.")
            self._retry_count = 0
            self._advance()
        else:
            self._retry_count += 1
            self.get_logger().warn(
                f"Waypoint {self._current_index + 1} failed "
                f"(status={status}, retry {self._retry_count}/"
                f"{self._max_retries})."
            )
            if self._retry_count > self._max_retries:
                self.get_logger().warn(
                    f"Skipping waypoint {self._current_index + 1} after "
                    f"{self._max_retries} retries."
                )
                self._retry_count = 0
                self._advance()
            else:
                self._publish_status("retrying")
                self._send_next_goal()

    def _advance(self) -> None:
        """Move to the next waypoint in the queue."""
        self._current_index += 1
        self._retry_count = 0
        self._send_next_goal()

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_status(self, status: str) -> None:
        """Publish waypoint progress as a JSON string."""
        msg = String()
        msg.data = json.dumps(
            {
                "current_index": self._current_index,
                "total": len(self._waypoints),
                "status": status,
            }
        )
        self._status_pub.publish(msg)
        self.get_logger().debug(f"Status: {msg.data}")


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------


def main(args: list[str] | None = None) -> None:
    """Spin up the WaypointManager node."""
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("WaypointManager shutting down.")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
