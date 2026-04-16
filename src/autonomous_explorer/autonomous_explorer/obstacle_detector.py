#!/usr/bin/env python3
"""obstacle_detector.py

Node for fusing LiDAR and Camera data to detect obstacles in a defined danger zone.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np
import math


class ObstacleDetector(Node):
    """Subscribes to /scan and /camera/image_raw to detect obstacles.

    Publishes a MarkerArray to /obstacles.
    Uses configurable weights to fuse LiDAR distance and Camera edges.
    """

    def __init__(self) -> None:
        """Initializes the ObstacleDetector node and its parameters."""
        super().__init__('obstacle_detector')

        # Declare parameters
        self.declare_parameter('danger_zone', 0.5)
        self.declare_parameter('w_lidar', 0.6)
        self.declare_parameter('w_cam', 0.4)
        self.declare_parameter('confidence_threshold', 0.5)

        # Get parameters
        self.danger_zone = self.get_parameter('danger_zone').value
        self.w_lidar = self.get_parameter('w_lidar').value
        self.w_cam = self.get_parameter('w_cam').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.img_callback,
            10
        )

        # Publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacles',
            10
        )

        # Tools
        self.bridge = CvBridge()

        # State vars
        self.latest_cam_confidence = 0.0
        self.latest_scan: LaserScan = None

        # Timer for fusing and publishing (10 Hz)
        self.timer = self.create_timer(0.1, self.fusion_callback)

        self.get_logger().info("ObstacleDetector initialized.")

    def img_callback(self, msg: Image) -> None:
        """Process the image stream to estimate obstacle presence.

        Looks for edges in the lower half of the image.

        Args:
            msg: The image message received from the camera.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Focus on the lower half where close obstacles typically appear
        h, w = cv_image.shape[:2]
        lower_half = cv_image[int(h / 2):, :]

        # Convert to grayscale and apply Gaussian blur
        gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Calculate confidence based on the density of edges
        edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])

        # Normalize edge density conceptually (tune range based on warehouse environment)
        # Assuming density of 0.05 is highly obscured 
        self.latest_cam_confidence = min(1.0, edge_density / 0.05)

    def scan_callback(self, msg: LaserScan) -> None:
        """Store the latest scan data for fusion.

        Args:
            msg: The laser scan message received from the LiDAR.
        """
        self.latest_scan = msg

    def fusion_callback(self) -> None:
        """Periodically evaluate both sensor streams, compute confidence.

        Publishes markers if an obstacle is detected based on fused confidence.
        """
        if self.latest_scan is None:
            return

        cam_conf = self.latest_cam_confidence
        scan = self.latest_scan

        obstacles = []

        # Analyze scan data in the frontal FOV to find clusters
        # The camera covers roughly [-0.523, 0.523] rad
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r > self.danger_zone or r < scan.range_min:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # Only consider LiDAR points that match the camera's FOV roughly
            if -0.6 < angle < 0.6:
                # Calculate confidence
                lidar_conf = 1.0 - (r / self.danger_zone)

                total_conf = self.w_lidar * lidar_conf + self.w_cam * cam_conf

                if total_conf >= self.conf_thresh:
                    obstacles.append((r, angle, total_conf))

        self.publish_markers(obstacles, scan.header.frame_id)

    def publish_markers(self, obstacles: list, frame_id: str) -> None:
        """Converts the list of obstacles to MarkerArray and publishes.

        Args:
            obstacles: A list of tuples containing (distance, angle, total_confidence).
            frame_id: The coordinate frame ID for the markers.
        """
        marker_array = MarkerArray()

        # Simplify by grouping close obstacles to avoid spamming markers
        if not obstacles:
            # Publish empty array or just return. Returning is fine, but we might want to clear old markers
            # Let's publish a delete-all marker
            del_marker = Marker()
            del_marker.action = Marker.DELETEALL
            marker_array.markers.append(del_marker)
            self.marker_pub.publish(marker_array)
            return

        # Find the single closest obstacle point in the danger zone for simplicity
        closest = min(obstacles, key=lambda x: x[0])
        r, angle, conf = closest

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'obstacles'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Coordinates
        marker.pose.position.x = r * math.cos(angle)
        marker.pose.position.y = r * math.sin(angle)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color (Red = highly confident, Orange = moderate)
        marker.color.r = 1.0
        marker.color.g = max(0.0, 1.0 - conf)
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args: list | None = None) -> None:
    """Main entry point for the node.

    Args:
        args: Command-line arguments.
    """
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
