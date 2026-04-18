import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from autonomous_explorer.obstacle_detector import ObstacleDetector


@pytest.fixture
def detector_node():
    rclpy.init()
    node = ObstacleDetector()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def test_initialization(detector_node):
    """Test that the node initializes and sets default parameters correctly."""
    assert detector_node is not None
    assert detector_node.danger_zone == 0.5
    assert detector_node.w_lidar == 0.6
    assert detector_node.w_cam == 0.4
    assert detector_node.conf_thresh == 0.5


def test_scan_callback(detector_node):
    """Test that the scan callback stores the scan data correctly."""
    msg = LaserScan()
    msg.header.frame_id = "base_link"
    msg.ranges = [1.0, 0.4, 2.0]

    detector_node.scan_callback(msg)

    assert detector_node.latest_scan is not None
    assert len(detector_node.latest_scan.ranges) == 3
    assert detector_node.latest_scan.ranges[1] == pytest.approx(0.4)


def test_fusion_logic_no_obstacle(detector_node):
    """Test the fusion logic when no obstacles are within the danger zone."""
    msg = LaserScan()
    msg.header.frame_id = "base_link"
    msg.angle_min = -0.5
    msg.angle_increment = 0.5
    msg.range_min = 0.1
    msg.range_max = 12.0
    msg.ranges = [1.0, 1.5, 2.0]  # All > danger_zone (0.5)

    detector_node.latest_scan = msg
    detector_node.latest_cam_confidence = 0.0

    # We will mock the publisher to see if anything is published
    published_markers = []

    def mock_publish(array):
        published_markers.append(array)

    detector_node.marker_pub.publish = mock_publish

    detector_node.fusion_callback()

    # Should publish a DELETEALL marker if no obstacles
    assert len(published_markers) == 1
    assert len(published_markers[0].markers) == 1
    assert published_markers[0].markers[0].action == 3  # Marker.DELETEALL


def test_fusion_logic_with_obstacle(detector_node):
    """Test the fusion logic when an obstacle is clearly inside the danger zone."""
    msg = LaserScan()
    msg.header.frame_id = "base_link"
    msg.angle_min = -0.5
    msg.angle_increment = 0.5
    msg.range_min = 0.1
    msg.range_max = 12.0
    msg.ranges = [
        1.0,
        0.2,
        2.0,
    ]  # 0.2 < 0.5 (danger zone), confidence = 1 - 0.2/0.5 = 0.6

    detector_node.latest_scan = msg
    detector_node.latest_cam_confidence = 1.0  # High camera confidence

    # Total conf = (0.6 * 0.6) + (0.4 * 1.0) = 0.36 + 0.40 = 0.76 >= 0.5

    published_markers = []

    def mock_publish(array):
        published_markers.append(array)

    detector_node.marker_pub.publish = mock_publish

    detector_node.fusion_callback()

    assert len(published_markers) == 1
    assert len(published_markers[0].markers) == 1
    marker = published_markers[0].markers[0]
    assert marker.action == 0  # Marker.ADD
    assert marker.pose.position.x == pytest.approx(
        0.2
    )  # angle for ranges[1] is 0.0 because -0.5 + 1*0.5 = 0.0
    assert marker.pose.position.y == pytest.approx(0.0)
