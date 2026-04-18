#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from tf2_msgs.msg import TFMessage


class TfStaticRepublisher(Node):
    def __init__(self):
        super().__init__("tf_static_republisher")

        # Subscriber QoS: Volatile (to match Gazebo bridge default)
        sub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Publisher QoS: Transient Local (ROS 2 standard for tf_static)
        pub_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(TFMessage, "/tf_static", pub_qos)

        self.subscriber = self.create_subscription(
            TFMessage, "/tf_static_bridge", self.listener_callback, sub_qos
        )
        self.get_logger().info(
            "Bridging /tf_static_bridge (VOLATILE) to /tf_static (TRANSIENT_LOCAL)"
        )

    def listener_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfStaticRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
