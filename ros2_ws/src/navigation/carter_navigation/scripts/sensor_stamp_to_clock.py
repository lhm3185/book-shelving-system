#!/usr/bin/env python3

"""Publish ROS simulation time from an Isaac Sim sensor timestamp."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


class SensorStampToClock(Node):
    def __init__(self):
        super().__init__("sensor_stamp_to_clock")

        self.declare_parameter("source_topic", "/front_3d_lidar/lidar_points")
        self.declare_parameter("clock_topic", "/clock")
        self.source_topic = self.get_parameter("source_topic").value
        self.clock_topic = self.get_parameter("clock_topic").value

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.clock_pub = self.create_publisher(Clock, self.clock_topic, clock_qos)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.source_topic, self._cloud_callback, sensor_qos
        )
        self.external_clock = False
        self.last_stamp_ns = -1
        self.has_published = False
        self.create_timer(1.0, self._check_for_external_clock)

    def _check_for_external_clock(self):
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        external_clock = any(
            info.node_name != own_name or info.node_namespace != own_namespace
            for info in self.get_publishers_info_by_topic(self.clock_topic)
        )
        if external_clock != self.external_clock:
            self.external_clock = external_clock
            if external_clock:
                self.get_logger().info(
                    "External /clock publisher detected; sensor clock fallback paused"
                )
            else:
                self.get_logger().info(
                    "External /clock publisher disappeared; sensor clock fallback resumed"
                )

    def _cloud_callback(self, cloud: PointCloud2):
        if self.external_clock:
            return

        stamp_ns = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if stamp_ns <= 0 or stamp_ns <= self.last_stamp_ns:
            return

        clock = Clock()
        clock.clock.sec = cloud.header.stamp.sec
        clock.clock.nanosec = cloud.header.stamp.nanosec
        self.clock_pub.publish(clock)
        self.last_stamp_ns = stamp_ns

        if not self.has_published:
            self.get_logger().info(
                f"Publishing {self.clock_topic} from {self.source_topic} timestamps"
            )
            self.has_published = True


def main(args=None):
    rclpy.init(args=args)
    node = SensorStampToClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
