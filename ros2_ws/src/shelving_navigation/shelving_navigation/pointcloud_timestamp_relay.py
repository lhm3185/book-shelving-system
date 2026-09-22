"""Relay Isaac LiDAR data with the active ROS simulation timestamp."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudTimestampRelay(Node):
    """Put Isaac point clouds in the same time domain as Nav2 and TF."""

    def __init__(self) -> None:
        super().__init__('pointcloud_timestamp_relay')

        self.declare_parameter('input_topic', '/lidar/points_raw')
        self.declare_parameter('output_topic', '/lidar/points_nav')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Restamping point clouds from {input_topic} to {output_topic}.'
        )

    def _on_pointcloud(self, message: PointCloud2) -> None:
        # The Isaac RTX LiDAR timestamp can continue from a different timeline
        # after simulation restarts.  Nav2 requires sensor data and TF to use
        # the /clock timeline, so replace only the header timestamp.
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudTimestampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
