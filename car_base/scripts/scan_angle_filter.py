#!/usr/bin/env python3
"""Block a contiguous angular sector from a LaserScan.

Subscribes to `input_topic` and republishes to `output_topic`, setting the
ranges of points whose angle (in the lidar frame) falls inside the closed
interval [block_lower, block_upper] to +inf so they are ignored by
downstream consumers (AMCL / costmap / SLAM).

Default config blocks lidar-frame angles [-pi/2, +pi/2], which on this car
corresponds to the rear half (the lidar's 0deg points toward the rear of
the chassis, so the car-front 180deg view lives outside that range).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanAngleFilter(Node):
    def __init__(self):
        super().__init__('scan_angle_filter')
        self.declare_parameter('block_lower', -math.pi / 2.0)
        self.declare_parameter('block_upper',  math.pi / 2.0)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        self.block_lower = float(self.get_parameter('block_lower').value)
        self.block_upper = float(self.get_parameter('block_upper').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, input_topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'scan_angle_filter: in={input_topic} out={output_topic} '
            f'block=[{self.block_lower:.3f},{self.block_upper:.3f}] rad'
        )

    def cb(self, m: LaserScan):
        out = LaserScan()
        out.header = m.header
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.intensities = list(m.intensities)

        ranges = list(m.ranges)
        ang = m.angle_min
        lo = self.block_lower
        hi = self.block_upper
        inf = float('inf')
        for i in range(len(ranges)):
            if lo <= ang <= hi:
                ranges[i] = inf
            ang += m.angle_increment
        out.ranges = ranges
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanAngleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
