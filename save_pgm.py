#!/usr/bin/env python3
"""Subscribe to /map with Transient Local QoS and write .pgm + .yaml.

slam_toolbox publishes /map with TRANSIENT_LOCAL durability. The standard
map_saver_cli subscribes with VOLATILE and therefore never receives the
message. This script constructs the subscription with the matching QoS policy.
"""
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapSaver(Node):
    def __init__(self, output_path):
        super().__init__('map_pgm_saver')
        self.output_path = output_path
        self.done = False

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        self.get_logger().info('Waiting for /map ...')

    def _cb(self, msg):
        if self.done:
            return
        self.done = True

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        pgm_path = self.output_path + '.pgm'
        yaml_path = self.output_path + '.yaml'

        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            # ROS OccupancyGrid origin is bottom-left; PGM origin is top-left
            for row in range(h - 1, -1, -1):
                for col in range(w):
                    val = msg.data[row * w + col]
                    if val == -1:
                        pixel = 205   # unknown → grey
                    elif val == 0:
                        pixel = 254   # free → near-white
                    else:
                        pixel = 0     # occupied → black
                    f.write(struct.pack('B', pixel))

        with open(yaml_path, 'w') as f:
            f.write(f'image: {pgm_path}\n')
            f.write(f'resolution: {res}\n')
            f.write(f'origin: [{ox}, {oy}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

        self.get_logger().info(f'Saved {pgm_path}')
        self.get_logger().info(f'Saved {yaml_path}')
        rclpy.shutdown()


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else '/tmp/map'
    rclpy.init()
    rclpy.spin(MapSaver(output))


if __name__ == '__main__':
    main()
