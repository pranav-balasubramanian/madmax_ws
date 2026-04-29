#!/usr/bin/env python3
"""Mapping supervisor: torque-boost startup, ramp, gated SLAM, lap detection, map save.

State machine
-------------
WAITING_FOR_START → operator presses Y on the joystick → STARTUP_BOOST
STARTUP_BOOST     → command boost_speed for boost_duration seconds
RAMP_DOWN         → linearly interpolate boost_speed → mapping_speed over ramp_duration
ENABLE_SLAM       → open the scan gate (forward /scan to /scan_for_slam)
MAPPING_ACTIVE    → command mapping_speed; capture start pose; watch /odom for return
LOOP_DETECTED     → command 0 m/s
STOP_AND_SAVE     → close scan gate, write /map to disk via TRANSIENT_LOCAL subscription
DONE              → idle

Why a scan gate instead of starting slam_toolbox late: launching slam_toolbox at runtime
adds lifecycle complexity, and scan-throttling makes the "no map data during boost"
guarantee local to this node and trivially testable.
"""
import math
import os
import struct
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Float32


class State(Enum):
    WAITING_FOR_START = 0
    STARTUP_BOOST = 1
    RAMP_DOWN = 2
    ENABLE_SLAM = 3
    MAPPING_ACTIVE = 4
    LOOP_DETECTED = 5
    STOP_AND_SAVE = 6
    DONE = 7


class MappingSupervisor(Node):
    Y_BUTTON_INDEX = 3  # matches wall_follower deadman

    def __init__(self):
        super().__init__('mapping_supervisor_node')

        self.declare_parameter('boost_speed', 2.2)
        self.declare_parameter('mapping_speed', 1.5)
        self.declare_parameter('boost_duration', 0.75)
        self.declare_parameter('ramp_duration', 1.5)
        self.declare_parameter('min_lap_time', 20.0)
        self.declare_parameter('loop_distance_threshold', 0.5)
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic_in', '/scan')
        self.declare_parameter('scan_topic_out', '/scan_for_slam')
        self.declare_parameter('target_speed_topic', '/wall_follower/target_speed')
        self.declare_parameter('map_output_dir', '~/f1tenth_mapper')
        self.declare_parameter('map_name', 'track_map')

        self.boost_speed = float(self.get_parameter('boost_speed').value)
        self.mapping_speed = float(self.get_parameter('mapping_speed').value)
        self.boost_duration = float(self.get_parameter('boost_duration').value)
        self.ramp_duration = float(self.get_parameter('ramp_duration').value)
        self.min_lap_time = float(self.get_parameter('min_lap_time').value)
        self.loop_distance_threshold = float(self.get_parameter('loop_distance_threshold').value)
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic_in = self.get_parameter('scan_topic_in').value
        self.scan_topic_out = self.get_parameter('scan_topic_out').value
        self.target_speed_topic = self.get_parameter('target_speed_topic').value
        self.map_output_dir = os.path.expanduser(self.get_parameter('map_output_dir').value)
        self.map_name = self.get_parameter('map_name').value

        self.state = State.WAITING_FOR_START
        self.state_entered = self.get_clock().now()
        self.start_pose = None
        self.mapping_started_time = None
        self.last_y_pressed = False
        self.scan_gate_open = False
        self.map_save_requested = False
        self.map_saved = False
        self._map_save_sub = None

        self.target_pub = self.create_publisher(Float32, self.target_speed_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic_out, 10)

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic_in, self.scan_cb, 10)

        self.create_timer(0.05, self.tick)  # 20 Hz state machine

        self.get_logger().info(
            'MappingSupervisor ready. Hold Y on the joystick to engage the deadman, '
            'then tap-and-hold Y to start: BOOST → RAMP → SLAM → LAP → SAVE.'
        )

    def _enter(self, new_state):
        self.get_logger().info(f'STATE: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_entered = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds * 1e-9

    def _publish_target(self, speed):
        msg = Float32()
        msg.data = float(speed)
        self.target_pub.publish(msg)

    def joy_cb(self, msg):
        if len(msg.buttons) <= self.Y_BUTTON_INDEX:
            return
        y_pressed = msg.buttons[self.Y_BUTTON_INDEX] == 1
        # Trigger the sequence on the rising edge of Y while waiting.
        if y_pressed and not self.last_y_pressed and self.state == State.WAITING_FOR_START:
            self.get_logger().info(
                f'Y pressed -> STARTUP_BOOST at {self.boost_speed:.2f} m/s '
                f'for {self.boost_duration:.2f}s'
            )
            self._enter(State.STARTUP_BOOST)
        self.last_y_pressed = y_pressed

    def scan_cb(self, msg):
        # Pass-through gate. slam_toolbox is configured to consume scan_topic_out,
        # so it sees nothing until ENABLE_SLAM opens the gate.
        if self.scan_gate_open:
            self.scan_pub.publish(msg)

    def odom_cb(self, msg):
        if self.state != State.MAPPING_ACTIVE:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_pose is None:
            self.start_pose = (x, y)
            self.mapping_started_time = self.get_clock().now()
            self.get_logger().info(
                f'Captured start pose ({x:.2f}, {y:.2f}). '
                f'Lap detection arms after {self.min_lap_time:.1f}s.'
            )
            return

        elapsed = (self.get_clock().now() - self.mapping_started_time).nanoseconds * 1e-9
        if elapsed < self.min_lap_time:
            return

        dist = math.hypot(x - self.start_pose[0], y - self.start_pose[1])
        if dist < self.loop_distance_threshold:
            self.get_logger().info(
                f'Loop closed: returned within {dist:.2f}m of start after {elapsed:.1f}s.'
            )
            self._enter(State.LOOP_DETECTED)

    def tick(self):
        if self.state == State.WAITING_FOR_START:
            self._publish_target(0.0)

        elif self.state == State.STARTUP_BOOST:
            self._publish_target(self.boost_speed)
            if self._elapsed() >= self.boost_duration:
                self._enter(State.RAMP_DOWN)

        elif self.state == State.RAMP_DOWN:
            t = min(1.0, self._elapsed() / max(1e-3, self.ramp_duration))
            speed = self.boost_speed + (self.mapping_speed - self.boost_speed) * t
            self._publish_target(speed)
            if t >= 1.0:
                self._enter(State.ENABLE_SLAM)

        elif self.state == State.ENABLE_SLAM:
            self._publish_target(self.mapping_speed)
            self.scan_gate_open = True
            self.get_logger().info(
                f'Scan gate OPEN: forwarding {self.scan_topic_in} -> {self.scan_topic_out}.'
            )
            self._enter(State.MAPPING_ACTIVE)

        elif self.state == State.MAPPING_ACTIVE:
            self._publish_target(self.mapping_speed)

        elif self.state == State.LOOP_DETECTED:
            self._publish_target(0.0)
            self._enter(State.STOP_AND_SAVE)

        elif self.state == State.STOP_AND_SAVE:
            self._publish_target(0.0)
            if not self.map_save_requested:
                self._request_map_save()
                self.map_save_requested = True
            # Close the gate so slam_toolbox stops integrating any further scans.
            self.scan_gate_open = False
            if self.map_saved:
                self._enter(State.DONE)

        elif self.state == State.DONE:
            self._publish_target(0.0)

    def _request_map_save(self):
        try:
            os.makedirs(self.map_output_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f'Cannot create {self.map_output_dir}: {e}')
            return

        out_path = os.path.join(self.map_output_dir, self.map_name)
        self.get_logger().info(
            f'Subscribing to /map (TRANSIENT_LOCAL) to write {out_path}.{{pgm,yaml}}'
        )

        # slam_toolbox publishes /map with TRANSIENT_LOCAL durability; the standard
        # map_saver_cli uses VOLATILE and never receives it. Match QoS exactly.
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._map_save_sub = self.create_subscription(
            OccupancyGrid, '/map', lambda m: self._write_map(m, out_path), qos
        )

    def _write_map(self, msg, out_path):
        if self.map_saved:
            return

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        pgm_path = out_path + '.pgm'
        yaml_path = out_path + '.yaml'

        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            # OccupancyGrid origin is bottom-left; PGM origin is top-left.
            for row in range(h - 1, -1, -1):
                for col in range(w):
                    val = msg.data[row * w + col]
                    if val == -1:
                        pixel = 205   # unknown
                    elif val == 0:
                        pixel = 254   # free
                    else:
                        pixel = 0     # occupied
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
        self.map_saved = True


def main(args=None):
    rclpy.init(args=args)
    node = MappingSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop = Float32()
            stop.data = 0.0
            node.target_pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
