#!/usr/bin/env python3
"""Terminal-based map viewer for F1Tenth localization.

Loads a PGM/YAML map at startup, subscribes to a pose topic, and renders
the map to the terminal as ANSI half-blocks with a red glyph at the
current car pose. No window forwarding or RViz required.

Layout:
  ─── F1Tenth Terminal Localization ────────────────────────
  (map drawn as ▀ half-blocks, walls black, free light, unknown grey,
   car as a directional arrow in bright red)
  pose:  x=+1.23 m   y=-0.45 m   yaw=+90.0°
  age:   0.12 s   map: 480x320 @ 0.050 m/cell
"""
import math
import os
import shutil
import sys
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


CLEAR_HOME = '\033[2J\033[H'
HIDE_CURSOR = '\033[?25l'
SHOW_CURSOR = '\033[?25h'
RESET = '\033[0m'


def color_pair(fg, bg):
    return f'\033[38;5;{fg};48;5;{bg}m'


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


# 8-direction heading glyphs starting at east (yaw=0), going CCW.
HEADING_GLYPHS = ['→', '↗', '↑', '↖', '←', '↙', '↓', '↘']


def heading_glyph(yaw):
    sector = int(round(yaw / (math.pi / 4.0))) % 8
    return HEADING_GLYPHS[sector]


class TerminalMapViewer(Node):
    def __init__(self):
        super().__init__('terminal_map_viewer')

        self.declare_parameter('map_yaml', os.path.expanduser('~/f1tenth_mapper/track_map.yaml'))
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('refresh_hz', 5.0)
        self.declare_parameter('reserved_top_lines', 1)
        self.declare_parameter('reserved_bottom_lines', 4)

        map_yaml = os.path.expanduser(self.get_parameter('map_yaml').value)
        pose_topic = self.get_parameter('pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        refresh_hz = float(self.get_parameter('refresh_hz').value)
        self.reserved_top = int(self.get_parameter('reserved_top_lines').value)
        self.reserved_bottom = int(self.get_parameter('reserved_bottom_lines').value)

        self.map_grid, self.resolution, self.origin = self._load_map(map_yaml)
        self.h, self.w = self.map_grid.shape

        self.pose_x = None
        self.pose_y = None
        self.pose_yaw = 0.0
        self.last_pose_time = None

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.last_odom_time = None

        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self.pose_cb, 10
        )
        self.create_subscription(
            Odometry, odom_topic, self.odom_cb, 10
        )
        self.create_timer(1.0 / refresh_hz, self.render)

        sys.stdout.write(HIDE_CURSOR)
        sys.stdout.flush()

        self.get_logger().info(
            f'Loaded map {self.w}x{self.h} @ {self.resolution:.3f} m/cell, '
            f'origin=({self.origin[0]:.2f}, {self.origin[1]:.2f}). '
            f'Subscribed to {pose_topic} at {refresh_hz} Hz redraw.'
        )

    def _load_map(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        image_path = meta['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        resolution = float(meta['resolution'])
        origin = meta['origin']
        grid = self._read_pgm(image_path)
        return grid, resolution, origin

    def _read_pgm(self, path):
        with open(path, 'rb') as f:
            magic = f.readline().strip()
            if magic != b'P5':
                raise RuntimeError(f'Expected P5 PGM, got {magic!r}')
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            w, h = map(int, line.split())
            _maxval = int(f.readline())
            data = f.read(w * h)
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w)

    def pose_cb(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.last_pose_time = time.time()

    def odom_cb(self, msg):
        # Twist is in body frame (REP-105): linear.x is forward velocity.
        self.speed = float(msg.twist.twist.linear.x)
        self.yaw_rate = float(msg.twist.twist.angular.z)
        self.last_odom_time = time.time()

    def render(self):
        term = shutil.get_terminal_size((100, 30))
        usable_rows = max(4, term.lines - self.reserved_top - self.reserved_bottom)
        usable_cols = max(10, term.columns)

        # Each terminal row holds two map pixels (▀ = top half fg, bottom half bg),
        # which roughly squares the effective pixel since terminal cells are ~2:1.
        target_pixel_rows = usable_rows * 2
        target_pixel_cols = usable_cols

        scale = max(self.w / target_pixel_cols, self.h / target_pixel_rows, 1e-6)
        out_w = max(1, int(self.w / scale))
        out_h = max(2, int(self.h / scale))
        if out_h % 2 == 1:
            out_h -= 1

        ds = self._block_min_downsample(self.map_grid, out_h, out_w)

        car_col = car_row = None
        if self.pose_x is not None:
            px = (self.pose_x - self.origin[0]) / self.resolution
            py_top = (self.h - 1) - (self.pose_y - self.origin[1]) / self.resolution
            cc = int(px / scale)
            cr = int(py_top / scale)
            if 0 <= cc < out_w and 0 <= cr < out_h:
                car_col, car_row = cc, cr

        lines = []
        if self.reserved_top > 0:
            lines.append(self._top_line(usable_cols))

        car_glyph = heading_glyph(self.pose_yaw) if self.pose_x is not None else '?'
        for tr in range(out_h // 2):
            chars = []
            top_y = 2 * tr
            bot_y = 2 * tr + 1
            on_car_row = car_row is not None and (top_y == car_row or bot_y == car_row)
            for c in range(out_w):
                if on_car_row and c == car_col:
                    chars.append(f'\033[1;91m{car_glyph}{RESET}')
                else:
                    fg = self._cell_color(ds[top_y, c])
                    bg = self._cell_color(ds[bot_y, c])
                    chars.append(f'{color_pair(fg, bg)}▀{RESET}')
            lines.append(''.join(chars))

        lines.extend(self._bottom_lines(usable_cols))

        sys.stdout.write(CLEAR_HOME + '\n'.join(lines))
        sys.stdout.flush()

    @staticmethod
    def _block_min_downsample(grid, out_h, out_w):
        # Min-pool so dark walls survive aggressive downsampling.
        h, w = grid.shape
        sy = h / out_h
        sx = w / out_w
        out = np.empty((out_h, out_w), dtype=grid.dtype)
        for r in range(out_h):
            r0 = int(r * sy)
            r1 = max(r0 + 1, int((r + 1) * sy))
            for c in range(out_w):
                c0 = int(c * sx)
                c1 = max(c0 + 1, int((c + 1) * sx))
                out[r, c] = grid[r0:r1, c0:c1].min()
        return out

    @staticmethod
    def _cell_color(val):
        # save_pgm.py convention: 0=occupied (black), 254=free (near-white), 205=unknown
        if val < 50:
            return 232   # near-black
        if val > 240:
            return 252   # light grey
        return 238       # dim grey

    @staticmethod
    def _top_line(width):
        title = ' F1Tenth Terminal Localization '
        return f'\033[1;36m{title.center(width, "─")}{RESET}'

    def _bottom_lines(self, width):
        if self.pose_x is None:
            l_pose = '  pose:  (waiting for pose...)'
            l_speed = self._speed_line()
            l_meta = f'  map:   {self.w}x{self.h} @ {self.resolution:.3f} m/cell'
            return ['', l_pose.ljust(width), l_speed.ljust(width), l_meta.ljust(width)]
        age = time.time() - (self.last_pose_time or 0.0)
        l_pose = (f'  pose:  x={self.pose_x:+6.2f} m   y={self.pose_y:+6.2f} m   '
                  f'yaw={math.degrees(self.pose_yaw):+6.1f}°   age={age:4.2f}s')
        l_speed = self._speed_line()
        l_meta = f'  map:   {self.w}x{self.h} @ {self.resolution:.3f} m/cell'
        return ['', l_pose.ljust(width), l_speed.ljust(width), l_meta.ljust(width)]

    def _speed_line(self):
        if self.last_odom_time is None:
            return '  speed: (waiting for /odom...)'
        age = time.time() - self.last_odom_time
        # Color the speed value: green when moving forward, red when reversing.
        color = '\033[1;92m' if self.speed >= 0 else '\033[1;91m'
        return (f'  speed: {color}{self.speed:+5.2f} m/s{RESET}   '
                f'yaw_rate={math.degrees(self.yaw_rate):+6.1f} °/s   age={age:4.2f}s')


def main(args=None):
    rclpy.init(args=args)
    node = TerminalMapViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR + '\n')
        sys.stdout.flush()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
