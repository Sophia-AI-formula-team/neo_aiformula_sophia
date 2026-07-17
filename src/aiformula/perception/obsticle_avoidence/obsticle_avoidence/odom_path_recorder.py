#!/usr/bin/env python3
import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class OdomPathRecorder(Node):
    def __init__(self):
        super().__init__('odom_path_recorder')

        # ===== Parameters =====
        self.declare_parameter('odom_topic', '/aiformula_sensing/gyro_odometry_publisher/odom')
        self.declare_parameter('output_dir', '.')
        self.declare_parameter('file_prefix', 'odom_path')
        self.declare_parameter('append_timestamp', True)

        # Record control
        self.declare_parameter('min_distance', 0.0)   # meters
        self.declare_parameter('flush_every_n', 10)

        # Plot style (defaults set to your requested style)
        self.declare_parameter('path_color', 'blue')
        self.declare_parameter('path_linewidth', 2.0)
        self.declare_parameter('background_color', 'white')
        self.declare_parameter('transparent_background', True)
        self.declare_parameter('fig_width', 8.0)
        self.declare_parameter('fig_height', 8.0)
        self.declare_parameter('dpi', 300)
        self.declare_parameter('equal_axis', True)
        self.declare_parameter('hide_axis', True)
        self.declare_parameter('padding_ratio', 0.05)

        # ===== Read parameters =====
        self.odom_topic = self.get_parameter('odom_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.append_timestamp = self.get_parameter('append_timestamp').value

        self.min_distance = float(self.get_parameter('min_distance').value)
        self.flush_every_n = int(self.get_parameter('flush_every_n').value)

        self.path_color = self.get_parameter('path_color').value
        self.path_linewidth = float(self.get_parameter('path_linewidth').value)
        self.background_color = self.get_parameter('background_color').value
        self.transparent_background = bool(self.get_parameter('transparent_background').value)
        self.fig_width = float(self.get_parameter('fig_width').value)
        self.fig_height = float(self.get_parameter('fig_height').value)
        self.dpi = int(self.get_parameter('dpi').value)
        self.equal_axis = bool(self.get_parameter('equal_axis').value)
        self.hide_axis = bool(self.get_parameter('hide_axis').value)
        self.padding_ratio = float(self.get_parameter('padding_ratio').value)

        os.makedirs(self.output_dir, exist_ok=True)

        if self.append_timestamp:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            base_name = f'{self.file_prefix}_{ts}'
        else:
            base_name = self.file_prefix

        self.csv_path = os.path.join(self.output_dir, f'{base_name}.csv')
        self.png_path = os.path.join(self.output_dir, f'{base_name}.png')

        # Trajectory points
        self.points = []
        self.last_x = None
        self.last_y = None
        self.row_count = 0

        # CSV writer
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['ros_time_sec', 'ros_time_nanosec', 'x', 'y'])

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Subscribed to: {self.odom_topic}')
        self.get_logger().info(f'CSV output: {self.csv_path}')
        self.get_logger().info(f'PNG output: {self.png_path}')

    def odom_callback(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        if self.last_x is not None and self.min_distance > 0.0:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < self.min_distance:
                return

        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)

        self.points.append((x, y))
        self.csv_writer.writerow([sec, nanosec, x, y])
        self.row_count += 1

        self.last_x = x
        self.last_y = y

        if self.row_count % max(1, self.flush_every_n) == 0:
            self.csv_file.flush()

    def save_plot(self):
        if len(self.points) == 0:
            self.get_logger().warning('No trajectory points recorded. Skip plotting.')
            return

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]

        fig, ax = plt.subplots(figsize=(self.fig_width, self.fig_height))
        fig.patch.set_facecolor(self.background_color)
        ax.set_facecolor(self.background_color)

        ax.plot(xs, ys, color=self.path_color, linewidth=self.path_linewidth)

        if self.equal_axis:
            ax.set_aspect('equal', adjustable='box')

        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        x_range = x_max - x_min
        y_range = y_max - y_min

        if x_range == 0:
            x_range = 1.0
        if y_range == 0:
            y_range = 1.0

        x_pad = x_range * self.padding_ratio
        y_pad = y_range * self.padding_ratio

        ax.set_xlim(x_min - x_pad, x_max + x_pad)
        ax.set_ylim(y_min - y_pad, y_max + y_pad)

        if self.hide_axis:
            ax.axis('off')
        else:
            ax.set_xlabel('x')
            ax.set_ylabel('y')

        plt.tight_layout()
        plt.savefig(
            self.png_path,
            dpi=self.dpi,
            transparent=self.transparent_background,
            bbox_inches='tight',
            pad_inches=0.02
        )
        plt.close(fig)

        self.get_logger().info(f'Plot saved: {self.png_path}')

    def close_and_export(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(f'CSV saved: {self.csv_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to close CSV: {e}')

        try:
            self.save_plot()
        except Exception as e:
            self.get_logger().error(f'Failed to save plot: {e}')

    def destroy_node(self):
        self.close_and_export()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping odom path recorder...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()