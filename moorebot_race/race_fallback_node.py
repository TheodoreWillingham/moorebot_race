#!/usr/bin/env python3
"""Open-loop ROS 2 Humble fallback race controller for Moorebot Scout.

Works with mqtt_ros2_bridge.py by auto-discovering /<robot>/cmd_vel
(or using an explicit robot_name parameter) and executing a 3-leg,
2-turn rectangular U-path from either the left or right entrance.
"""

import math
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

CMD_VEL_PATTERN = re.compile(r'^/([A-Za-z][A-Za-z0-9_]*)/cmd_vel$')


class RaceFallbackNode(Node):
    def __init__(self):
        super().__init__('moorebot_race_fallback')

        # --- Configurable Parameters ---
        self.declare_parameter('robot_name', '')          # Empty = auto-discover
        self.declare_parameter('start_side', 'left')      # 'left' or 'right'

        # Track geometry (meters)
        self.declare_parameter('long_distance', 1.5)      # 1st and 3rd straights
        self.declare_parameter('short_distance', 1.0)     # 2nd straight

        # Motion speeds (ROS Twist units)
        self.declare_parameter('linear_speed', 0.25)      # Forward speed
        self.declare_parameter('angular_speed', 0.8)      # Turn speed magnitude

        # Drift compensation applied during straight segments
        # Negative angular.z pulls right to counteract left drift
        self.declare_parameter('drift_angular_trim', -0.05)
        self.declare_parameter('drift_strafe_trim', 0.0)  # Optional linear.y trim

        # Calibration multipliers
        self.declare_parameter('distance_scale', 1.0)
        # Default 1.5 compensates for mqtt_ros2_bridge.py dividing angular.z by 1.5
        self.declare_parameter('turn_scale', 1.5)

        # Timing & safety
        self.declare_parameter('pause_duration', 0.4)     # Settle pause between moves
        self.declare_parameter('start_delay', 2.0)        # Wait after topic discovery
        self.declare_parameter('publish_rate', 20.0)      # Hz (must be >> 2 Hz)

        # Read parameters
        self.robot_name = self.get_parameter('robot_name').value.strip()
        self.start_side = self.get_parameter('start_side').value.strip().lower()
        self.long_distance = float(self.get_parameter('long_distance').value)
        self.short_distance = float(self.get_parameter('short_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.drift_angular_trim = float(self.get_parameter('drift_angular_trim').value)
        self.drift_strafe_trim = float(self.get_parameter('drift_strafe_trim').value)
        self.distance_scale = float(self.get_parameter('distance_scale').value)
        self.turn_scale = float(self.get_parameter('turn_scale').value)
        self.pause_duration = float(self.get_parameter('pause_duration').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self._validate_parameters()
        self.segments = self._build_segments()

        self.cmd_pub = None
        self.state = 'DISCOVERING'
        self.segment_idx = 0
        self.state_start_time = time.monotonic()
        self.last_discovery_log = 0.0
        self.finished = False

        if self.robot_name:
            self._attach_publisher(self.robot_name)
        else:
            self.get_logger().info(
                'Searching ROS 2 graph for /<robot>/cmd_vel from mqtt_ros2_bridge...'
            )

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._control_loop)

    def _validate_parameters(self):
        if self.start_side not in ('left', 'right'):
            raise ValueError("Parameter 'start_side' must be 'left' or 'right'.")
        if self.linear_speed <= 0.0:
            raise ValueError("Parameter 'linear_speed' must be > 0.")
        if self.angular_speed <= 0.0:
            raise ValueError("Parameter 'angular_speed' must be > 0.")
        if self.publish_rate < 4.0:
            self.get_logger().warning(
                'publish_rate < 4 Hz risks tripping the 0.5 s bridge timeout.'
            )

    def _build_segments(self):
        long_dur = (self.long_distance * self.distance_scale) / self.linear_speed
        short_dur = (self.short_distance * self.distance_scale) / self.linear_speed
        turn_dur = ((math.pi / 2.0) * self.turn_scale) / self.angular_speed

        # Left entrance -> turn right (-angular.z)
        # Right entrance -> turn left (+angular.z)
        turn_sign = -1.0 if self.start_side == 'left' else 1.0
        turn_label = 'right' if self.start_side == 'left' else 'left'
        turn_cmd = turn_sign * self.angular_speed

        raw_plan = [
            ('leg_1_straight', long_dur, self.linear_speed, self.drift_strafe_trim, self.drift_angular_trim),
            ('pause_1', self.pause_duration, 0.0, 0.0, 0.0),
            (f'turn_1_{turn_label}', turn_dur, 0.0, 0.0, turn_cmd),
            ('pause_2', self.pause_duration, 0.0, 0.0, 0.0),
            ('leg_2_short_straight', short_dur, self.linear_speed, self.drift_strafe_trim, self.drift_angular_trim),
            ('pause_3', self.pause_duration, 0.0, 0.0, 0.0),
            (f'turn_2_{turn_label}', turn_dur, 0.0, 0.0, turn_cmd),
            ('pause_4', self.pause_duration, 0.0, 0.0, 0.0),
            ('leg_3_straight', long_dur, self.linear_speed, self.drift_strafe_trim, self.drift_angular_trim),
            ('final_stop', 0.5, 0.0, 0.0, 0.0),
        ]
        # Skip zero-duration segments (e.g., if pause_duration == 0)
        return [seg for seg in raw_plan if seg[1] > 0.0]

    def _attach_publisher(self, name: str):
        self.robot_name = name
        topic = f'/{name}/cmd_vel'
        self.cmd_pub = self.create_publisher(Twist, topic, 10)
        self.state = 'START_DELAY'
        self.state_start_time = time.monotonic()
        self.get_logger().info(
            f'Connected to {topic} | side={self.start_side} | '
            f'long={self.long_distance:.2f}m, short={self.short_distance:.2f}m, '
            f'v={self.linear_speed:.2f}, w={self.angular_speed:.2f}, '
            f'drift_trim={self.drift_angular_trim:+.3f}'
        )
        self.get_logger().info(
            f'Waiting {self.start_delay:.1f}s before starting run...'
        )

    def _try_discover_robot(self):
        for topic_name, topic_types in self.get_topic_names_and_types():
            match = CMD_VEL_PATTERN.fullmatch(topic_name)
            if match and 'geometry_msgs/msg/Twist' in topic_types:
                self._attach_publisher(match.group(1))
                return
        now = time.monotonic()
        if now - self.last_discovery_log > 3.0:
            self.get_logger().info(
                'Waiting for /<robot>/cmd_vel (is start_bridge.sh running?)...'
            )
            self.last_discovery_log = now

    def _publish_twist(self, lx: float, ly: float, az: float):
        if self.cmd_pub is None:
            return
        msg = Twist()
        msg.linear.x = float(lx)
        msg.linear.y = float(ly)
        msg.angular.z = float(az)
        self.cmd_pub.publish(msg)

    def _control_loop(self):
        if self.finished:
            return

        now = time.monotonic()

        if self.state == 'DISCOVERING':
            self._try_discover_robot()
            return

        if self.state == 'START_DELAY':
            self._publish_twist(0.0, 0.0, 0.0)
            if now - self.state_start_time >= self.start_delay:
                self.state = 'RUNNING'
                self.segment_idx = 0
                self.state_start_time = now
                seg = self.segments[0]
                self.get_logger().info(
                    f'Starting segment [1/{len(self.segments)}]: {seg[0]} ({seg[1]:.2f}s)'
                )
            return

        if self.state == 'RUNNING':
            name, duration, lx, ly, az = self.segments[self.segment_idx]
            elapsed = now - self.state_start_time

            if elapsed >= duration:
                self.segment_idx += 1
                if self.segment_idx >= len(self.segments):
                    self._publish_twist(0.0, 0.0, 0.0)
                    self.get_logger().info('Fallback race sequence complete.')
                    self.finished = True
                    return
                self.state_start_time = now
                next_seg = self.segments[self.segment_idx]
                self.get_logger().info(
                    f'Starting segment [{self.segment_idx + 1}/{len(self.segments)}]: '
                    f'{next_seg[0]} ({next_seg[1]:.2f}s)'
                )
                name, duration, lx, ly, az = next_seg

            self._publish_twist(lx, ly, az)

    def send_emergency_stop(self):
        """Send a short burst of zero Twists since MQTT bridge uses qos=0."""
        if self.cmd_pub is None:
            return
        for _ in range(5):
            self._publish_twist(0.0, 0.0, 0.0)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = RaceFallbackNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().warning('Interrupted by user — stopping robot.')
    finally:
        node.send_emergency_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
