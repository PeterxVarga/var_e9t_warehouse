"""Track one startup-configured goal, with latched arrival and feedback faults."""

from dataclasses import fields
from math import atan2, hypot, isfinite
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from warehouse_control.control import ControlConfig, goal_command


class GoalController(Node):
    """Consume wheel odometry and publish bounded planar velocity commands."""

    def __init__(self, node_name='goal_controller'):
        super().__init__(node_name)
        try:
            self.configure()
        except Exception:
            self.destroy_node()
            raise
        self.state = 'WAITING_FOR_ODOM'
        self.pose = self.stamp = self.received = self.last_clock = None
        self.publisher = self.create_publisher(Twist, '/warehouse/cmd_vel', 1)
        odom_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            Odometry, '/warehouse/odom', self.on_odom, odom_qos)
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'WAITING_FOR_ODOM: {self.target_description}; timeout={self.timeout} s')
        self.publish_stop()

    def goal_parameters(self):
        return dict(target_x=1.0, target_y=0.0)

    def configure(self):
        """Validate the complete target before creating command publishers/timers."""
        defaults = {f.name: getattr(ControlConfig(), f.name) for f in fields(ControlConfig)}
        defaults.update(odom_timeout=1.0, **self.goal_parameters())
        values = {}
        for name, default in defaults.items():
            values[name] = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
        self.config = ControlConfig(**{f.name: values[f.name] for f in fields(ControlConfig)})
        self.timeout = values['odom_timeout']
        if not isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('odom_timeout must be finite and positive')
        self.configure_target(values)

    def configure_target(self, values):
        self.target = values['target_x'], values['target_y']
        if not all(isfinite(v) for v in self.target):
            raise ValueError('target must be finite')
        self.target_description = f'target={self.target} m in odom'

    def command_for_pose(self, pose):
        return goal_command(*pose, *self.target, self.config)

    def transition(self, state, reason):
        if self.state != state:
            self.state = state
            self.get_logger().info(f'{state}: {reason}')
        if state in ('FAULT', 'REACHED'):
            self.publish_stop()

    def publish_stop(self):
        """Request a stop while the publisher and ROS context are still alive."""
        self.publisher.publish(Twist())

    def on_odom(self, msg):
        if self.state in ('FAULT', 'REACHED'):
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        norm = hypot(q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != 'odom'
                or msg.child_frame_id != 'warehouse_robot/base_link'
                or not all(isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                or not isfinite(norm) or norm < 1e-12
                or msg.header.stamp.sec < 0
                or not 0 <= msg.header.stamp.nanosec < 1000000000):
            self.transition('FAULT', 'invalid odometry pose, frame or timestamp')
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if self.stamp is not None:
            if stamp < self.stamp:
                self.transition('FAULT', 'odometry time moved backwards')
                return
            if stamp == self.stamp:
                return
        qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
        yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.pose, self.stamp, self.received = (p.x, p.y, yaw), stamp, time.monotonic()

    def tick(self):
        if self.state in ('FAULT', 'REACHED'):
            self.publish_stop()
            return
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.transition('FAULT', 'ROS time moved backwards')
        self.last_clock = now
        if self.state in ('FAULT', 'REACHED') or self.pose is None:
            self.publish_stop()
            return
        if time.monotonic() - self.received > self.timeout:
            self.transition('FAULT', 'odometry stopped advancing')
            return
        if now == 0:
            self.publish_stop()
            return
        if abs(now - self.stamp) > self.timeout * 1000000000:
            self.transition('FAULT', 'odometry timestamp is not near ROS time')
            return
        try:
            linear, angular, reached = self.command_for_pose(self.pose)
        except ValueError as exc:
            self.transition('FAULT', str(exc))
            return
        if reached:
            self.transition('REACHED', 'position tolerance satisfied')
            return
        self.transition('TRACKING', 'fresh odometry available')
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        self.publisher.publish(command)


def run_controller(node_factory, args=None):
    """Keep ROS alive until a stop request has been sent on normal termination."""
    stop_requested = False

    def interrupted(_signum, _frame):
        # Raising from a signal handler can interrupt a pybind11 odometry take
        # and emerge as RuntimeError rather than KeyboardInterrupt in Humble.
        # Leave ROS calls on their normal stack and stop after the bounded spin.
        nonlocal stop_requested
        stop_requested = True

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    previous_int = signal.signal(signal.SIGINT, interrupted)
    node = None
    try:
        node = node_factory()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except (ValueError, OSError, ET.ParseError) as exc:
        print(f'Controller failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None and rclpy.ok():
            node.timer.cancel()
            node.publish_stop()
            # Give the transport a short opportunity to deliver the final zero.
            time.sleep(0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
        signal.signal(signal.SIGINT, previous_int)
    return 0


def main(args=None):
    return run_controller(GoalController, args)
