"""Planar simulation feedback from IMU attitude and wheel displacement.

No dynamic world pose is consumed. Pose covariance is unavailable (all-zero
placeholder); twist and its covariance are copied raw, not fused estimates.
"""

from collections import deque
from copy import deepcopy
from math import atan2, cos, isfinite, sin
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from warehouse_control.heading_odometry import HeadingOdometry, heading, multiply, quaternion
from warehouse_control.route import load_spawn


SOURCE_AGE_NS = 150_000_000
INPUT_GAP_NS = 100_000_000
WALL_FRESHNESS_SECONDS = 0.5


def load_mount(model):
    """Read the installed, world-referenced base-link sensor extrinsics."""
    sensor = ET.parse(model).find("./model/link[@name='base_link']/sensor[@name='imu_heading']")
    if sensor is None or sensor.get('type') != 'imu':
        raise ValueError('installed model must contain the base-link imu_heading sensor')
    pose = sensor.find('pose')
    if pose is None or pose.get('relative_to'):
        raise ValueError('IMU pose must be relative to its base link')
    values = [float(v) for v in pose.text.split()]
    reference = sensor.find('imu/orientation_reference_frame')
    if (len(values) != 6 or not all(isfinite(v) for v in values)
            or values[:3] != [0., 0., 0.] or reference is None
            or reference.findtext('localization') != 'CUSTOM'
            or reference.find('custom_rpy') is None
            or reference.find('custom_rpy').get('parent_frame') != 'world'
            or reference.findtext('custom_rpy').split() != ['0', '0', '0']
            or sensor.findtext('imu/enable_orientation') != 'true'):
        raise ValueError('IMU requires a zero-offset mount and identity world orientation reference')
    roll, pitch, yaw = values[3:]
    return multiply(multiply((0., 0., sin(yaw / 2), cos(yaw / 2)),
                             (0., sin(pitch / 2), 0., cos(pitch / 2))),
                    (sin(roll / 2), 0., 0., cos(roll / 2)))


class HeadingNode(Node):
    """Wait for readiness, then latch on any invalid or missing input."""

    def __init__(self):
        super().__init__('heading_odometry')
        try:
            timeout = self.declare_parameter(
                'startup_timeout', 10.0, ParameterDescriptor(read_only=True)).value
            if type(timeout) is not float or not isfinite(timeout) or timeout <= 0:
                raise ValueError('startup_timeout must be a finite positive DOUBLE in wall seconds')
            share = Path(get_package_share_directory('warehouse_sim'))
            self.mount = load_mount(share / 'models/warehouse_robot/model.sdf')
            self.reference = load_spawn(share / 'worlds/warehouse.sdf').yaw
        except Exception:
            self.destroy_node()
            raise
        self.startup_timeout = timeout
        self.core = HeadingOdometry()
        self.pending = deque()
        self.state = 'WAITING_FOR_SENSORS'
        self.last = {'wheel': None, 'imu': None}
        self.counts = {'wheel': 0, 'imu': 0}
        self.receipts = {}
        self.begin = time.monotonic()
        self.clock_seen = None
        self.publisher = self.create_publisher(Odometry, '/warehouse/imu_odom', 10)
        self.create_subscription(Odometry, '/warehouse/odom', self.wheel, qos_profile_sensor_data)
        self.create_subscription(Imu, '/warehouse/imu', self.imu, qos_profile_sensor_data)
        self.timer = self.create_timer(
            .02, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'WAITING_FOR_SENSORS: fresh advancing wheel and IMU inputs; '
            f'startup timeout={timeout:g} wall seconds; no feedback before readiness')

    @property
    def fault(self):
        return self.state == 'FAULT'

    def fail(self, error):
        if not self.fault:
            self.get_logger().error(f'ESTIMATOR_FAULT: {error}')
        self.state = 'FAULT'
        self.pending.clear()

    def stamp(self, msg, kind, expected_frame):
        if msg.header.frame_id != expected_frame:
            raise ValueError(f'{kind} frame mismatch: {msg.header.frame_id}')
        s = msg.header.stamp
        if s.sec < 0 or not 0 <= s.nanosec < 1_000_000_000:
            raise ValueError('invalid source timestamp fields')
        t = s.sec * 1_000_000_000 + s.nanosec
        previous = self.last[kind]
        if previous is not None and t < previous:
            raise ValueError(f'{kind} time moved backwards')
        if previous == t:
            return None
        now = self.get_clock().now().nanoseconds
        if now == 0 or t == 0 or abs(now - t) > SOURCE_AGE_NS:
            if self.state == 'ACTIVE':
                raise ValueError(f'{kind} source time too far from clock')
            # Startup discovery can deliver old queued samples. They neither
            # establish readiness nor refresh channel freshness.
            return None
        if previous is not None and t - previous > INPUT_GAP_NS:
            if self.state == 'ACTIVE':
                raise ValueError(f'{kind} source gap exceeds limit')
            self.core = HeadingOdometry()
            self.pending.clear()
            self.counts = {'wheel': 0, 'imu': 0}
        self.last[kind] = t
        self.counts[kind] += 1
        self.receipts[kind] = time.monotonic()
        return t

    def imu(self, msg):
        if self.fault:
            return
        try:
            q = msg.orientation
            angle = heading((q.x, q.y, q.z, q.w), self.mount, self.reference)
            vectors = (msg.angular_velocity, msg.linear_acceleration)
            values = [getattr(v, k) for v in vectors for k in ('x', 'y', 'z')]
            values += list(msg.orientation_covariance)
            if msg.orientation_covariance[0] == -1 or not all(isfinite(v) for v in values):
                raise ValueError('IMU orientation unavailable or non-finite values')
            t = self.stamp(msg, 'imu', 'warehouse_robot/base_link/imu_heading')
            if t is None:
                return
            self.core.add_imu(t, angle)
            self.drain()
        except ValueError as exc:
            self.fail(exc)

    def wheel(self, msg):
        if self.fault:
            return
        try:
            if msg.child_frame_id != 'warehouse_robot/base_link':
                raise ValueError('wheel child frame mismatch')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            x, y, z, w = quaternion((q.x, q.y, q.z, q.w))
            vectors = (p, msg.twist.twist.linear, msg.twist.twist.angular)
            values = [getattr(v, k) for v in vectors for k in ('x', 'y', 'z')]
            values += list(msg.pose.covariance) + list(msg.twist.covariance)
            if not all(isfinite(v) for v in values):
                raise ValueError('non-finite wheel values')
            t = self.stamp(msg, 'wheel', 'odom')
            if t is None:
                return
            yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            if len(self.pending) >= 10:
                if self.state == 'ACTIVE':
                    raise ValueError('wheel alignment queue overflow')
                self.pending.popleft()
            self.pending.append((t, p.x, p.y, yaw, msg))
            self.drain()
        except ValueError as exc:
            self.fail(exc)

    def ready(self, now):
        return (now > 0 and all(
            self.counts[k] >= 2 and self.last[k] is not None
            and abs(now - self.last[k]) <= SOURCE_AGE_NS
            and time.monotonic() - self.receipts[k] <= WALL_FRESHNESS_SECONDS
            for k in ('wheel', 'imu')))

    def drain(self):
        if self.fault:
            return
        now = self.get_clock().now().nanoseconds
        if not self.ready(now):
            return
        while self.pending:
            t, x, y, yaw, msg = self.pending[0]
            if self.state == 'WAITING_FOR_SENSORS' and (
                    abs(now - t) > SOURCE_AGE_NS
                    or (self.core.imu and t < self.core.imu[0][0])):
                self.pending.popleft()
                continue
            try:
                row = self.core.update(t, x, y, yaw)
            except LookupError:
                return
            if abs(now - t) > SOURCE_AGE_NS:
                raise ValueError('aligned feedback already stale')
            self.pending.popleft()
            out = Odometry()
            out.header = deepcopy(msg.header)
            out.child_frame_id = msg.child_frame_id
            out.pose.pose.position.x, out.pose.pose.position.y = row[1:3]
            out.pose.pose.orientation.z = sin(row[3] / 2)
            out.pose.pose.orientation.w = cos(row[3] / 2)
            # No measured pose uncertainty exists. Zero is a structural
            # placeholder, NOT zero variance. Never use this as filter covariance.
            out.twist = deepcopy(msg.twist)  # Raw wheel velocity, not IMU fusion.
            if self.state != 'ACTIVE':
                self.state = 'ACTIVE'
                self.get_logger().info(
                    f'ACTIVE: paired source-stamped feedback; stamp={t * 1e-9:.9f} s; '
                    'pose covariance unavailable (zero placeholder); twist is raw wheel data')
            self.publisher.publish(out)

    def watchdog(self):
        if self.fault:
            return
        try:
            now = self.get_clock().now().nanoseconds
            if self.clock_seen is not None and now < self.clock_seen:
                raise ValueError('clock moved backwards')
            self.clock_seen = now
            if self.state == 'WAITING_FOR_SENSORS':
                if time.monotonic() - self.begin >= self.startup_timeout:
                    raise ValueError('startup timeout waiting for fresh advancing wheel/IMU and clock')
            else:
                for k in ('wheel', 'imu'):
                    if (time.monotonic() - self.receipts[k] > WALL_FRESHNESS_SECONDS
                            or abs(now - self.last[k]) > SOURCE_AGE_NS):
                        raise ValueError(f'{k} stopped advancing or source became stale')
            self.drain()
        except ValueError as exc:
            self.fail(exc)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except (ValueError, OSError, ET.ParseError) as exc:
        print(f'Heading odometry failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0
