"""Execute a startup-configured ordered mission using configured odometry feedback."""

from collections import deque
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
from warehouse_interfaces.srv import ResumeMission, ResumeMissionWithClosures
from warehouse_planning.roadmap import graph_without_edges

from warehouse_control.goal_controller import GoalController, run_controller
from warehouse_control.mission import prepare_mission
from warehouse_control.mission_markers import mission_markers


class MissionController(GoalController):
    def __init__(self):
        super().__init__('mission_controller')
        # The default mutually exclusive callback group serializes this service
        # with odometry and the control timer, including with a multithreaded executor.
        self.rest_samples = deque(maxlen=4096)
        self.accepted_closures = ()
        self.before_resume_plan = None
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/warehouse/mission_plan_markers', QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pause_service = self.create_service(
            Trigger, '/warehouse/pause_mission', self.on_pause)
        self.resume_service = self.create_service(
            ResumeMission, '/warehouse/resume_mission', self.on_resume)
        self.closures_service = self.create_service(
            ResumeMissionWithClosures, '/warehouse/resume_mission_with_closures',
            self.on_resume_with_closures)
        self.publish_plan_markers()

    def publish_plan_markers(self):
        remaining = self.mission._active_order[self.mission.leg_index:]
        self.marker_publisher.publish(mission_markers(
            self.mission.validated_graph, self.transform, self.mission.plan,
            self.before_resume_plan, self.accepted_closures,
            self.mission.original_requests, remaining, self.get_clock().now().to_msg()))

    def on_pause(self, _request, response):
        if self.state in ('FAULT', 'REACHED'):
            response.success = False
            response.message = f'Pause unavailable: mission is {self.state}.'
            return response
        already_paused = self.state == 'PAUSED'
        try:
            self.mission.pause()
        except RuntimeError:
            response.success = False
            response.message = 'Pause unavailable: mission has not started.'
            return response
        if not already_paused:
            self.rest_samples.clear()
            self.before_resume_plan = self.mission.plan
            self.publish_plan_markers()
        self.transition('PAUSED', 'pause request accepted')
        self.publish_stop()
        response.success = True
        response.message = 'Pause request accepted.'
        return response

    def _check_rest(self, now, wall):
        """Check the bounded, continuously sampled paused odometry window."""
        samples = self.rest_samples
        if self.state != 'PAUSED' or self.mission.state != 'PAUSED':
            raise ValueError('mission is not PAUSED')
        if not samples or self.pose is None or self.received is None:
            raise ValueError('no paused odometry observation')
        if (now <= 0 or (self.last_clock is not None and now < self.last_clock)
                or wall < self.received or wall - self.received > min(self.timeout, 0.2)
                or abs(now - self.stamp) > min(self.timeout, 0.2) * 1e9):
            raise ValueError('odometry or ROS clock is stale or invalid')
        first, last = samples[0], samples[-1]
        if last[0] != self.stamp or last[1] != self.received:
            raise ValueError('odometry observation was replaced')
        if last[0] - first[0] < 500000000 or last[1] - first[1] < 0.5:
            raise ValueError('need at least 0.5 s of paused odometry')
        if any(b[0] - a[0] > 200000000 or b[1] - a[1] > 0.2
               or b[0] <= a[0] or b[1] <= a[1]
               for a, b in zip(samples, list(samples)[1:])):
            raise ValueError('paused odometry has a sampling gap')
        x, y, yaw = first[2]
        if any(not sample[3] or hypot(sample[2][0] - x, sample[2][1] - y) > 0.002
               or abs(atan2(sin(sample[2][2] - yaw), cos(sample[2][2] - yaw))) > 0.002
               for sample in samples):
            raise ValueError('odometry indicates motion or invalid velocity')

    def on_resume(self, request, response):
        # Compatibility service deliberately retains the last accepted closures.
        return self._resume(request.start_node, self.accepted_closures, response)

    def on_resume_with_closures(self, request, response):
        edges = tuple((edge.node_a, edge.node_b) for edge in request.closed_edges)
        return self._resume(request.start_node, edges, response)

    def _resume(self, start_node, closed_edges, response):
        """Validate and commit one complete plan/closure snapshot for both APIs."""
        try:
            now, wall = self.get_clock().now().nanoseconds, time.monotonic()
            self._check_rest(now, wall)
            if start_node not in self.mission.validated_graph.nodes:
                raise ValueError(f'unknown graph node: {start_node}')
            graph = graph_without_edges(self.mission.validated_graph, closed_edges)
            closures = tuple(sorted(tuple(sorted(edge)) for edge in closed_edges))
            stamp, received, pose = self.stamp, self.received, self.pose
            previous_plan = self.mission.plan

            def precommit():
                # The default mutually exclusive callback group prevents odom,
                # pause and timer callbacks from replacing the snapshot here.
                current = self.get_clock().now().nanoseconds
                self._check_rest(current, time.monotonic())
                if (self.stamp, self.received, self.pose) != (stamp, received, pose):
                    raise ValueError('odometry changed during planning')
                if current < now:
                    raise ValueError('ROS time moved backwards during planning')

            self.mission.resume_from_node(
                graph, start_node, self.transform,
                pose, self.config, stamp, now, precommit=precommit)
        except (ValueError, RuntimeError) as exc:
            if self.state == 'PAUSED':
                self.publish_stop()
            response.success = False
            response.message = f'Resume unavailable: {exc}'
            return response
        # No callback can observe a partial commit in this mutually exclusive
        # group. Failed planning/precommit leaves closures and markers untouched.
        self.accepted_closures = closures
        self.before_resume_plan = previous_plan
        self.publish_plan_markers()
        self.rest_samples.clear()
        self.transition('TRACKING', f'resume accepted from {start_node}')
        self.get_logger().info(
            f'Accepted resume plan: start={start_node}; closures={closures}; '
            f'route={self.mission.plan.nodes}; '
            f'remaining_request_ids={tuple(i + 1 for i in self.mission._active_order)}')
        response.success = True
        response.message = f'Resumed from {start_node}; {len(closures)} closed edge(s).'
        return response

    def goal_parameters(self):
        try:
            strategy = self.declare_parameter(
                'strategy', 'fifo', ParameterDescriptor(read_only=True))
        except InvalidParameterTypeException as exc:
            raise ValueError('strategy must be a STRING: fifo, nn, nn-2opt') from exc
        if (strategy.type_ != Parameter.Type.STRING
                or strategy.value not in ('fifo', 'nn', 'nn-2opt')):
            raise ValueError('strategy must be a STRING: fifo, nn, nn-2opt')
        # Humble's YAML parser represents an explicit [] override as NOT_SET;
        # Python list inference instead produces BYTE_ARRAY. Public parameter
        # getters cannot distinguish absent and explicit NOT_SET overrides.
        # Inspect only presence in this Humble startup mapping, then use public
        # APIs to type/lock the value before any motion publisher/executor exists.
        supplied = 'pickup_nodes' in self._parameter_overrides
        value = self.declare_parameter(
            'pickup_nodes', None, ParameterDescriptor(dynamic_typing=True))
        if value.type_ == Parameter.Type.NOT_SET and not supplied:
            raise ValueError('pickup_nodes is required; provide a STRING_ARRAY (or explicit [])')
        empty_override = value.type_ == Parameter.Type.NOT_SET and supplied
        if (not empty_override and value.type_ != Parameter.Type.STRING_ARRAY
                and not (value.type_ in (Parameter.Type.BYTE_ARRAY, Parameter.Type.BOOL_ARRAY,
                                        Parameter.Type.INTEGER_ARRAY, Parameter.Type.DOUBLE_ARRAY)
                         and len(value.value) == 0)):
            raise ValueError('pickup_nodes must be a STRING_ARRAY (or explicit [])')
        requests = [] if empty_override else list(value.value)
        result = self.set_parameters([
            Parameter('pickup_nodes', Parameter.Type.STRING_ARRAY, requests)])[0]
        if not result.successful:
            raise ValueError(f'cannot initialize pickup_nodes: {result.reason}')
        self.set_descriptor(
            'pickup_nodes', ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY, read_only=True))
        return {}

    def configure_target(self, values):
        try:
            graph = (Path(get_package_share_directory('warehouse_planning'))
                     / 'config/warehouse_graph.json')
            world = Path(get_package_share_directory('warehouse_sim')) / 'worlds/warehouse.sdf'
        except PackageNotFoundError as exc:
            raise ValueError(f'installed warehouse package missing: {exc}') from exc
        self.mission, self.transform = prepare_mission(
            graph, world, self.get_parameter('pickup_nodes').value,
            self.get_parameter('strategy').value)
        self.target_description = 'FIFO mission from the depot in a fresh simulation'
        if self.mission.strategy != 'fifo':
            self.target_description = f'{self.mission.strategy} mission from the depot in a fresh simulation'
            execution = tuple((i + 1, self.mission.original_requests[i])
                              for i in self.mission.request_order)
            self.get_logger().info(
                f'Mission strategy: {self.mission.strategy}; '
                f'original pickup requests: {self.mission.original_requests}; '
                f'execution order: {execution}; '
                f'planned length={self.mission.plan.total_distance_m:g} m; '
                f'static world-to-odom spawn={self.transform}; simulated service=1 s per request')
            return
        self.get_logger().info(
            f'FIFO pickup requests: {self.mission.plan.pickup_nodes}; '
            f'planned length={self.mission.plan.total_distance_m:g} m; '
            f'static world-to-odom spawn={self.transform}; simulated service=1 s per request')

    def command_for_pose(self, pose):
        command = self.mission.command(pose, self.config, self.stamp, self.last_clock)
        for event in self.mission.events:
            timing = (f'feedback_stamp={event.feedback_stamp * 1e-9:.9f} s; '
                      f'clock_stamp={event.clock_stamp * 1e-9:.9f} s')
            if event.kind == 'waypoint':
                total = len(self.mission.plan.legs[event.leg].nodes)
                message = (f'Mission waypoint leg {event.leg + 1} '
                           f'{event.waypoint + 1}/{total} {event.node} reached in odom')
            elif event.kind == 'arrival':
                message = f'Pickup request {event.request_index + 1} {event.node} arrived; simulated service started'
            elif event.kind == 'serviced':
                message = f'Pickup request {event.request_index + 1} {event.node} simulated service completed'
            elif event.kind == 'depot':
                message = f'Final depot {event.node} reached in odom'
            else:
                message = f'Pickup request {event.request_index + 1} {event.node} {event.kind}'
            self.get_logger().info(f'{message}; {timing}')
        if any(event.kind in ('serviced', 'depot') for event in self.mission.events):
            self.publish_plan_markers()
        return command

    def on_odom(self, msg):
        previous = self.stamp
        super().on_odom(msg)
        if self.state == 'PAUSED':
            if self.stamp != previous:
                twist = msg.twist.twist
                velocity = (twist.linear.x, twist.linear.y, twist.linear.z,
                            twist.angular.x, twist.angular.y, twist.angular.z)
                still = all(isfinite(v) and abs(v) <= 0.01 for v in velocity)
                samples = self.rest_samples
                if samples and (self.stamp - samples[-1][0] > 200000000
                                or self.received - samples[-1][1] > 0.2
                                or self.received <= samples[-1][1]):
                    samples.clear()
                samples.append((self.stamp, self.received, self.pose, still))
                # Bound the observation even if one clock advances very slowly.
                while (samples and (self.stamp - samples[0][0] > 2000000000
                                    or self.received - samples[0][1] > 2.0)):
                    samples.popleft()
                # Keep the shortest suffix covering both required durations.
                while (len(samples) > 2 and self.stamp - samples[1][0] >= 500000000
                       and self.received - samples[1][1] >= 0.5):
                    samples.popleft()
            else:
                # Duplicate/invalid input must not refresh or bridge a window.
                self.rest_samples.clear()
        if self.state not in ('FAULT', 'REACHED') and self.stamp != previous:
            # A departure followed by a return between timer callbacks must
            # still invalidate the continuous service interval.
            try:
                self.mission.observe_service_pose(self.pose, self.config)
            except ValueError as exc:
                self.transition('FAULT', str(exc))

    def transition(self, state, reason):
        if state == 'FAULT':
            self.mission.fail()
        elif state == 'TRACKING':
            state = self.mission.state
        super().transition(state, reason)


def main(args=None):
    return run_controller(MissionController, args)
