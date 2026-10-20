"""Execute a startup-configured ordered mission using configured odometry feedback."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.parameter import Parameter

from warehouse_control.goal_controller import GoalController, run_controller
from warehouse_control.mission import prepare_mission


class MissionController(GoalController):
    def __init__(self):
        super().__init__('mission_controller')

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
        return command

    def on_odom(self, msg):
        previous = self.stamp
        super().on_odom(msg)
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
