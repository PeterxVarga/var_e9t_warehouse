"""Follow a checked Dijkstra route using the shared odometry/stop contract."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from warehouse_control.goal_controller import GoalController, run_controller
from warehouse_control.route import prepare_route, validate_start


class RouteController(GoalController):
    """One depot-start route; fresh simulation required, no online localization."""

    def __init__(self):
        super().__init__('route_controller')

    def goal_parameters(self):
        return dict(goal_node='r3c2')

    def configure_target(self, values):
        try:
            graph = (Path(get_package_share_directory('warehouse_planning'))
                     / 'config' / 'warehouse_graph.json')
            world = (Path(get_package_share_directory('warehouse_sim'))
                     / 'worlds' / 'warehouse.sdf')
        except PackageNotFoundError as exc:
            raise ValueError(f'required installed package not found: {exc}') from exc
        self.route, transform, cost = prepare_route(graph, world, values['goal_node'])
        self.started = False
        self.target_description = f'route to {values["goal_node"]} from a fresh depot start'
        self.get_logger().info(
            f'world <- odom: translation=({transform.x:g}, {transform.y:g}) m, '
            f'yaw={transform.yaw:g} rad; planned length={cost:g} m; '
            f'nodes={" -> ".join(self.route.nodes)}')

    def command_for_pose(self, pose):
        if not self.started:
            validate_start(pose)
            self.started = True
        index = self.route.index
        result = self.route.command(pose, self.config)
        if self.route.index != index or result[2]:
            self.get_logger().info(
                f'Waypoint {index + 1}/{len(self.route.nodes)} '
                f'{self.route.nodes[index]} reached in odom; '
                f'feedback_stamp={self.stamp * 1e-9:.9f} s')
        return result


def main(args=None):
    return run_controller(RouteController, args)
