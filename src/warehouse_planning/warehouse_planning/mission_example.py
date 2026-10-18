"""Print a geometrically checked mission; never send robot commands."""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.ordering import nearest_neighbor_order, two_opt_order
from warehouse_planning.roadmap import load_graph, load_obstacles, validate_geometry


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--pickups', nargs='*', required=True, metavar='NODE',
                        help='Pickup requests in input order; no nodes means an empty mission')
    parser.add_argument('--graph', type=Path, help='Override the installed graph JSON')
    parser.add_argument('--strategy', choices=('fifo', 'nn', 'nn-2opt'), default='fifo',
                        help='Request ordering: FIFO (default), nearest neighbor, or NN with 2-opt')
    options = parser.parse_args(args)
    try:
        graph_path = options.graph
        if graph_path is None:
            graph_path = (Path(get_package_share_directory('warehouse_planning'))
                          / 'config' / 'warehouse_graph.json')
        graph = load_graph(graph_path)
        sdf_path = (Path(get_package_share_directory('warehouse_sim'))
                    / 'worlds' / 'warehouse.sdf')
        obstacles = load_obstacles(sdf_path)
        validate_geometry(graph, obstacles)
        order = tuple(range(len(options.pickups)))
        if options.strategy != 'fifo':
            order = nearest_neighbor_order(graph, options.pickups)
            if options.strategy == 'nn-2opt':
                order = two_opt_order(graph, options.pickups, order)
        plan = plan_fifo_mission(graph, [options.pickups[i] for i in order])
    except (ValueError, OSError, ET.ParseError, PackageNotFoundError) as error:
        print(f'Mission planning failed: {error}', file=sys.stderr)
        return 1

    lines = [f'Frame: {graph.frame}']
    if options.strategy != 'fifo':
        original = ' -> '.join(f'{i + 1}: {node}' for i, node in enumerate(options.pickups))
        execution = ' -> '.join(f'{i + 1}: {options.pickups[i]}' for i in order)
        lines.extend([f'Strategy: {options.strategy}',
                      'Original pickup requests: ' + (original or '(none)'),
                      'Execution order: ' + (execution or '(none)')])
    lines.append('Pickup requests: ' + (' -> '.join(plan.pickup_nodes) or '(none)'))
    for index, leg in enumerate(plan.legs):
        label = f'Pickup {index + 1}' if index < len(plan.pickup_nodes) else 'Return'
        if options.strategy != 'fifo' and index < len(plan.pickup_nodes):
            label += f' (request {order[index] + 1})'
        lines.append(f'{label}: {leg.start} -> {leg.goal}; distance: {leg.distance_m:g} m')
    lines.extend(['Nodes: ' + ' -> '.join(plan.nodes),
                  f'Total planned distance: {plan.total_distance_m:g} m'])
    print('\n'.join(lines))
    return 0
