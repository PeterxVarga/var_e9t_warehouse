"""Print a geometrically checked FIFO mission; never send robot commands."""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from warehouse_planning.mission import plan_fifo_mission
from warehouse_planning.roadmap import load_graph, load_obstacles, validate_geometry


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--pickups', nargs='*', required=True, metavar='NODE',
                        help='FIFO pickup requests; no nodes means an empty mission')
    parser.add_argument('--graph', type=Path, help='Override the installed graph JSON')
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
        plan = plan_fifo_mission(graph, options.pickups)
    except (ValueError, OSError, ET.ParseError, PackageNotFoundError) as error:
        print(f'Mission planning failed: {error}', file=sys.stderr)
        return 1

    lines = [f'Frame: {graph.frame}',
             'Pickup requests: ' + (' -> '.join(plan.pickup_nodes) or '(none)')]
    for index, leg in enumerate(plan.legs):
        label = f'Pickup {index + 1}' if index < len(plan.pickup_nodes) else 'Return'
        lines.append(f'{label}: {leg.start} -> {leg.goal}; distance: {leg.distance_m:g} m')
    lines.extend(['Nodes: ' + ' -> '.join(plan.nodes),
                  f'Total planned distance: {plan.total_distance_m:g} m'])
    print('\n'.join(lines))
    return 0
