"""Print a geometrically checked graph route; never send robot commands."""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from warehouse_planning.roadmap import load_graph, load_obstacles, shortest_path, validate_geometry


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', default='r0c0')
    parser.add_argument('--goal', default='r3c2')
    parser.add_argument('--graph', type=Path, help='Override the installed graph JSON')
    options = parser.parse_args(args)
    try:
        graph_path = options.graph
        if graph_path is None:
            graph_path = Path(get_package_share_directory('warehouse_planning')) / 'config' / 'warehouse_graph.json'
        sdf_path = Path(get_package_share_directory('warehouse_sim')) / 'worlds' / 'warehouse.sdf'
        graph = load_graph(graph_path)
        validate_geometry(graph, load_obstacles(sdf_path))
        result = shortest_path(graph, options.start, options.goal)
        if result is None:
            print(f'No graph route from {options.start} to {options.goal}.', file=sys.stderr)
            return 2
    except (ValueError, OSError, ET.ParseError, PackageNotFoundError) as error:
        print(f'Route planning failed: {error}', file=sys.stderr)
        return 1
    path, cost = result
    print(f'Frame: {graph.frame}')
    print('Nodes: ' + ' -> '.join(path))
    print('Coordinates (m): ' + ' -> '.join(
        f'({graph.nodes[node][0]:g}, {graph.nodes[node][1]:g})' for node in path))
    print(f'Total route length: {cost:g} m')
    return 0
