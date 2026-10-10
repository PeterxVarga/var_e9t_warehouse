#!/usr/bin/env python3
"""Offline, timestamp-matched physical/odometry measurements; never a controller."""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def samples(path):
    values = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    if not values or any(not isinstance(value, list) for value in values):
        raise ValueError(f'invalid measurement stream: {path}')
    if any(b[0] <= a[0] for a, b in zip(values, values[1:])):
        raise ValueError(f'non-increasing timestamps: {path}')
    return values


def interpolate(values, stamp):
    """Bracket, interpolate and bound sample gaps; yaw follows the short arc."""
    index = bisect_left([value[0] for value in values], stamp)
    if index < len(values) and values[index][0] == stamp:
        return list(values[index])
    if index == 0 or index == len(values):
        raise ValueError(f'unbracketed timestamp {stamp}')
    a, b = values[index - 1], values[index]
    gap = b[0] - a[0]
    if gap > 0.1:
        raise ValueError(f'measurement gap {gap} at {stamp}')
    fraction = (stamp - a[0]) / gap
    result = [stamp] + [x + fraction * (y - x) for x, y in zip(a[1:], b[1:])]
    result[3] = wrap(a[3] + fraction * wrap(b[3] - a[3]))
    return result


def phase_metrics(physical, odom, start, end):
    p0, p1 = interpolate(physical, start), interpolate(physical, end)
    o0, o1 = interpolate(odom, start), interpolate(odom, end)
    pd = (p1[1] - p0[1], p1[2] - p0[2])
    od = (o1[1] - o0[1], o1[2] - o0[2])
    pl, ol = math.hypot(*pd), math.hypot(*od)
    py, oy = wrap(p1[3] - p0[3]), wrap(o1[3] - o0[3])
    return dict(start_stamp=start, end_stamp=end, physical_yaw_change=py,
                odom_yaw_change=oy, yaw_change_error_rad=wrap(py - oy),
                yaw_ratio=py / oy if abs(oy) > 0.1 else None,
                physical_distance_m=pl, odom_distance_m=ol,
                distance_error_m=pl - ol, distance_ratio=pl / ol if ol > 0.1 else None,
                direction_error_rad=wrap(math.atan2(pd[1], pd[0]) - math.atan2(od[1], od[0]))
                if min(pl, ol) > 0.1 else None,
                lateral_error_m=pd[1] * math.cos(o0[3]) - pd[0] * math.sin(o0[3]),
                start_yaw_disagreement_rad=wrap(p0[3] - o0[3]),
                end_yaw_disagreement_rad=wrap(p1[3] - o1[3]))


def geometry_audit(model):
    root = ET.parse(model).getroot().find('model')
    plugin = root.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    wheels = {}
    for side in ('left', 'right'):
        link = root.find(f"link[@name='{side}_wheel']")
        pose = [float(value) for value in link.findtext('pose').split()]
        collision = link.find("collision[@name='wheel_collision']")
        cylinder = collision.find('geometry/cylinder')
        wheels[side] = dict(center=pose[:3], radius=float(cylinder.findtext('radius')),
                            width=float(cylinder.findtext('length')),
                            collision_pose=[float(value) for value in collision.findtext('pose').split()],
                            joint_axis=root.findtext(f"joint[@name='{side}_wheel_joint']/axis/xyz"),
                            plugin_joint=plugin.findtext(f'{side}_joint'))
        roll, pitch, heading = wheels[side]['collision_pose'][3:]
        collision_axis = (math.cos(heading) * math.sin(pitch) * math.cos(roll)
                          + math.sin(heading) * math.sin(roll),
                          math.sin(heading) * math.sin(pitch) * math.cos(roll)
                          - math.cos(heading) * math.sin(roll),
                          math.cos(pitch) * math.cos(roll))
        joint_axis = [float(value) for value in wheels[side]['joint_axis'].split()]
        wheels[side]['collision_axis_parallel_to_joint'] = (
            abs(abs(sum(a*b for a, b in zip(collision_axis, joint_axis))) - 1) < 1e-9
            and all(abs(value) < 1e-9 for value in pose[3:]))
        mass = float(link.findtext('inertial/mass'))
        r, width = wheels[side]['radius'], wheels[side]['width']
        expected = (mass * (3*r*r + width*width) / 12, mass * r*r / 2,
                    mass * (3*r*r + width*width) / 12)
        actual = [float(link.findtext(f'inertial/inertia/{key}')) for key in ('ixx', 'iyy', 'izz')]
        wheels[side]['inertia_matches_collision_cylinder'] = all(
            abs(a-b) < 1e-9 for a, b in zip(actual, expected))
    nominal = math.dist(wheels['left']['center'], wheels['right']['center'])
    separation, radius = float(plugin.findtext('wheel_separation')), float(plugin.findtext('wheel_radius'))
    return dict(wheels=wheels, nominal_center_separation_m=nominal,
                diffdrive_separation_m=separation, diffdrive_radius_m=radius,
                separation_matches_centers=abs(nominal - separation) < 1e-9,
                radius_matches_collisions=all(abs(w['radius'] - radius) < 1e-9 for w in wheels.values()),
                wheel_centers_at_radius_height=all(abs(w['center'][2] - w['radius']) < 1e-9
                                                   for w in wheels.values()))


def waypoint_metrics(directory, graph, world):
    directory = Path(directory)
    physical, odom = samples(directory / 'physical.jsonl'), samples(directory / 'odom.jsonl')
    nodes = {node['id']: (node['x'], node['y']) for node in json.loads(Path(graph).read_text())['nodes']}
    spawn = [float(value) for value in ET.parse(world).getroot().findtext('world/include/pose').split()]
    c, s = math.cos(spawn[5]), math.sin(spawn[5])
    pattern = r'Waypoint (\d+)/(\d+) (\w+) reached in odom; feedback_stamp=([\d.]+) s'
    results = []
    for index, count, node, stamp in re.findall(pattern, (directory / 'controller.log').read_text()):
        stamp = float(stamp)
        p, o = interpolate(physical, stamp), interpolate(odom, stamp)
        transformed = (spawn[0] + c * o[1] - s * o[2], spawn[1] + s * o[1] + c * o[2])
        results.append(dict(index=int(index), total=int(count), node=node, stamp=stamp,
                            physical_xy=p[1:3], odom_world_xy=transformed,
                            disagreement_xy=[p[1] - transformed[0], p[2] - transformed[1]],
                            position_disagreement_m=math.dist(p[1:3], transformed),
                            physical_goal_error_m=math.dist(p[1:3], nodes[node]),
                            odom_goal_error_m=math.dist(transformed, nodes[node]),
                            physical_yaw=p[3], odom_world_yaw=wrap(o[3] + spawn[5]),
                            yaw_disagreement_rad=wrap(p[3] - o[3] - spawn[5])))
    if not results:
        raise ValueError('no timestamped waypoint events')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--graph', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = {str(directory): waypoint_metrics(directory, args.graph, args.world)
            for directory in args.directories}
    args.output.write_text(json.dumps(data, indent=2) + '\n')


if __name__ == '__main__':
    main()
