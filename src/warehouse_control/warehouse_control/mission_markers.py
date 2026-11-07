"""Complete RViz planning snapshots in odom; never a measured trajectory."""

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def mission_markers(graph, transform, plan, before_plan, closed_edges,
                    original_requests, remaining_order, stamp):
    """Replace this topic's entire display, including labels for duplicate IDs.

    Convert world graph coordinates with the same static transform used by
    execution. RViz needs only fixed frame odom, with no TF publisher required.
    The two polylines retain full plan snapshots, including traversed prefixes.
    """
    result = MarkerArray()
    clear = Marker()
    clear.header.frame_id = 'odom'
    clear.header.stamp = stamp
    clear.action = Marker.DELETEALL
    result.markers.append(clear)

    def marker(namespace, kind, color, width):
        item = Marker()
        item.header.frame_id = 'odom'
        item.header.stamp = stamp
        item.ns = namespace
        item.id = len(result.markers)
        item.type = kind
        item.action = Marker.ADD
        item.pose.orientation.w = 1.0
        item.color.r, item.color.g, item.color.b, item.color.a = color
        item.scale.x = item.scale.y = item.scale.z = width
        result.markers.append(item)
        return item

    def point(node, z):
        x, y = transform.world_to_odom(graph.nodes[node])
        return Point(x=float(x), y=float(y), z=z)

    def label(namespace, node, text, z, color, offset=(0.0, 0.0)):
        item = marker(namespace, Marker.TEXT_VIEW_FACING, color, 0.22)
        item.pose.position = point(node, z)
        item.pose.position.x += offset[0]
        item.pose.position.y += offset[1]
        item.text = text
        return item

    if before_plan is not None and len(before_plan.nodes) > 1:
        item = marker('plan_before_resume', Marker.LINE_STRIP, (0.2, 0.5, 1.0, 0.65), 0.18)
        item.points = [point(node, 0.04) for node in before_plan.nodes]
    if len(plan.nodes) > 1:
        item = marker('accepted_plan', Marker.LINE_STRIP, (0.1, 1.0, 0.25, 1.0), 0.045)
        item.points = [point(node, 0.09) for node in plan.nodes]
    if closed_edges:
        item = marker('accepted_closures', Marker.LINE_LIST, (1.0, 0.1, 0.1, 1.0), 0.09)
        item.points = [point(node, 0.14) for edge in closed_edges for node in edge]
    item = marker('plan_start', Marker.SPHERE, (1.0, 0.8, 0.1, 1.0), 0.28)
    item.pose.position = point(plan.nodes[0], 0.2)
    label('plan_start_label', plan.nodes[0], f'Plan start: {plan.nodes[0]}', 0.5,
          (1.0, 0.8, 0.1, 1.0), offset=(0.0, 0.4))
    if remaining_order:
        item = marker('remaining_goals', Marker.SPHERE_LIST, (1.0, 0.3, 0.9, 1.0), 0.22)
        item.points = [point(original_requests[i], 0.22) for i in remaining_order]
    heights = {}
    for i in remaining_order:
        node = original_requests[i]
        height = heights.get(node, 0)
        heights[node] = height + 1
        label('remaining_request_labels', node, f'Request {i + 1}: {node}',
              0.8 + height * 0.25, (1.0, 0.3, 0.9, 1.0),
              offset=(0.0, 0.8 + height * 0.35))
    label('depot', graph.depot, f'Return depot: {graph.depot}', 0.3,
          (1.0, 1.0, 1.0, 1.0), offset=(0.0, -0.4))
    legend = label('legend', graph.depot,
                   'PLANS, NOT TRAVELLED PATHS\n'
                   'blue: before resume | green: accepted | red: logical closures\n'
                   'yellow: plan start | pink: remaining requests',
                   1.8, (1.0, 1.0, 1.0, 1.0))
    graph_points = [transform.world_to_odom(p) for p in graph.nodes.values()]
    legend.pose.position.x = (min(p[0] for p in graph_points)
                              + max(p[0] for p in graph_points)) / 2
    legend.pose.position.y = min(p[1] for p in graph_points) - 1.8
    return result
