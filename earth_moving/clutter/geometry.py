import numpy as np
from shapely.geometry import Point, MultiPoint

def create_convex_hull(pebble_positions, circle_points):
    """
    Create a convex hull that includes pebbles and circle points.

    :param pebble_positions: List of pebble positions.
    :param circle_points: Circle boundary points.
    :return: Convex hull geometry.
    """
    all_positions = np.array(pebble_positions + list(circle_points))
    points = MultiPoint([tuple(pos) for pos in all_positions])
    convex_hull = points.convex_hull
    return convex_hull

def find_boundary_pebble_positions(pebble_positions, convex_hull, distance_threshold):
    """
    Find pebbles on the boundary of the convex hull or close to it.

    :param pebble_positions: List of pebble positions.
    :param convex_hull: Convex hull geometry.
    :param distance_threshold: Distance threshold for considering pebbles as boundary pebbles.
    :return: List of boundary pebble positions.
    """
    boundary_pebble_positions = []
    for pos in pebble_positions:
        point = Point(pos)
        if convex_hull.boundary.distance(point) <= distance_threshold:
            boundary_pebble_positions.append(pos)
    return boundary_pebble_positions

def closest_point_on_circle(circle_center, circle_radius, external_point):
    """
    Find the closest point on the circle from an external point.

    :param circle_center: Center of the circle.
    :param circle_radius: Radius of the circle.
    :param external_point: External point.
    :return: Closest point on the circle.
    """
    cx, cy = circle_center
    px, py = external_point

    # Calculate the direction vector from the circle's center to the external point
    direction = np.array([px - cx, py - cy])
    norm_direction = direction / np.linalg.norm(direction)

    # The closest point on the circle's circumference
    closest_point = circle_center + norm_direction * circle_radius

    return closest_point
