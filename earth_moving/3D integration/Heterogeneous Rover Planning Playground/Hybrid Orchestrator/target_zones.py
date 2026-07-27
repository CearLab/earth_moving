"""Serializable, shape-generic target-zone geometry for planning and simulation.

Simple target shapes use analytic point queries. Shapely remains the canonical
polygon representation for visualization, grid conversion, and arbitrary
concave targets, but it is kept out of the high-frequency A* inner loops.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property, lru_cache
import math
from typing import Any, Mapping, Tuple

from shapely.affinity import affine_transform, rotate, scale
from shapely.geometry import Point, Polygon, box
from shapely.ops import nearest_points, triangulate
from shapely.prepared import prep
from shapely.validation import explain_validity


CIRCLE = "circle"
RECTANGLE = "rectangle"
ELLIPSE = "ellipse"
SEMICIRCLE = "semicircle"
POLYGON = "polygon"
TARGET_SHAPES = frozenset({CIRCLE, RECTANGLE, ELLIPSE, SEMICIRCLE, POLYGON})
_EPSILON = 1e-12


def _finite_pair(value, label):
    pair = (float(value[0]), float(value[1]))
    if not all(math.isfinite(item) for item in pair):
        raise ValueError(f"{label} must contain finite coordinates")
    return pair


def _to_local(x, y, center, rotation_deg):
    dx = float(x) - float(center[0])
    dy = float(y) - float(center[1])
    if abs(float(rotation_deg)) <= _EPSILON:
        return dx, dy
    angle = math.radians(float(rotation_deg))
    cosine, sine = math.cos(angle), math.sin(angle)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _to_world(x, y, center, rotation_deg):
    if abs(float(rotation_deg)) <= _EPSILON:
        return float(x) + center[0], float(y) + center[1]
    angle = math.radians(float(rotation_deg))
    cosine, sine = math.cos(angle), math.sin(angle)
    return (
        center[0] + cosine * float(x) - sine * float(y),
        center[1] + sine * float(x) + cosine * float(y),
    )


def _rectangle_signed_distance(local_x, local_y, half_width, half_height):
    qx = abs(float(local_x)) - float(half_width)
    qy = abs(float(local_y)) - float(half_height)
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    return outside + min(max(qx, qy), 0.0)


def _closest_axis_ellipse(x, y, radius_x, radius_y):
    """Return a robust closest boundary point on an axis-aligned ellipse."""
    x, y = float(x), float(y)
    radius_x, radius_y = float(radius_x), float(radius_y)
    sign_x = -1.0 if x < 0.0 else 1.0
    sign_y = -1.0 if y < 0.0 else 1.0
    px, py = abs(x), abs(y)

    if px <= _EPSILON and py <= _EPSILON:
        if radius_x <= radius_y:
            return sign_x * radius_x, 0.0
        return 0.0, sign_y * radius_y

    def distance_sq(angle):
        ex = radius_x * math.cos(angle)
        ey = radius_y * math.sin(angle)
        return (ex - px) ** 2 + (ey - py) ** 2

    # A short coarse scan makes the refinement reliable for points both inside
    # and outside highly eccentric ellipses.
    segments = 16
    step = 0.5 * math.pi / segments
    best_index = min(range(segments + 1), key=lambda index: distance_sq(index * step))
    lower = max(0.0, (best_index - 1) * step)
    upper = min(0.5 * math.pi, (best_index + 1) * step)

    golden = 0.5 * (math.sqrt(5.0) - 1.0)
    left = upper - golden * (upper - lower)
    right = lower + golden * (upper - lower)
    left_value = distance_sq(left)
    right_value = distance_sq(right)
    for _ in range(26):
        if left_value <= right_value:
            upper = right
            right, right_value = left, left_value
            left = upper - golden * (upper - lower)
            left_value = distance_sq(left)
        else:
            lower = left
            left, left_value = right, right_value
            right = lower + golden * (upper - lower)
            right_value = distance_sq(right)
    angle = 0.5 * (lower + upper)
    return sign_x * radius_x * math.cos(angle), sign_y * radius_y * math.sin(angle)


@dataclass(frozen=True)
class TargetZoneSpec:
    """Serializable world-coordinate definition for any supported target shape."""

    shape: str = CIRCLE
    center: Tuple[float, float] = (0.0, 0.0)
    radius: float = 0.8
    width: float = 0.0
    height: float = 0.0
    rotation_deg: float = 0.0
    vertices: Tuple[Tuple[float, float], ...] = ()

    def __post_init__(self):
        shape = str(self.shape).strip().lower()
        if shape not in TARGET_SHAPES:
            raise ValueError(f"target shape must be one of {sorted(TARGET_SHAPES)}")
        center = _finite_pair(self.center, "target center")
        rotation_deg = float(self.rotation_deg)
        if not math.isfinite(rotation_deg):
            raise ValueError("target rotation must be finite")
        vertices = tuple(_finite_pair(value, "polygon vertex") for value in self.vertices)
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "rotation_deg", rotation_deg)
        object.__setattr__(self, "vertices", vertices)

        if shape in (CIRCLE, SEMICIRCLE):
            radius = float(self.radius)
            if not math.isfinite(radius) or radius <= 0.0:
                raise ValueError(f"{shape} target radius must be positive")
            object.__setattr__(self, "radius", radius)
        if shape in (RECTANGLE, ELLIPSE):
            width, height = float(self.width), float(self.height)
            if not all(math.isfinite(value) and value > 0.0 for value in (width, height)):
                raise ValueError(f"{shape} target width and height must be positive")
            object.__setattr__(self, "width", width)
            object.__setattr__(self, "height", height)
        if shape == POLYGON:
            if len(vertices) < 3:
                raise ValueError("polygon target requires at least three vertices")
            polygon = Polygon(vertices)
            if polygon.is_empty or polygon.area <= 1e-10 or not polygon.is_valid:
                raise ValueError(f"invalid polygon target: {explain_validity(polygon)}")
            centroid = polygon.centroid
            object.__setattr__(self, "center", (float(centroid.x), float(centroid.y)))

    @classmethod
    def circle(cls, radius: float, center=(0.0, 0.0)):
        return cls(shape=CIRCLE, center=center, radius=radius)

    @classmethod
    def rectangle(cls, width: float, height: float, center=(0.0, 0.0), rotation_deg=0.0):
        return cls(
            shape=RECTANGLE, center=center, width=width, height=height,
            rotation_deg=rotation_deg)

    @classmethod
    def ellipse(cls, width: float, height: float, center=(0.0, 0.0), rotation_deg=0.0):
        return cls(
            shape=ELLIPSE, center=center, width=width, height=height,
            rotation_deg=rotation_deg)

    @classmethod
    def semicircle(cls, radius: float, center=(0.0, 0.0), direction_deg=0.0):
        """Create a half disk whose curved side points toward ``direction_deg``."""
        return cls(
            shape=SEMICIRCLE, center=center, radius=radius,
            rotation_deg=direction_deg)

    @classmethod
    def polygon(cls, vertices, rotation_deg=0.0):
        return cls(shape=POLYGON, vertices=tuple(vertices), rotation_deg=rotation_deg)

    def translated_to(self, center) -> "TargetZoneSpec":
        """Return the same target shape with its geometric center moved to ``center``."""
        center = _finite_pair(center, "target center")
        if self.shape == POLYGON:
            delta_x = center[0] - self.center[0]
            delta_y = center[1] - self.center[1]
            return TargetZoneSpec.polygon(
                tuple(
                    (float(x) + delta_x, float(y) + delta_y)
                    for x, y in self.vertices
                ),
                rotation_deg=self.rotation_deg,
            )
        return TargetZoneSpec(
            shape=self.shape,
            center=center,
            radius=self.radius,
            width=self.width,
            height=self.height,
            rotation_deg=self.rotation_deg,
        )

    @cached_property
    def geometry(self):
        cx, cy = self.center
        if self.shape == CIRCLE:
            geometry = Point(cx, cy).buffer(self.radius, quad_segs=64)
        elif self.shape == RECTANGLE:
            geometry = box(
                cx - 0.5 * self.width, cy - 0.5 * self.height,
                cx + 0.5 * self.width, cy + 0.5 * self.height)
            if abs(self.rotation_deg) > _EPSILON:
                geometry = rotate(geometry, self.rotation_deg, origin=self.center)
        elif self.shape == ELLIPSE:
            geometry = Point(cx, cy).buffer(1.0, quad_segs=64)
            geometry = scale(
                geometry, xfact=0.5 * self.width, yfact=0.5 * self.height,
                origin=self.center)
            if abs(self.rotation_deg) > _EPSILON:
                geometry = rotate(geometry, self.rotation_deg, origin=self.center)
        elif self.shape == SEMICIRCLE:
            start_angle = self.rotation_deg - 90.0
            arc = []
            for index in range(65):
                angle = math.radians(start_angle + 180.0 * index / 64.0)
                arc.append((cx + self.radius * math.cos(angle), cy + self.radius * math.sin(angle)))
            geometry = Polygon([self.center] + arc)
        else:
            geometry = Polygon(self.vertices)
            if abs(self.rotation_deg) > _EPSILON:
                geometry = rotate(geometry, self.rotation_deg, origin=self.center)
        if geometry.is_empty or geometry.area <= 1e-10 or not geometry.is_valid:
            raise ValueError(f"target geometry is invalid: {explain_validity(geometry)}")
        return geometry

    @cached_property
    def prepared_geometry(self):
        return prep(self.geometry)

    @property
    def bounding_radius(self) -> float:
        if self.shape == CIRCLE:
            return math.hypot(*self.center) + self.radius
        return max(
            math.hypot(float(x), float(y))
            for x, y in self.geometry.exterior.coords
        )

    @property
    def area(self) -> float:
        if self.shape == CIRCLE:
            return math.pi * self.radius ** 2
        if self.shape == RECTANGLE:
            return self.width * self.height
        if self.shape == ELLIPSE:
            return math.pi * 0.5 * self.width * 0.5 * self.height
        if self.shape == SEMICIRCLE:
            return 0.5 * math.pi * self.radius ** 2
        return float(self.geometry.area)

    def _semicircle_boundary_local(self, local_x, local_y):
        radius = self.radius
        flat = (0.0, min(max(float(local_y), -radius), radius))
        norm = math.hypot(local_x, local_y)
        if local_x >= 0.0 and norm > _EPSILON:
            arc = (radius * local_x / norm, radius * local_y / norm)
        else:
            arc = (0.0, radius if local_y >= 0.0 else -radius)
        flat_d2 = (flat[0] - local_x) ** 2 + (flat[1] - local_y) ** 2
        arc_d2 = (arc[0] - local_x) ** 2 + (arc[1] - local_y) ** 2
        return flat if flat_d2 <= arc_d2 else arc

    def signed_distance_world(self, x: float, y: float) -> float:
        """Positive outside, zero on the boundary, negative inside."""
        x, y = float(x), float(y)
        local_x, local_y = _to_local(x, y, self.center, self.rotation_deg)
        if self.shape == CIRCLE:
            return math.hypot(local_x, local_y) - self.radius
        if self.shape == RECTANGLE:
            return _rectangle_signed_distance(
                local_x, local_y, 0.5 * self.width, 0.5 * self.height)
        if self.shape == ELLIPSE:
            radius_x, radius_y = 0.5 * self.width, 0.5 * self.height
            boundary_x, boundary_y = _closest_axis_ellipse(
                local_x, local_y, radius_x, radius_y)
            distance = math.hypot(local_x - boundary_x, local_y - boundary_y)
            normalized = (local_x / radius_x) ** 2 + (local_y / radius_y) ** 2
            return -distance if normalized <= 1.0 + _EPSILON else distance
        if self.shape == SEMICIRCLE:
            boundary_x, boundary_y = self._semicircle_boundary_local(local_x, local_y)
            distance = math.hypot(local_x - boundary_x, local_y - boundary_y)
            inside = local_x >= -_EPSILON and math.hypot(local_x, local_y) <= self.radius + _EPSILON
            return -distance if inside else distance
        point = Point(x, y)
        if self.prepared_geometry.covers(point):
            return -float(self.geometry.boundary.distance(point))
        return float(self.geometry.distance(point))

    def contains_world(self, x: float, y: float, margin: float = 0.0) -> bool:
        x, y, margin = float(x), float(y), float(margin)
        if abs(margin) <= _EPSILON and self.shape != POLYGON:
            local_x, local_y = _to_local(x, y, self.center, self.rotation_deg)
            if self.shape == CIRCLE:
                return local_x * local_x + local_y * local_y <= self.radius ** 2 + _EPSILON
            if self.shape == RECTANGLE:
                return (
                    abs(local_x) <= 0.5 * self.width + _EPSILON
                    and abs(local_y) <= 0.5 * self.height + _EPSILON
                )
            if self.shape == ELLIPSE:
                return (
                    (local_x / (0.5 * self.width)) ** 2
                    + (local_y / (0.5 * self.height)) ** 2
                    <= 1.0 + _EPSILON
                )
            return (
                local_x >= -_EPSILON
                and local_x * local_x + local_y * local_y <= self.radius ** 2 + _EPSILON
            )
        if self.shape != POLYGON:
            return self.signed_distance_world(x, y) <= margin + _EPSILON
        if abs(margin) <= _EPSILON:
            return self.prepared_geometry.covers(Point(x, y))
        prepared = _buffered_prepared_geometry(self, margin)
        return prepared.covers(Point(x, y))

    def distance_world(self, x: float, y: float) -> float:
        return max(0.0, self.signed_distance_world(float(x), float(y)))

    def closest_point_world(self, x: float, y: float) -> Tuple[float, float]:
        x, y = float(x), float(y)
        if self.contains_world(x, y):
            return x, y
        return self.closest_boundary_point_world(x, y)

    def closest_boundary_point_world(self, x: float, y: float) -> Tuple[float, float]:
        x, y = float(x), float(y)
        local_x, local_y = _to_local(x, y, self.center, self.rotation_deg)
        if self.shape == CIRCLE:
            norm = math.hypot(local_x, local_y)
            if norm <= _EPSILON:
                boundary = (self.radius, 0.0)
            else:
                boundary = (self.radius * local_x / norm, self.radius * local_y / norm)
            return _to_world(*boundary, self.center, self.rotation_deg)
        if self.shape == RECTANGLE:
            half_width, half_height = 0.5 * self.width, 0.5 * self.height
            boundary_x = min(max(local_x, -half_width), half_width)
            boundary_y = min(max(local_y, -half_height), half_height)
            if abs(local_x) <= half_width and abs(local_y) <= half_height:
                dx, dy = half_width - abs(local_x), half_height - abs(local_y)
                if dx <= dy:
                    boundary_x = half_width if local_x >= 0.0 else -half_width
                else:
                    boundary_y = half_height if local_y >= 0.0 else -half_height
            return _to_world(boundary_x, boundary_y, self.center, self.rotation_deg)
        if self.shape == ELLIPSE:
            boundary = _closest_axis_ellipse(
                local_x, local_y, 0.5 * self.width, 0.5 * self.height)
            return _to_world(*boundary, self.center, self.rotation_deg)
        if self.shape == SEMICIRCLE:
            boundary = self._semicircle_boundary_local(local_x, local_y)
            return _to_world(*boundary, self.center, self.rotation_deg)
        closest, _ = nearest_points(self.geometry.boundary, Point(x, y))
        return float(closest.x), float(closest.y)

    def outward_direction_world(self, x: float, y: float) -> Tuple[float, float]:
        x, y = float(x), float(y)
        bx, by = self.closest_boundary_point_world(x, y)
        if self.contains_world(x, y):
            dx, dy = bx - x, by - y
        else:
            dx, dy = x - bx, y - by
        norm = math.hypot(dx, dy)
        if norm <= _EPSILON:
            dx, dy = x - self.center[0], y - self.center[1]
            norm = math.hypot(dx, dy)
        if norm <= _EPSILON:
            return 1.0, 0.0
        return dx / norm, dy / norm

    def world_polygon(self, resolution: int = 64):
        return self.geometry

    def grid_polygon(self, grid_size: int, cell_size: float):
        """Transform the world polygon into the legacy top-left grid frame."""
        grid_size = int(grid_size)
        cell_size = float(cell_size)
        grid_center = grid_size // 2
        if (
            self.shape == CIRCLE
            and self.center == (0.0, 0.0)
            and abs(self.rotation_deg) <= _EPSILON
        ):
            return Point(grid_center, grid_center).buffer(max(1, int(self.radius / cell_size)))
        return affine_transform(
            self.geometry,
            [1.0 / cell_size, 0.0, 0.0, -1.0 / cell_size, grid_center, grid_center],
        )

    def boundary_loops_world(self):
        loops = [tuple((float(x), float(y)) for x, y in self.geometry.exterior.coords)]
        loops.extend(
            tuple((float(x), float(y)) for x, y in ring.coords)
            for ring in self.geometry.interiors
        )
        return tuple(loops)

    def triangles_world(self):
        """Return interior triangles suitable for a flat PyBullet visual mesh."""
        result = []
        for triangle in triangulate(self.geometry):
            if not self.prepared_geometry.covers(triangle.representative_point()):
                continue
            coords = tuple((float(x), float(y)) for x, y in list(triangle.exterior.coords)[:3])
            if len(coords) == 3:
                result.append(coords)
        return tuple(result)

    def to_dict(self) -> dict:
        return {
            "shape": self.shape,
            "center": [float(self.center[0]), float(self.center[1])],
            "radius": float(self.radius),
            "width": float(self.width),
            "height": float(self.height),
            "rotation_deg": float(self.rotation_deg),
            "vertices": [[float(x), float(y)] for x, y in self.vertices],
            "bounding_radius": float(self.bounding_radius),
            "area": float(self.area),
        }


@lru_cache(maxsize=128)
def _buffered_prepared_geometry(zone: TargetZoneSpec, margin: float):
    """Cache expensive polygon buffers used by repeated keepout queries."""
    return prep(zone.geometry.buffer(float(margin)))


def resolve_target_zone(value: Any = None, fallback_radius: float = 0.8) -> TargetZoneSpec:
    if isinstance(value, TargetZoneSpec):
        return value
    if value is None:
        return TargetZoneSpec.circle(fallback_radius)
    if isinstance(value, (int, float)):
        return TargetZoneSpec.circle(float(value))
    if isinstance(value, Mapping):
        shape = str(value.get("shape", CIRCLE)).strip().lower()
        center = tuple(value.get("center", (0.0, 0.0)))
        rotation = float(value.get("rotation_deg", 0.0))
        if shape == CIRCLE:
            return TargetZoneSpec.circle(float(value.get("radius", fallback_radius)), center=center)
        if shape == RECTANGLE:
            return TargetZoneSpec.rectangle(
                float(value["width"]), float(value["height"]),
                center=center, rotation_deg=rotation)
        if shape == ELLIPSE:
            return TargetZoneSpec.ellipse(
                float(value["width"]), float(value["height"]),
                center=center, rotation_deg=rotation)
        if shape == SEMICIRCLE:
            return TargetZoneSpec.semicircle(
                float(value.get("radius", fallback_radius)),
                center=center, direction_deg=rotation)
        if shape == POLYGON:
            return TargetZoneSpec.polygon(value["vertices"], rotation_deg=rotation)
        raise ValueError(f"unsupported target shape: {shape}")
    raise TypeError(f"unsupported target-zone definition: {type(value).__name__}")

