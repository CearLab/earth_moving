"""Named, reproducible thesis scenarios for click-to-run experiments."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping

from pebble_profiles import COUNT_MODE, DEFAULT_PEBBLE_DISTRIBUTION, MASS_MODE
from target_zones import TargetZoneSpec


@dataclass(frozen=True)
class ScenarioConfig:
    name: str
    description: str
    target_zone: TargetZoneSpec
    env_radius: float = 3.0
    num_pebbles: int = 50
    random_seed: int = 41
    rover_profiles: str = "small,large,small"
    material_mode: str = MASS_MODE
    pebble_distribution: Mapping[str, float] = field(
        default_factory=lambda: dict(DEFAULT_PEBBLE_DISTRIBUTION)
    )

    def with_target_center(self, center) -> "ScenarioConfig":
        """Return this scenario with its target translated to a requested world position."""
        target_zone = self.target_zone.translated_to(center)
        if target_zone.bounding_radius > float(self.env_radius) + 1e-9:
            raise ValueError(
                f"target centered at {target_zone.center} extends outside the "
                f"{self.env_radius:.3f}m environment radius "
                f"(requires {target_zone.bounding_radius:.3f}m)"
            )
        return replace(self, target_zone=target_zone)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "target_zone": self.target_zone.to_dict(),
            "env_radius": float(self.env_radius),
            "num_pebbles": int(self.num_pebbles),
            "random_seed": int(self.random_seed),
            "rover_profiles": self.rover_profiles,
            "material_mode": self.material_mode,
            "pebble_distribution": dict(self.pebble_distribution),
        }


# Bare-name aliases for the click-to-run SCENARIO_NAME setting. They intentionally
# use the same lowercase spelling as the scenario keys so either of these works:
# SCENARIO_NAME = rectangle_target
# SCENARIO_NAME = "rectangle_target"
baseline_circle_uniform = "baseline_circle_uniform"
mixed_pebbles_circle = "mixed_pebbles_circle"
rectangle_target = "rectangle_target"
heterogeneous_cooperation = "heterogeneous_cooperation"
ellipse_target = "ellipse_target"
semicircle_target = "semicircle_target"
l_shape_target = "l_shape_target"
off_center_target = "off_center_target"
concave_target = "concave_target"
amorphous_target = "amorphous_target"
lab_circle_center = "lab_circle_center"
lab_circle_offcenter = "lab_circle_offcenter"
lab_rectangle_center = "lab_rectangle_center"
lab_rectangle_offcenter = "lab_rectangle_offcenter"
lab_amorphous_center = "lab_amorphous_center"
lab_amorphous_offcenter = "lab_amorphous_offcenter"
lab_l_center = "lab_l_center"
lab_l_offcenter = "lab_l_offcenter"

_THIN_L_SHAPE_VERTICES = (
    (-0.90, -0.65),
    (-0.46, -0.65),
    (-0.46, 0.25),
    (0.90, 0.25),
    (0.90, 0.65),
    (-0.90, 0.65),
)

_CONCAVE_L_SHAPE_VERTICES = (
    (-0.90, -0.65),
    (0.10, -0.65),
    (0.10, -0.15),
    (0.90, -0.15),
    (0.90, 0.65),
    (-0.90, 0.65),
)

SCENARIOS = {
    "baseline_circle_uniform": ScenarioConfig(
        name="baseline_circle_uniform",
        description="Legacy circular target, uniform small pebbles and homogeneous small rovers.",
        target_zone=TargetZoneSpec.circle(0.8),
        rover_profiles="small,small,small",
        material_mode=COUNT_MODE,
        pebble_distribution={"small": 1.0, "medium": 0.0, "large": 0.0},
    ),
    "mixed_pebbles_circle": ScenarioConfig(
        name="mixed_pebbles_circle",
        description="Current mixed-material experiment with the legacy circular target.",
        target_zone=TargetZoneSpec.circle(0.8),
    ),
    "off_center_target": ScenarioConfig(
        name="off_center_target",
        description="Random mixed aggregates with a circular target fixed away from the arena center.",
        target_zone=TargetZoneSpec.circle(radius=0.8, center=(1.25, 0.65)),
        random_seed=73,
    ),
    "rectangle_target": ScenarioConfig(
        name="rectangle_target",
        description="Comparable-area axis-aligned rectangular target using the same heatmap method.",
        # 1.80 * 1.12 ~= pi * 0.8^2, so shape changes without materially changing area.
        target_zone=TargetZoneSpec.rectangle(width=1.80, height=1.12),
    ),
    "heterogeneous_cooperation": ScenarioConfig(
        name="heterogeneous_cooperation",
        description="Mixed rovers and materials with the circular target for role-policy studies.",
        target_zone=TargetZoneSpec.circle(0.8),
    ),
    "ellipse_target": ScenarioConfig(
        name="ellipse_target",
        description="Rotated ellipse with approximately the same area as the baseline circle.",
        target_zone=TargetZoneSpec.ellipse(
            width=2.00, height=1.28, rotation_deg=25.0),
    ),
    "semicircle_target": ScenarioConfig(
        name="semicircle_target",
        description="Oriented half-disk target with a flat and a curved entry boundary.",
        target_zone=TargetZoneSpec.semicircle(
            radius=1.00, direction_deg=30.0),
    ),
    "l_shape_target": ScenarioConfig(
        name="l_shape_target",
        description="Thin L-shaped target with a concave inner corner.",
        target_zone=TargetZoneSpec.polygon(_THIN_L_SHAPE_VERTICES),
    ),
    "concave_target": ScenarioConfig(
        name="concave_target",
        description="Original thicker L-shaped concave target retained for comparison.",
        target_zone=TargetZoneSpec.polygon(_CONCAVE_L_SHAPE_VERTICES),
    ),
    "amorphous_target": ScenarioConfig(
        name="amorphous_target",
        description="Fixed reproducible irregular non-convex target.",
        target_zone=TargetZoneSpec.polygon([
            (-1.00, -0.15),
            (-0.65, -0.65),
            (-0.10, -0.55),
            (0.35, -0.85),
            (0.90, -0.35),
            (0.70, 0.15),
            (1.00, 0.55),
            (0.25, 0.70),
            (-0.15, 0.48),
            (-0.75, 0.75),
        ]),
    ),
}


# Controlled single-rover lab targets.  Each centered/off-center pair uses the
# same geometry; the experiment runner separately fixes count mode, uniform
# small pebbles, pebble count, rover profile, and seed.
_LAB_OFFCENTER = (1.00, 0.50)
_LAB_REFERENCE_AREA = SCENARIOS["baseline_circle_uniform"].target_zone.area


def _lab_equal_area_polygon(zone):
    """Scale a polygon about its centroid to the baseline circle area."""
    factor = (_LAB_REFERENCE_AREA / zone.area) ** 0.5
    cx, cy = zone.center
    scaled = TargetZoneSpec.polygon(tuple(
        (cx + factor * (float(x) - cx), cy + factor * (float(y) - cy))
        for x, y in zone.vertices
    ))
    return scaled.translated_to((0.0, 0.0))


_LAB_AMORPHOUS = _lab_equal_area_polygon(SCENARIOS["amorphous_target"].target_zone)
_LAB_L = _lab_equal_area_polygon(SCENARIOS["l_shape_target"].target_zone)
_LAB_TARGETS = {
    "lab_circle_center": SCENARIOS["baseline_circle_uniform"].target_zone,
    "lab_circle_offcenter": SCENARIOS["baseline_circle_uniform"].target_zone.translated_to(_LAB_OFFCENTER),
    "lab_rectangle_center": SCENARIOS["rectangle_target"].target_zone,
    "lab_rectangle_offcenter": SCENARIOS["rectangle_target"].target_zone.translated_to(_LAB_OFFCENTER),
    "lab_amorphous_center": _LAB_AMORPHOUS,
    "lab_amorphous_offcenter": _LAB_AMORPHOUS.translated_to(_LAB_OFFCENTER),
    "lab_l_center": _LAB_L,
    "lab_l_offcenter": _LAB_L.translated_to(_LAB_OFFCENTER),
}
for _lab_name, _lab_target in _LAB_TARGETS.items():
    SCENARIOS[_lab_name] = ScenarioConfig(
        name=_lab_name,
        description=(
            "Controlled lab target-shape comparison; material and rover "
            "conditions are supplied by the experiment plan."
        ),
        target_zone=_lab_target,
        rover_profiles="small",
        material_mode=COUNT_MODE,
        pebble_distribution={"small": 1.0, "medium": 0.0, "large": 0.0},
    )


# Change this one line when running by clicking Play in the IDE.
DEFAULT_SCENARIO_NAME = rectangle_target


def get_scenario(name: str) -> ScenarioConfig:
    key = str(name).strip().lower()
    try:
        return SCENARIOS[key]
    except KeyError as exc:
        raise ValueError(f"unknown scenario {name!r}; choose from {sorted(SCENARIOS)}") from exc
