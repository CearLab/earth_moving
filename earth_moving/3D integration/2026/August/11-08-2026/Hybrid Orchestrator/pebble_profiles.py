"""Generic mixed-pebble profiles and count/mass planning adapters."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, Mapping, Sequence

COUNT_MODE = "count"
MASS_MODE = "mass"
MATERIAL_VALUE_MODES = frozenset({COUNT_MODE, MASS_MODE})
DEFAULT_MATERIAL_VALUE_MODE = MASS_MODE


@dataclass(frozen=True)
class PebbleProfile:
    name: str
    visual_scale: float
    material_mass: int
    color: tuple[float, float, float, float]
    physics_mass_scale: float = 1.0

    def __post_init__(self):
        if self.visual_scale <= 0.0:
            raise ValueError("visual_scale must be positive")
        if int(self.material_mass) != self.material_mass or self.material_mass <= 0:
            raise ValueError("material_mass must be a positive integer material unit")
        if self.physics_mass_scale <= 0.0:
            raise ValueError("physics_mass_scale must be positive")


@dataclass(frozen=True)
class PebbleInstance:
    index: int
    profile_name: str
    visual_scale: float
    material_mass: int
    color: tuple[float, float, float, float]
    physics_mass_scale: float = 1.0

    @classmethod
    def from_profile(cls, index: int, profile: PebbleProfile):
        return cls(
            int(index), profile.name, float(profile.visual_scale),
            int(profile.material_mass), tuple(profile.color),
            float(profile.physics_mass_scale),
        )


PEBBLE_PROFILES = {
    "small": PebbleProfile("small", 1.00, 1, (0.25, 0.20, 0.16, 1.0)),
    "medium": PebbleProfile("medium", 2.00, 2, (0.38, 0.31, 0.23, 1.0)),
    "large": PebbleProfile("large", 3.50, 4, (0.52, 0.43, 0.30, 1.0)),
}

DEFAULT_PEBBLE_DISTRIBUTION = {
    "small": 0.30,
    "medium": 0.45,
    "large": 0.25,
}


def normalize_material_mode(mode: str) -> str:
    normalized = str(mode or DEFAULT_MATERIAL_VALUE_MODE).strip().lower()
    if normalized not in MATERIAL_VALUE_MODES:
        raise ValueError(f"material mode must be one of {sorted(MATERIAL_VALUE_MODES)}")
    return normalized


def normalize_distribution(distribution: Mapping[str, float] | None = None):
    source = dict(DEFAULT_PEBBLE_DISTRIBUTION if distribution is None else distribution)
    unknown = set(source) - set(PEBBLE_PROFILES)
    if unknown:
        raise ValueError(f"unknown pebble profiles: {sorted(unknown)}")
    weights = {name: max(0.0, float(source.get(name, 0.0))) for name in PEBBLE_PROFILES}
    total = sum(weights.values())
    if total <= 0.0:
        raise ValueError("pebble distribution must contain positive weight")
    return {name: weight / total for name, weight in weights.items()}


def generate_pebble_instances(count: int, seed: int, distribution=None):
    """Generate a deterministic profile sequence independently of position RNG."""
    distribution = normalize_distribution(distribution)
    names = sorted(PEBBLE_PROFILES)
    cumulative = []
    running = 0.0
    for name in names:
        running += distribution[name]
        cumulative.append((running, name))
    rng = random.Random(int(seed) ^ 0x5EEDBEEF)
    instances = []
    for index in range(max(0, int(count))):
        value = rng.random()
        name = next((name for threshold, name in cumulative if value <= threshold), names[-1])
        instances.append(PebbleInstance.from_profile(index, PEBBLE_PROFILES[name]))
    return tuple(instances)


def planning_units(instance: PebbleInstance, material_mode: str) -> int:
    return int(instance.material_mass) if normalize_material_mode(material_mode) == MASS_MODE else 1


def expand_positions_for_planning(
    positions_xyz: Sequence[Sequence[float]],
    instances: Sequence[PebbleInstance],
    material_mode: str,
):
    """Expand physical positions into integer material quanta for the legacy 2D solver."""
    if len(positions_xyz) != len(instances):
        raise ValueError("positions and pebble instances must have equal length")
    expanded = []
    for position, instance in zip(positions_xyz, instances):
        point = (float(position[0]), float(position[1]), float(position[2]))
        expanded.extend([point] * planning_units(instance, material_mode))
    return expanded


def annotate_material_cells(env_2d, converter, pebbles_xy, instances):
    """Attach true physical count and mass to canonical cells after 2D computation."""
    if len(pebbles_xy) != len(instances):
        raise ValueError("positions and pebble instances must have equal length")
    counts = {}
    masses = {}
    profile_counts = {}
    for position, instance in zip(pebbles_xy, instances):
        key = tuple(converter.convert_3d_to_2d(float(position[0]), float(position[1])))
        counts[key] = counts.get(key, 0) + 1
        masses[key] = masses.get(key, 0) + int(instance.material_mass)
        per_profile = profile_counts.setdefault(key, {})
        per_profile[instance.profile_name] = per_profile.get(instance.profile_name, 0) + 1
    for cell in getattr(env_2d, "all_cells", []):
        key = (int(cell.x), int(cell.y))
        cell.physical_object_count = int(counts.get(key, 0))
        cell.material_mass = int(masses.get(key, 0))
        cell.pebble_profile_counts = dict(profile_counts.get(key, {}))
    env_2d.total_physical_objects = len(pebbles_xy)
    env_2d.total_material_mass = sum(int(instance.material_mass) for instance in instances)
    return env_2d


def summarize_instances(instances: Iterable[PebbleInstance]):
    summary = {name: {"count": 0, "material_mass": 0} for name in PEBBLE_PROFILES}
    total_count = 0
    total_mass = 0
    for instance in instances:
        item = summary.setdefault(instance.profile_name, {"count": 0, "material_mass": 0})
        item["count"] += 1
        item["material_mass"] += int(instance.material_mass)
        total_count += 1
        total_mass += int(instance.material_mass)
    return {"total_count": total_count, "total_material_mass": total_mass, "profiles": summary}


def progress_from_positions(pebbles, target_zone_radius: float = None, target_zone=None):
    """Summarize count/mass inside and outside any supported target shape."""
    from target_zones import resolve_target_zone

    zone = resolve_target_zone(
        target_zone,
        0.8 if target_zone_radius is None else float(target_zone_radius),
    )
    delivered_count = delivered_mass = remaining_count = remaining_mass = 0
    delivered_profiles = {}
    remaining_profiles = {}
    for position, instance in pebbles:
        inside = zone.contains_world(float(position[0]), float(position[1]))
        count_bucket = delivered_profiles if inside else remaining_profiles
        count_bucket[instance.profile_name] = count_bucket.get(instance.profile_name, 0) + 1
        if inside:
            delivered_count += 1
            delivered_mass += int(instance.material_mass)
        else:
            remaining_count += 1
            remaining_mass += int(instance.material_mass)
    return {
        "delivered_count": delivered_count,
        "delivered_material_mass": delivered_mass,
        "remaining_count": remaining_count,
        "remaining_material_mass": remaining_mass,
        "delivered_profiles": delivered_profiles,
        "remaining_profiles": remaining_profiles,
    }

