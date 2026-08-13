"""Generic rover geometry, capability, and task-policy configuration."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

TARGET_TASK = "target"
HIGHWAY_TASK = "highway"
TASK_POLICY_MODES = frozenset({"legacy", "dynamic", "target_only", "highway_only"})
OVERCAPACITY_BEHAVIORS = frozenset({"allow", "deprioritize", "reject"})
BASE_SHOVEL_WIDTH = 0.22
BASE_SHOVEL_DEPTH = 0.01
BASE_SHOVEL_HEIGHT = 0.08
BASE_SHOVEL_OFFSET = 0.17
BASE_CHASSIS_RADIUS = 0.15
BASE_WHEEL_RADIUS = 0.07
BASE_TRACK_WIDTH = 0.20
BASE_SAFETY_RADIUS = 0.46

# Percentage-based task-policy defaults.  Fractions are based on the physical
# count of pebbles that remain outside the target zone, not logical mass.
#
# Strict behavior is selected with task_policy_mode. Fractions are only the
# adjustable thresholds used by dynamic mode.
DEFAULT_GOOD_LOCATION_POTENTIAL_RATIO = 0.65
SMALL_TARGET_PREFERENCE_ENTER_FRACTION = 0.30
SMALL_TARGET_PREFERENCE_EXIT_FRACTION = 0.20
SMALL_ENDGAME_TARGET_ONLY_REMAINING_FRACTION = 0.20


@dataclass(frozen=True)
class RoverGeometry:
    """Final world dimensions, with optional uniform chassis scaling."""
    chassis_scale: float = 1.0
    shovel_width: float = BASE_SHOVEL_WIDTH
    shovel_depth: float = BASE_SHOVEL_DEPTH
    shovel_height: float = BASE_SHOVEL_HEIGHT
    shovel_offset: float = BASE_SHOVEL_OFFSET
    wheel_radius_override: float | None = None
    track_width_override: float | None = None
    planning_cell_size_override: float | None = None

    def __post_init__(self):
        for name in ("chassis_scale", "shovel_width", "shovel_depth", "shovel_height", "shovel_offset"):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_scale(cls, scale: float, **overrides):
        values = {
            "chassis_scale": scale,
            "shovel_width": BASE_SHOVEL_WIDTH * scale,
            "shovel_depth": BASE_SHOVEL_DEPTH * scale,
            "shovel_height": BASE_SHOVEL_HEIGHT * scale,
            "shovel_offset": BASE_SHOVEL_OFFSET * scale,
        }
        values.update(overrides)
        return cls(**values)

    @property
    def body_radius(self): return BASE_CHASSIS_RADIUS * self.chassis_scale
    @property
    def wheel_radius(self): return self.wheel_radius_override or BASE_WHEEL_RADIUS * self.chassis_scale
    @property
    def track_width(self): return self.track_width_override or BASE_TRACK_WIDTH * self.chassis_scale
    @property
    def overlay_cell_size(self):
        return self.planning_cell_size_override or self.shovel_width / (2.0 * math.sqrt(2.0))
    @property
    def astar_radius(self):
        corner = math.hypot(self.shovel_offset + self.shovel_depth / 2.0, self.shovel_width / 2.0)
        return max(self.body_radius, corner)
    @property
    def collision_radius(self):
        scaled_base_width = BASE_SHOVEL_WIDTH * self.chassis_scale
        return BASE_SAFETY_RADIUS * self.chassis_scale + max(0.0, self.shovel_width - scaled_base_width) / 2.0
    @property
    def reservation_radius(self): return self.shovel_width / 2.0 + 0.03 * self.chassis_scale
    @property
    def overlay_signature(self):
        return tuple(round(value, 6) for value in (
            self.chassis_scale, self.shovel_width, self.shovel_depth,
            self.shovel_height, self.shovel_offset, self.overlay_cell_size,
        ))


@dataclass(frozen=True)
class RoverCapabilities:
    capacity_objects: int = 8
    capacity_mass: int = 10
    supported_tasks: frozenset[str] = field(default_factory=lambda: frozenset({TARGET_TASK, HIGHWAY_TASK}))

    def __post_init__(self):
        if self.capacity_objects <= 0:
            raise ValueError("capacity_objects must be positive")
        if self.capacity_mass <= 0:
            raise ValueError("capacity_mass must be positive")


@dataclass(frozen=True)
class TaskPolicy:
    allowed_tasks: frozenset[str] = field(default_factory=lambda: frozenset({TARGET_TASK, HIGHWAY_TASK}))
    # "dynamic" uses the percentage thresholds, while the two *_only modes
    # are explicit and do not rely on special 0%/100% values.
    task_policy_mode: str = "legacy"
    # Earlier entries are strict preferences; later entries are fallbacks only.
    # Retained for profiles that do not enable percentage-based selection.
    task_fallback_order: tuple[str, ...] = ()
    # A cell is a good direct-target location when its normalized heat-map
    # potential meets this ratio and a feasible target candidate exists.
    good_location_potential_ratio: float = DEFAULT_GOOD_LOCATION_POTENTIAL_RATIO
    # Enter target preference at `enter` and return to highway preference at
    # `exit`. Values between the two retain the previous rover preference.
    target_preference_enter_fraction: float | None = None
    target_preference_exit_fraction: float | None = None
    allow_highway_fallback_when_target_preferred: bool = True
    allow_target_fallback_when_highway_preferred: bool = True
    # Once this share (or less) of the original physical pebbles remains
    # outside the target, dynamic mode becomes strict target-only. None disables.
    endgame_target_only_remaining_fraction: float | None = None
    # When enabled, direct-to-target work starts only at an upstream/root source:
    # no other source corridor may collect this source on its way to the target.
    target_root_sources_only: bool = False
    delivered_weight: float = 10.0
    target_weight: float = 1.0
    highway_weight: float = 0.7
    heat_weight: float = 2.0
    capacity_utilization_weight: float = 2.0
    approach_distance_weight: float = 0.5
    spillage_weight: float = 2.0
    minimum_task_objects: float = 1.0
    # Hard feasibility floor. A candidate below this fraction is rejected, so
    # the rover waits/parks for a better load. It is evaluated as the larger of
    # physical object-count utilization and material-mass utilization.
    minimum_capacity_utilization: float = 0.0
    # Candidate load uses the uncapped number of physical objects and material
    # mass on the path. Capacity-fit candidates can be ranked before overloaded
    # candidates without rejecting the latter completely.
    preferred_capacity_min_fraction: float = 0.35
    preferred_capacity_max_fraction: float = 1.00
    overcapacity_behavior: str = "deprioritize"
    max_overcapacity_fraction: float | None = None
    capacity_fit_before_task_fallback: bool = True
    allow_wait: bool = True

    def __post_init__(self):
        if self.task_policy_mode not in TASK_POLICY_MODES:
            raise ValueError(f"task_policy_mode must be one of {sorted(TASK_POLICY_MODES)}")
        if self.overcapacity_behavior not in OVERCAPACITY_BEHAVIORS:
            raise ValueError(
                f"overcapacity_behavior must be one of {sorted(OVERCAPACITY_BEHAVIORS)}"
            )
        if len(set(self.task_fallback_order)) != len(self.task_fallback_order):
            raise ValueError("task_fallback_order cannot contain duplicates")
        unknown = set(self.task_fallback_order) - set(self.allowed_tasks)
        if unknown:
            raise ValueError(f"fallback tasks must also be allowed: {sorted(unknown)}")
        if not 0.0 <= float(self.good_location_potential_ratio) <= 1.0:
            raise ValueError("good_location_potential_ratio must be between 0 and 1")
        percentage_values = (
            self.target_preference_enter_fraction,
            self.target_preference_exit_fraction,
        )
        if (percentage_values[0] is None) != (percentage_values[1] is None):
            raise ValueError(
                "target preference enter/exit fractions must both be set or both be None"
            )
        if percentage_values[0] is not None:
            enter = float(percentage_values[0])
            exit_fraction = float(percentage_values[1])
            if not 0.0 <= exit_fraction <= enter <= 1.0:
                raise ValueError(
                    "target preference fractions must satisfy 0 <= exit <= enter <= 1"
                )
            required = {TARGET_TASK, HIGHWAY_TASK}
            missing = required.difference(self.allowed_tasks)
            if missing and self.task_policy_mode == "dynamic":
                raise ValueError(
                    "dynamic percentage policies require target and highway in allowed_tasks"
                )
        if self.task_policy_mode == "dynamic" and percentage_values[0] is None:
            raise ValueError("dynamic task policy requires enter/exit fractions")
        if self.task_policy_mode == "target_only" and TARGET_TASK not in self.allowed_tasks:
            raise ValueError("target_only policy must allow target tasks")
        if self.task_policy_mode == "highway_only" and HIGHWAY_TASK not in self.allowed_tasks:
            raise ValueError("highway_only policy must allow highway tasks")
        if self.endgame_target_only_remaining_fraction is not None:
            fraction = float(self.endgame_target_only_remaining_fraction)
            if not 0.0 <= fraction <= 1.0:
                raise ValueError("endgame target-only remaining fraction must be between 0 and 1")
        capacity_min = float(self.preferred_capacity_min_fraction)
        capacity_max = float(self.preferred_capacity_max_fraction)
        if not 0.0 <= capacity_min <= capacity_max:
            raise ValueError("preferred capacity fractions must satisfy 0 <= min <= max")
        hard_minimum = float(self.minimum_capacity_utilization)
        if not 0.0 <= hard_minimum <= capacity_max:
            raise ValueError(
                "minimum capacity utilization must be between 0 and the preferred maximum"
            )
        if self.max_overcapacity_fraction is not None:
            if float(self.max_overcapacity_fraction) < capacity_max:
                raise ValueError("max overcapacity fraction cannot be below preferred maximum")

    def permits(self, task_type: str) -> bool:
        return task_type in self.allowed_tasks

    @property
    def uses_percentage_policy(self):
        return self.task_policy_mode == "dynamic"

    def selection_tiers(
        self,
        target_ready_fraction=None,
        previous_preference=None,
        remaining_fraction=None,
    ):
        """Return strict candidate tiers; scoring happens only inside one tier.

        Strict modes return one tier only. Dynamic policies retain the previous
        preference inside the hysteresis band and may become strict target-only
        during the configured percentage-based endgame.
        """
        if self.task_policy_mode == "target_only":
            return (frozenset({TARGET_TASK}),)
        if self.task_policy_mode == "highway_only":
            return (frozenset({HIGHWAY_TASK}),)
        if self.task_policy_mode == "dynamic":
            endgame_threshold = self.endgame_target_only_remaining_fraction
            if (
                endgame_threshold is not None
                and remaining_fraction is not None
                and float(remaining_fraction) <= float(endgame_threshold) + 1e-12
            ):
                return (frozenset({TARGET_TASK}),)
            enter = float(self.target_preference_enter_fraction)
            exit_fraction = float(self.target_preference_exit_fraction)
            ready_fraction = max(0.0, min(1.0, float(target_ready_fraction or 0.0)))
            previous = (
                previous_preference
                if previous_preference in (TARGET_TASK, HIGHWAY_TASK)
                else HIGHWAY_TASK
            )
            if previous == TARGET_TASK:
                preferred = (
                    HIGHWAY_TASK
                    if ready_fraction <= exit_fraction
                    else TARGET_TASK
                )
            else:
                preferred = (
                    TARGET_TASK
                    if ready_fraction >= enter
                    else HIGHWAY_TASK
                )
            fallback = HIGHWAY_TASK if preferred == TARGET_TASK else TARGET_TASK
            tiers = [frozenset({preferred})]
            allow_fallback = (
                self.allow_highway_fallback_when_target_preferred
                if preferred == TARGET_TASK
                else self.allow_target_fallback_when_highway_preferred
            )
            if allow_fallback:
                tiers.append(frozenset({fallback}))
            return tuple(tiers)

        tiers = [frozenset({task}) for task in self.task_fallback_order]
        remaining = frozenset(self.allowed_tasks.difference(self.task_fallback_order))
        if remaining:
            tiers.append(remaining)
        return tuple(tiers)


@dataclass(frozen=True)
class RoverType:
    name: str
    geometry: RoverGeometry
    capabilities: RoverCapabilities
    policy: TaskPolicy
    # Smaller numeric value means higher right-of-way within the same task phase.
    right_of_way_priority: float = 0.0

    # Compatibility properties keep existing orchestrator code profile-driven.
    @property
    def shovel_width(self): return self.geometry.shovel_width
    @property
    def shovel_offset(self): return self.geometry.shovel_offset
    @property
    def body_radius(self): return self.geometry.body_radius
    @property
    def astar_radius(self): return self.geometry.astar_radius
    @property
    def collision_radius(self): return self.geometry.collision_radius
    @property
    def reservation_radius(self): return self.geometry.reservation_radius
    @property
    def overlay_cell_size(self): return self.geometry.overlay_cell_size
    @property
    def path_cell_size(self): return self.geometry.overlay_cell_size
    @property
    def capacity_objects(self): return self.capabilities.capacity_objects
    @property
    def capacity_mass(self): return self.capabilities.capacity_mass
    @property
    def overlay_cache_key(self):
        return (
            self.geometry.overlay_signature,
            self.capabilities.capacity_objects,
            self.capabilities.capacity_mass,
            tuple(sorted(self.capabilities.supported_tasks)),
        )


ROVER_TYPES = {
    "small": RoverType(
        "small",
        RoverGeometry(),
        RoverCapabilities(capacity_objects=8, capacity_mass=10),
        TaskPolicy(
            allowed_tasks=frozenset({HIGHWAY_TASK, TARGET_TASK}),
            task_policy_mode="dynamic",
            target_preference_enter_fraction=SMALL_TARGET_PREFERENCE_ENTER_FRACTION,
            target_preference_exit_fraction=SMALL_TARGET_PREFERENCE_EXIT_FRACTION,
            endgame_target_only_remaining_fraction=(
                SMALL_ENDGAME_TARGET_ONLY_REMAINING_FRACTION
            ),
            target_weight=1.0,
            highway_weight=1.0,
        ),
        right_of_way_priority=0.0,
    ),
    "large": RoverType(
        "large",
        RoverGeometry(shovel_width=0.33),  # Same chassis; wider shovel only.
        RoverCapabilities(capacity_objects=12, capacity_mass=18, supported_tasks=frozenset({TARGET_TASK, HIGHWAY_TASK})),
        TaskPolicy(
            allowed_tasks=frozenset({TARGET_TASK, HIGHWAY_TASK}),
            task_policy_mode="target_only",
            target_root_sources_only=True,
            target_weight=1.4,
            highway_weight=0.0,
            capacity_utilization_weight=3.0,
            # Initial research hypothesis: the large rover waits unless a path
            # uses at least 35% of either its object or material-mass capacity.
            minimum_capacity_utilization=0.35,
        ),
        right_of_way_priority=0.0,
    ),
}
ROVER_PROFILES = ROVER_TYPES  # Backward-compatible public name.


def resolve_profiles(spec, rover_count):
    names = [part.strip().lower() for part in (spec or "small").split(",") if part.strip()]
    if len(names) == 1:
        names *= rover_count
    if len(names) != rover_count:
        raise ValueError(f"expected one rover type or {rover_count} comma-separated types")
    unknown = set(names) - set(ROVER_TYPES)
    if unknown:
        raise ValueError(f"unknown rover types: {sorted(unknown)}")
    return [ROVER_TYPES[name] for name in names]


def ensure_profile_urdf(base_urdf, rover_type, output_dir=None):
    """Generate a geometry-specific URDF; global scaling is applied by PyBullet."""
    geometry = rover_type.geometry if hasattr(rover_type, "geometry") else rover_type
    base_path = Path(base_urdf)
    scale = geometry.chassis_scale
    default_scaled = (
        abs(geometry.shovel_width - BASE_SHOVEL_WIDTH * scale) < 1e-9
        and abs(geometry.shovel_depth - BASE_SHOVEL_DEPTH * scale) < 1e-9
        and abs(geometry.shovel_height - BASE_SHOVEL_HEIGHT * scale) < 1e-9
        and abs(geometry.shovel_offset - BASE_SHOVEL_OFFSET * scale) < 1e-9
    )
    if default_scaled:
        return str(base_path)

    output_root = Path(output_dir) if output_dir else base_path.parent / "generated_rovers"
    output_root.mkdir(parents=True, exist_ok=True)
    safe_name = re.sub(r"[^a-zA-Z0-9_-]+", "_", getattr(rover_type, "name", "custom"))
    signature = "_".join(str(int(round(value * 1000))) for value in (
        scale, geometry.shovel_width, geometry.shovel_depth, geometry.shovel_offset,
    ))
    output_path = output_root / f"2_wheel_rover_{safe_name}_{signature}.urdf"

    tree = ET.parse(base_path)
    root = tree.getroot()
    shovel = root.find("./link[@name='shovel']")
    if shovel is None:
        raise ValueError(f"No shovel link found in {base_path}")
    desired_pre_scale = (
        geometry.shovel_depth / scale,
        geometry.shovel_width / scale,
        geometry.shovel_height / scale,
    )
    boxes = shovel.findall("./visual/geometry/box") + shovel.findall("./collision/geometry/box")
    if len(boxes) < 2:
        raise ValueError(f"Shovel visual/collision boxes not found in {base_path}")
    for box in boxes:
        box.set("size", " ".join(f"{value:.6g}" for value in desired_pre_scale))
    joint = root.find("./joint[@name='base_to_fattach']/origin")
    if joint is None:
        raise ValueError(f"Shovel attachment joint not found in {base_path}")
    xyz = [float(value) for value in joint.attrib["xyz"].split()]
    xyz[0] = geometry.shovel_offset / scale
    joint.set("xyz", " ".join(f"{value:.6g}" for value in xyz))
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return str(output_path)




