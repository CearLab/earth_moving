"""
Default parameter tuning for APF navigation.

Provides reasonable defaults adapted for earth-moving rovers based on:
- Rover radius: ~0.35m
- Target zone radius: ~1.5m
- Operating speeds: 0.3-0.8 m/s
- Safety margins for multi-rover coordination
"""

from .models import RoverParams


def defaults() -> RoverParams:
    """
    Return default APF parameters tuned for earth-moving rovers with small pebbles.

    These values are optimized for navigating around small obstacles (pebble radius ~0.05m)
    while maintaining safety for the rover (radius ~0.35m).

    Returns:
        RoverParams with default tuning
    """
    return RoverParams(
        # Attractive field (goal seeking)
        k_att=1.5,           # Strong attraction to overcome small obstacles
        d_switch=2.0,        # Switch to conic at 2m from goal

        # Static repulsion (small pebbles)
        k_rep=0.2,           # Reduced repulsion for small obstacles (was 0.5)
        d0=0.8,              # Shorter range for small obstacles (was 1.5m)

        # Dynamic repulsion (other rovers)
        k_dyn=0.8,           # Strong inter-rover repulsion
        alpha=3.0,           # Moderate sigmoid steepness
        beta=0.5,            # Moderate closing speed weighting
        dyn_range=3.0,       # Consider neighbors within 3m

        # Safety distances (based on rover radius ~0.35m + pebble ~0.05m)
        d_safe=0.45,         # Safety bubble: rover + pebble + small margin
        d_stop=0.35,         # Emergency stop: rover + pebble radius
        d_slow=0.8,          # Start slowing: comfortable margin (was 1.0m)

        # Control parameters
        v0=0.5,              # Nominal speed: 0.5 m/s (moderate)
        v_max=0.8,           # Max speed: 0.8 m/s (conservative)
        w_max=1.5,           # Max turn rate: ~86°/s (smooth turning)
        k_w=2.0,             # Angular gain: responsive but not jerky
        w_slow=0.8,          # Reduce speed if turning faster than 46°/s

        # Tangential bias (optional, for head-on scenarios)
        bias_eta=0.0         # Disabled by default (enable with 0.1-0.3)
    )


def aggressive() -> RoverParams:
    """
    Aggressive tuning for faster navigation with small pebbles.

    Minimal obstacle avoidance - will pass close to pebbles (0.35-0.45m).

    CORRECTED: Fixed freeze issue by ensuring d0 > d_stop + buffer.
    The repulsion must have room to work before safety stop triggers!

    Use when:
    - Single rover operation (no collision risk)
    - Sparse obstacles (small pebbles)
    - Time-critical tasks
    - Willing to accept close passes

    Returns:
        RoverParams with aggressive tuning
    """
    return RoverParams(
        # Attractive field (goal seeking)
        k_att=5.5,           # Very strong attraction
        d_switch=1.5,        # Earlier switch to conic

        # Static repulsion (small pebbles) - weak but functional
        k_rep=0.08,          # Minimal but enough to avoid (was 0.05 - too weak!)
        d0=0.55,             # Sufficient range for repulsion to work (was 0.45 - too short!)

        # Dynamic repulsion
        k_dyn=0.5,
        alpha=2.0,
        beta=0.3,
        dyn_range=2.5,

        # Safety distances - tight but functional
        d_safe=0.22,         # Tight safety margin
        d_stop=0.01,         # Close stop distance
        d_slow=0.35,         # Moderate slowing (< d0 for consistency)

        # Control parameters - fast
        v0=0.9,              # High nominal speed
        v_max=1.2,           # High max speed (reduced from 1.3 for stability)
        w_max=3.5,           # Fast turning (reduced from 2.8 for stability)
        k_w=3.0,             # Responsive (reduced from 3.5 for stability)
        w_slow=1.5,          # Less speed reduction during turns

        # Tangential bias
        bias_eta=0.0
    )


def conservative() -> RoverParams:
    """
    Conservative tuning for maximum safety with small pebbles.

    Use when:
    - Dense pebble fields
    - Narrow passages or cluttered environments
    - Want maximum clearance from obstacles
    - Precision over speed

    Returns:
        RoverParams with conservative tuning
    """
    return RoverParams(
        # Attractive field (goal seeking)
        k_att=1.0,           # Moderate attraction (don't overpower repulsion)
        d_switch=2.5,        # Later switch for smoother approach

        # Static repulsion (small pebbles) - stronger for safety
        k_rep=0.3,           # Stronger repulsion for safety (was 0.8)
        d0=1.0,              # Longer range for early avoidance (was 2.0m)

        # Dynamic repulsion
        k_dyn=1.2,
        alpha=4.0,
        beta=0.7,
        dyn_range=4.0,

        # Safety distances - larger margins
        d_safe=0.50,         # Larger safety bubble
        d_stop=0.40,         # Stop further away
        d_slow=1.0,          # Slow down earlier (was 1.5m)

        # Control parameters - slow and careful
        v0=0.3,              # Slow nominal speed
        v_max=0.5,           # Low max speed
        w_max=1.0,           # Gentle turning
        k_w=1.5,             # Smooth angular response
        w_slow=0.5,          # More speed reduction during turns

        # Tangential bias - helps in tight spaces
        bias_eta=0.2
    )


def custom(k_rep=0.2, d0=0.8, k_att=1.5, v_max=0.8) -> RoverParams:
    """
    Create custom APF parameters with direct control over key values.

    This allows fine-tuning the repulsion influence for small pebbles.

    IMPORTANT: Ensures d0 > d_slow > d_safe > d_stop to prevent freezing!

    Args:
        k_rep (float): Repulsion strength (0.08-0.5). Lower = less intimidated by obstacles
        d0 (float): Repulsion range in meters (0.55-1.2). Must be > d_stop + 0.20m
        k_att (float): Attraction strength (1.0-2.5). Higher = stronger goal pull
        v_max (float): Maximum velocity in m/s (0.5-1.5)

    Returns:
        RoverParams with custom tuning

    Examples:
        Aggressive (minimal obstacle avoidance):
        >>> custom(k_rep=0.10, d0=0.65, k_att=2.5, v_max=1.2)

        Moderate:
        >>> custom(k_rep=0.20, d0=0.80, k_att=1.5, v_max=0.8)

        Cautious:
        >>> custom(k_rep=0.35, d0=1.00, k_att=1.2, v_max=0.6)
    """
    # Enforce minimum d0 to prevent freezing
    min_d0 = 0.55  # Absolute minimum for stability
    if d0 < min_d0:
        print(f"WARNING: d0={d0:.2f}m too small, increasing to {min_d0}m")
        d0 = min_d0

    # Enforce minimum k_rep for functionality
    min_k_rep = 0.08
    if k_rep < min_k_rep:
        print(f"WARNING: k_rep={k_rep:.2f} too weak, increasing to {min_k_rep}")
        k_rep = min_k_rep

    # Calculate other parameters based on the key ones
    v0 = min(v_max * 0.75, 0.9)  # Nominal speed is 75% of max

    # Safety distances must satisfy: d0 > d_slow > d_safe > d_stop
    d_stop = 0.30 + (k_rep - 0.08) * 0.15  # Scale with repulsion strength
    d_safe = d_stop + 0.08
    d_slow = min(d0 * 0.9, d0 - 0.05)  # Just below d0

    # Ensure ordering
    d_stop = max(0.30, min(d_stop, d_safe - 0.05))
    d_safe = max(d_stop + 0.05, min(d_safe, d_slow - 0.05))
    d_slow = max(d_safe + 0.05, min(d_slow, d0 - 0.05))

    return RoverParams(
        # User-controlled parameters
        k_att=k_att,
        k_rep=k_rep,
        d0=d0,
        v_max=v_max,

        # Derived parameters
        d_switch=2.0,
        v0=v0,
        d_safe=d_safe,
        d_stop=d_stop,
        d_slow=d_slow,

        # Standard dynamic repulsion (not used for single rover)
        k_dyn=0.8,
        alpha=3.0,
        beta=0.5,
        dyn_range=3.0,

        # Control parameters
        w_max=2.0 if v_max > 0.9 else 1.5,
        k_w=2.5 if v_max > 0.9 else 2.0,
        w_slow=1.0 if v_max > 0.9 else 0.8,

        # Tangential bias
        bias_eta=0.0
    )
