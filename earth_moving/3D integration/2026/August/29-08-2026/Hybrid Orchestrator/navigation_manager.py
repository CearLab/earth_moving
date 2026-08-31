"""
navigation_manager.py - Navigation Mode Manager for Hybrid Orchestrator

This module coordinates navigation modes:
- IDLE: No active navigation
- ORCA_NAVIGATE: Using ORCA to navigate to path start
- PATH_TRACK: Following A* path precisely

Two-phase trajectory execution:
1. ORCA navigation from current position to path[0]
2. Path tracking along A* path until completion
"""

import math
from enum import Enum
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from path_tracker import PathTracker
from orca_navigator import ORCANavigator, sailing_priority, NAV_FIELD_CONFIG, SHOVEL_TRACK_OFFSET


class NavigationMode(Enum):
    """Navigation mode states."""
    IDLE = "idle"
    ORCA_NAVIGATE = "orca_navigate"
    PATH_TRACK = "path_track"


@dataclass
class NavigationStatus:
    """Status of current navigation."""
    mode: NavigationMode
    v_cmd: float
    w_cmd: float
    progress: float  # 0.0-1.0 for path tracking
    dist_to_goal: float
    completed: bool
    message: str


class NavigationManager:
    """
    Coordinates navigation modes for the hybrid orchestrator.

    Manages transitions between:
    - IDLE -> ORCA_NAVIGATE (when navigate_to_path_start called)
    - ORCA_NAVIGATE -> PATH_TRACK (when arrived at path start)
    - PATH_TRACK -> IDLE (when path completed)
    """

    def __init__(self,
                 world_bounds=(-3.5, 3.5, -3.5, 3.5),
                 grid_size=(81, 81),
                 rover_radius=0.25,
                 pebble_radius=0.05,
                 target_zone_radius=0.3,
                 phase1_tracking_point="shovel"):
        """
        Initialize navigation manager.

        Args:
            world_bounds: (xmin, xmax, ymin, ymax)
            grid_size: (grid_w, grid_h)
            rover_radius: rover safety radius
            pebble_radius: pebble radius
        """
        self.world_bounds = world_bounds
        self.grid_size = grid_size
        self.rover_radius = rover_radius
        self.pebble_radius = pebble_radius
        self.target_zone_radius = target_zone_radius
        self.phase1_tracking_point = phase1_tracking_point

        # Navigation components
        self.orca_navigator = ORCANavigator(
            world_bounds=world_bounds,
            grid_size=grid_size,
            tau=5.0,
            v_max=1.5,
            rover_radius=NAV_FIELD_CONFIG["rover_radius"],
            pebble_radius=pebble_radius,
            arrival_threshold=0.15,
            target_zone_radius=target_zone_radius,
            phase1_tracking_point=phase1_tracking_point,
            shovel_offset=SHOVEL_TRACK_OFFSET,
        )

        self.path_tracker = PathTracker(
            world_bounds=world_bounds,
            grid_size=grid_size,
            k_t=2.0,
            k_n=10.0,
            band_radius=0.5,
            v_max=1.0,
            w_max=6.0,
            shovel_offset=0.17,
            rover_radius=rover_radius,
            pebble_radius=pebble_radius,
        )

        # State
        self.mode = NavigationMode.IDLE
        self.current_path = None
        self.pebble_centers = []
        self.agent = None

    def set_agent(self, agent):
        """
        Set the agent to control.

        Args:
            agent: agent dict with "state", "shape", etc.
        """
        self.agent = agent

    def update_pebbles(self, pebble_centers):
        """
        Update pebble positions.

        Args:
            pebble_centers: list of (x, y) pebble positions
        """
        self.pebble_centers = list(pebble_centers)

    def start_navigation(self, path_points, pebble_centers=None):
        """
        Start two-phase navigation to execute a path.

        Phase 1: ORCA navigate to path[0]
        Phase 2: Path track along path

        Args:
            path_points: list of (x, y) waypoints from A* planner
            pebble_centers: optional updated pebble positions

        Returns:
            True if navigation started, False otherwise
        """
        if len(path_points) < 2:
            print("[NavigationManager] Error: path must have at least 2 points")
            return False

        if self.agent is None:
            print("[NavigationManager] Error: no agent set")
            return False

        self.current_path = [(pt[0], pt[1]) for pt in path_points]

        if pebble_centers is not None:
            self.pebble_centers = list(pebble_centers)

        # Build ORCA field to path start
        path_start = self.current_path[0]
        self.orca_navigator.build_field(path_start, self.pebble_centers)

        # Check if already at path start
        x, y = self.orca_navigator.get_tracking_position(self.agent["state"])
        dist_to_start = math.hypot(x - path_start[0], y - path_start[1])

        if dist_to_start < self.orca_navigator.arrival_threshold:
            # Skip ORCA phase, go directly to path tracking
            print(f"[NavigationManager] Already at path start (dist={dist_to_start:.2f}m), starting path tracking")
            self._start_path_tracking()
        else:
            # Start ORCA navigation to path start
            print(f"[NavigationManager] Starting ORCA navigation to path start (dist={dist_to_start:.2f}m)")
            self.mode = NavigationMode.ORCA_NAVIGATE
            # Update sailing priority
            yaw = self.agent["state"][2]
            self.agent["priority"] = sailing_priority(yaw)

        return True

    def _start_path_tracking(self):
        """Start path tracking phase."""
        if self.current_path is None or len(self.current_path) < 2:
            print("[NavigationManager] Error: no valid path for tracking")
            self.mode = NavigationMode.IDLE
            return

        tracking_path = list(self.current_path)
        if self.agent is not None and len(tracking_path) >= 2:
            x_track, y_track = self.orca_navigator.get_tracking_position(self.agent["state"])
            tracking_path = [(float(x_track), float(y_track))] + tracking_path[1:]

        self.current_path = tracking_path
        self.path_tracker.build_field(self.current_path, self.pebble_centers)
        self.mode = NavigationMode.PATH_TRACK
        print(f"[NavigationManager] Started path tracking ({len(self.current_path)} waypoints)")

    def step(self, dt=0.05, other_agents=None) -> NavigationStatus:
        """
        Execute one navigation step.

        Args:
            dt: time step for progress computation
            other_agents: list of other agents for ORCA (multi-agent support)

        Returns:
            NavigationStatus with mode, commands, progress, etc.
        """
        if self.agent is None:
            return NavigationStatus(
                mode=NavigationMode.IDLE,
                v_cmd=0.0,
                w_cmd=0.0,
                progress=0.0,
                dist_to_goal=float("inf"),
                completed=False,
                message="No agent set"
            )

        if self.mode == NavigationMode.IDLE:
            return NavigationStatus(
                mode=NavigationMode.IDLE,
                v_cmd=0.0,
                w_cmd=0.0,
                progress=0.0,
                dist_to_goal=float("inf"),
                completed=False,
                message="Idle"
            )

        elif self.mode == NavigationMode.ORCA_NAVIGATE:
            return self._step_orca_navigate(dt, other_agents)

        elif self.mode == NavigationMode.PATH_TRACK:
            return self._step_path_track(dt)

        return NavigationStatus(
            mode=self.mode,
            v_cmd=0.0,
            w_cmd=0.0,
            progress=0.0,
            dist_to_goal=float("inf"),
            completed=False,
            message="Unknown mode"
        )

    def _step_orca_navigate(self, dt, other_agents) -> NavigationStatus:
        """Execute one ORCA navigation step."""
        # Update sailing priority
        yaw = self.agent["state"][2]
        self.agent["priority"] = sailing_priority(yaw)

        result = self.orca_navigator.compute_control(
            agent=self.agent,
            dt=dt,
            other_agents=other_agents,
        )

        if result["arrived"]:
            print(f"[NavigationManager] Arrived at path start (dist={result['dist_to_goal']:.3f}m)")
            self._start_path_tracking()
            return NavigationStatus(
                mode=NavigationMode.ORCA_NAVIGATE,
                v_cmd=0.0,
                w_cmd=0.0,
                progress=0.0,
                dist_to_goal=result["dist_to_goal"],
                completed=False,
                message="Arrived at path start, transitioning to path tracking"
            )

        return NavigationStatus(
            mode=NavigationMode.ORCA_NAVIGATE,
            v_cmd=result["v_cmd"],
            w_cmd=result["w_cmd"],
            progress=0.0,
            dist_to_goal=result["dist_to_goal"],
            completed=False,
            message=f"ORCA: dist={result['dist_to_goal']:.2f}m"
        )

    def _step_path_track(self, dt) -> NavigationStatus:
        """Execute one path tracking step."""
        result = self.path_tracker.compute_control(
            state=self.agent["state"],
            dt=dt,
        )

        if result["completed"]:
            print(f"[NavigationManager] Path tracking complete (progress={result['progress']:.2f})")
            self.mode = NavigationMode.IDLE
            self.current_path = None
            return NavigationStatus(
                mode=NavigationMode.PATH_TRACK,
                v_cmd=0.0,
                w_cmd=0.0,
                progress=1.0,
                dist_to_goal=0.0,
                completed=True,
                message="Path completed"
            )

        return NavigationStatus(
            mode=NavigationMode.PATH_TRACK,
            v_cmd=result["v_cmd"],
            w_cmd=result["w_cmd"],
            progress=result["progress"],
            dist_to_goal=result["total_L"] - result["s_now"],
            completed=False,
            message=f"Path: {result['progress']*100:.1f}% (d={result['d_now']:.3f}m)"
        )

    def cancel(self):
        """Cancel current navigation and return to IDLE."""
        print("[NavigationManager] Navigation cancelled")
        self.mode = NavigationMode.IDLE
        self.current_path = None

    def is_idle(self) -> bool:
        """Check if navigation is idle."""
        return self.mode == NavigationMode.IDLE

    def is_navigating(self) -> bool:
        """Check if actively navigating."""
        return self.mode != NavigationMode.IDLE

    def get_mode(self) -> NavigationMode:
        """Get current navigation mode."""
        return self.mode

    def draw_debug(self, scale=0.2, life_time=0.0):
        """Draw debug visualization in PyBullet."""
        if self.mode == NavigationMode.ORCA_NAVIGATE:
            self.orca_navigator.draw_debug(scale=scale, life_time=life_time)
        elif self.mode == NavigationMode.PATH_TRACK:
            self.path_tracker.draw_debug(scale=scale, life_time=life_time)
