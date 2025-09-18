"""
Real Simulation Interface for 2D Earth Moving Algorithm

This module provides the interface between the 2D strategic planning algorithm
and the real 3D simulation environment, enabling bidirectional communication
for state synchronization and move execution.
"""

import json
import math
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
from core_env import SimulationEnv


@dataclass
class AgentState:
    """Represents the state of an agent from the real simulation."""
    agent_id: int
    position: Tuple[float, float, float]  # (x, y, z) in 3D space
    orientation: float  # Orientation in degrees
    carrying_objects: int  # Number of objects currently being carried
    capacity: int  # Maximum carrying capacity
    status: str  # e.g., "idle", "moving", "collecting", "delivering"


@dataclass
class ObjectDistribution:
    """Represents object distribution in the real simulation."""
    cell_x: int
    cell_y: int
    num_objects: int
    object_type: str  # Type of objects (if applicable)


@dataclass
class MoveCommand:
    """Represents a move command to be sent to the real simulation."""
    agent_id: int
    source_cell: Tuple[int, int]
    target_cell: Tuple[int, int]
    path_type: str  # "target" or "highway"
    expected_objects: int  # Expected objects to collect
    priority: int  # Execution priority (1=highest)


@dataclass
class ValidationReport:
    """Report on consistency between real simulation and 2D algorithm."""
    is_consistent: bool
    discrepancies: List[str]
    severity_level: str  # "low", "medium", "high"
    recommended_action: str


class SimulationSyncMode(Enum):
    """Synchronization modes between real simulation and 2D algorithm."""
    FULL_SYNC = "full_sync"  # Complete state synchronization
    INCREMENTAL = "incremental"  # Only sync changes
    VALIDATION_ONLY = "validation_only"  # Just validate consistency
    IMPORT_ONLY = "import_only"  # Only import from real sim


class RealSimulationInterface:
    """
    Interface between 2D algorithm and real 3D simulation.
    
    Provides methods for:
    - Importing state from real simulation
    - Exporting move commands to real simulation
    - Validating state consistency
    - Synchronizing discrepancies
    """
    
    def __init__(self, grid_size: int, target_zone_radius: float):
        """
        Initialize the interface.
        
        Args:
            grid_size: Size of the 2D grid
            target_zone_radius: Radius of the target zone
        """
        self.grid_size = grid_size
        self.target_zone_radius = target_zone_radius
        self.last_sync_timestamp = None
        self.sync_history = []
        
    def import_from_real_simulation(self, real_sim_data: Dict[str, Any]) -> SimulationEnv:
        """
        Import current state from real simulation and create/update 2D environment.
        
        Args:
            real_sim_data: Dictionary containing real simulation state
            
        Returns:
            SimulationEnv: Updated 2D environment reflecting real simulation state
            
        Expected real_sim_data format:
        {
            'agents': [AgentState, ...],
            'objects': [ObjectDistribution, ...],
            'timestamp': float,
            'environment_config': {
                'grid_size': int,
                'target_zone_center': (x, y),
                'target_zone_radius': float
            }
        }
        """
        print(f"Importing state from real simulation (timestamp: {real_sim_data.get('timestamp', 'unknown')})")
        
        # Extract environment configuration
        env_config = real_sim_data.get('environment_config', {})
        grid_size = env_config.get('grid_size', self.grid_size)
        target_zone_radius = env_config.get('target_zone_radius', self.target_zone_radius)
        
        # Extract agent positions for 2D environment initialization
        agents_data = real_sim_data.get('agents', [])
        agent_positions = []
        for agent_state in agents_data:
            if isinstance(agent_state, dict):
                agent = AgentState(**agent_state)
            else:
                agent = agent_state
                
            # Convert 3D position to 2D (ignore z-coordinate)
            x_2d, y_2d = self._convert_3d_to_2d_position(agent.position)
            agent_positions.append((x_2d, y_2d, agent.orientation))
        
        # Create new 2D environment
        env_2d = SimulationEnv(
            grid_size=grid_size,
            target_zone_radius=target_zone_radius,
            agent_positions=agent_positions,
            num_random_objects=0  # Objects will be set from real simulation data
        )
        
        # Import object distribution
        objects_data = real_sim_data.get('objects', [])
        self._import_object_distribution(env_2d, objects_data)
        
        # Update agent carrying states
        self._update_agent_states(env_2d, agents_data)
        
        # Record sync timestamp
        self.last_sync_timestamp = real_sim_data.get('timestamp')
        
        print(f"Successfully imported state: {len(env_2d.cells_with_objects)} cells with objects, {len(agent_positions)} agents")
        
        return env_2d
    
    def update_from_real_simulation(self, env_2d: SimulationEnv, real_sim_data: Dict[str, Any]) -> bool:
        """
        Update existing 2D environment with changes from real simulation.
        
        Args:
            env_2d: Existing 2D environment to update
            real_sim_data: New state data from real simulation
            
        Returns:
            bool: True if update successful, False otherwise
        """
        try:
            print("Updating 2D environment from real simulation changes...")
            
            # Update object distribution
            objects_data = real_sim_data.get('objects', [])
            self._update_object_distribution(env_2d, objects_data)
            
            # Update agent states
            agents_data = real_sim_data.get('agents', [])
            self._update_agent_states(env_2d, agents_data)
            
            # Trigger selective environment update for efficiency
            env_2d.update_environment()
            
            self.last_sync_timestamp = real_sim_data.get('timestamp')
            print("2D environment successfully updated from real simulation")
            
            return True
            
        except Exception as e:
            print(f"Error updating 2D environment: {e}")
            return False
    
    def export_move_commands(self, moves: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Export move commands from 2D algorithm to format suitable for real simulation.
        
        Args:
            moves: List of move dictionaries from 2D algorithm
            
        Returns:
            Dictionary formatted for real simulation execution
            
        Expected moves format from 2D algorithm:
        [
            {
                'agent_id': int,
                'path': [Cell, Cell, ...],
                'path_type': 'target' or 'highway',
                'expected_objects': int,
                'estimated_time': float
            }
        ]
        """
        print(f"Exporting {len(moves)} move commands for real simulation")
        
        move_commands = []
        
        for i, move in enumerate(moves):
            # Extract path information
            path = move.get('path', [])
            if not path or len(path) < 2:
                continue
                
            source_cell = (path[0].x, path[0].y)
            target_cell = (path[-1].x, path[-1].y)
            
            # Create move command
            command = MoveCommand(
                agent_id=move.get('agent_id', 0),
                source_cell=source_cell,
                target_cell=target_cell,
                path_type=move.get('path_type', 'target'),
                expected_objects=move.get('expected_objects', 0),
                priority=i + 1  # Sequential priority
            )
            
            move_commands.append(command)
        
        # Format for real simulation
        export_data = {
            'commands': [self._move_command_to_dict(cmd) for cmd in move_commands],
            'timestamp': self._get_current_timestamp(),
            'sync_mode': SimulationSyncMode.FULL_SYNC.value,
            'metadata': {
                'total_commands': len(move_commands),
                'planning_algorithm': '2D_strategic_planner',
                'expected_total_objects': sum(cmd.expected_objects for cmd in move_commands)
            }
        }
        
        print(f"Successfully exported {len(move_commands)} move commands")
        return export_data
    
    def validate_state_consistency(self, 
                                 env_2d: SimulationEnv, 
                                 real_sim_data: Dict[str, Any]) -> ValidationReport:
        """
        Validate consistency between 2D algorithm state and real simulation state.
        
        Args:
            env_2d: 2D environment state
            real_sim_data: Real simulation state data
            
        Returns:
            ValidationReport: Detailed validation results
        """
        print("Validating state consistency between 2D algorithm and real simulation...")
        
        discrepancies = []
        severity = "low"
        
        try:
            # Validate object distribution
            objects_data = real_sim_data.get('objects', [])
            object_discrepancies = self._validate_object_distribution(env_2d, objects_data)
            discrepancies.extend(object_discrepancies)
            
            # Validate agent states  
            agents_data = real_sim_data.get('agents', [])
            agent_discrepancies = self._validate_agent_states(env_2d, agents_data)
            discrepancies.extend(agent_discrepancies)
            
            # Validate environment configuration
            env_config = real_sim_data.get('environment_config', {})
            config_discrepancies = self._validate_environment_config(env_2d, env_config)
            discrepancies.extend(config_discrepancies)
            
            # Determine severity level
            if len(discrepancies) == 0:
                severity = "low"
            elif len(discrepancies) <= 3:
                severity = "medium"
            else:
                severity = "high"
            
            # Determine recommended action
            if severity == "low":
                recommended_action = "Continue with current state"
            elif severity == "medium":
                recommended_action = "Consider incremental synchronization"
            else:
                recommended_action = "Full state synchronization required"
            
            is_consistent = len(discrepancies) == 0
            
            print(f"Validation complete: {'Consistent' if is_consistent else f'{len(discrepancies)} discrepancies found'}")
            
            return ValidationReport(
                is_consistent=is_consistent,
                discrepancies=discrepancies,
                severity_level=severity,
                recommended_action=recommended_action
            )
            
        except Exception as e:
            print(f"Error during validation: {e}")
            return ValidationReport(
                is_consistent=False,
                discrepancies=[f"Validation error: {e}"],
                severity_level="high",
                recommended_action="Full state synchronization required"
            )
    
    def synchronize_states(self, 
                          env_2d: SimulationEnv, 
                          real_sim_data: Dict[str, Any],
                          validation_report: ValidationReport,
                          sync_mode: SimulationSyncMode = SimulationSyncMode.FULL_SYNC) -> bool:
        """
        Synchronize states when discrepancies are found.
        
        Args:
            env_2d: 2D environment to synchronize
            real_sim_data: Real simulation state (authoritative source)
            validation_report: Results from validation
            sync_mode: Synchronization mode
            
        Returns:
            bool: True if synchronization successful
        """
        print(f"Synchronizing states (mode: {sync_mode.value}, discrepancies: {len(validation_report.discrepancies)})")
        
        if validation_report.is_consistent:
            print("States are already consistent, no synchronization needed")
            return True
        
        try:
            if sync_mode == SimulationSyncMode.FULL_SYNC:
                # Complete state synchronization - reimport everything
                return self.update_from_real_simulation(env_2d, real_sim_data)
            
            elif sync_mode == SimulationSyncMode.INCREMENTAL:
                # Selective synchronization based on discrepancies
                return self._incremental_synchronization(env_2d, real_sim_data, validation_report.discrepancies)
            
            else:
                print(f"Synchronization mode {sync_mode.value} not implemented for actual sync")
                return False
                
        except Exception as e:
            print(f"Error during synchronization: {e}")
            return False
    
    # Private helper methods
    
    def _convert_3d_to_2d_position(self, position_3d: Tuple[float, float, float]) -> Tuple[int, int]:
        """Convert 3D position to 2D grid coordinates."""
        x_3d, y_3d, z_3d = position_3d
        
        # Simple projection: ignore z-coordinate and discretize to grid
        x_2d = max(0, min(self.grid_size - 1, int(round(x_3d))))
        y_2d = max(0, min(self.grid_size - 1, int(round(y_3d))))
        
        return x_2d, y_2d
    
    def _import_object_distribution(self, env_2d: SimulationEnv, objects_data: List[Dict[str, Any]]):
        """Import object distribution into 2D environment."""
        print(f"Importing object distribution: {len(objects_data)} object locations")
        
        # Clear existing object distribution
        for cell in env_2d.all_cells:
            cell.num_objects = 0
            cell.current_objects = 0
        
        # Set new object distribution
        for obj_data in objects_data:
            if isinstance(obj_data, dict):
                obj_dist = ObjectDistribution(**obj_data)
            else:
                obj_dist = obj_data
                
            cell = env_2d.get_cell(obj_dist.cell_x, obj_dist.cell_y)
            if cell:
                cell.num_objects = obj_dist.num_objects
                cell.current_objects = obj_dist.num_objects
        
        # Update tracking lists
        env_2d._update_cells_with_objects_tracking()
    
    def _update_object_distribution(self, env_2d: SimulationEnv, objects_data: List[Dict[str, Any]]):
        """Update object distribution in existing 2D environment."""
        self._import_object_distribution(env_2d, objects_data)
    
    def _update_agent_states(self, env_2d: SimulationEnv, agents_data: List[Dict[str, Any]]):
        """Update agent states in 2D environment."""
        # Update agent positions and states
        for i, agent_data in enumerate(agents_data):
            if isinstance(agent_data, dict):
                agent = AgentState(**agent_data)
            else:
                agent = agent_data
                
            if i < len(env_2d.agents):
                x_2d, y_2d = self._convert_3d_to_2d_position(agent.position)
                env_2d.agents[i]["position"] = (x_2d, y_2d)
                env_2d.agents[i]["orientation"] = agent.orientation
                
                # Store additional agent state information
                env_2d.agents[i]["carrying_objects"] = agent.carrying_objects
                env_2d.agents[i]["capacity"] = agent.capacity
                env_2d.agents[i]["status"] = agent.status
    
    def _validate_object_distribution(self, env_2d: SimulationEnv, objects_data: List[Dict[str, Any]]) -> List[str]:
        """Validate object distribution consistency."""
        discrepancies = []
        
        # Create expected distribution map
        expected_objects = {}
        for obj_data in objects_data:
            if isinstance(obj_data, dict):
                obj_dist = ObjectDistribution(**obj_data)
            else:
                obj_dist = obj_data
                
            expected_objects[(obj_dist.cell_x, obj_dist.cell_y)] = obj_dist.num_objects
        
        # Compare with 2D environment
        for cell in env_2d.cells_with_objects:
            expected = expected_objects.get((cell.x, cell.y), 0)
            actual = cell.num_objects
            
            if expected != actual:
                discrepancies.append(f"Cell ({cell.x}, {cell.y}): expected {expected} objects, found {actual}")
        
        return discrepancies
    
    def _validate_agent_states(self, env_2d: SimulationEnv, agents_data: List[Dict[str, Any]]) -> List[str]:
        """Validate agent state consistency."""
        discrepancies = []
        
        if len(agents_data) != len(env_2d.agents):
            discrepancies.append(f"Agent count mismatch: expected {len(agents_data)}, found {len(env_2d.agents)}")
        
        for i, agent_data in enumerate(agents_data):
            if i >= len(env_2d.agents):
                break
                
            if isinstance(agent_data, dict):
                expected_agent = AgentState(**agent_data)
            else:
                expected_agent = agent_data
                
            actual_agent = env_2d.agents[i]
            
            expected_pos = self._convert_3d_to_2d_position(expected_agent.position)
            actual_pos = actual_agent["position"]
            
            if expected_pos != actual_pos:
                discrepancies.append(f"Agent {i} position: expected {expected_pos}, found {actual_pos}")
        
        return discrepancies
    
    def _validate_environment_config(self, env_2d: SimulationEnv, env_config: Dict[str, Any]) -> List[str]:
        """Validate environment configuration consistency."""
        discrepancies = []
        
        expected_grid_size = env_config.get('grid_size')
        if expected_grid_size and expected_grid_size != env_2d.grid_size:
            discrepancies.append(f"Grid size mismatch: expected {expected_grid_size}, found {env_2d.grid_size}")
        
        expected_target_radius = env_config.get('target_zone_radius')
        if expected_target_radius and abs(expected_target_radius - env_2d.target_zone_radius) > 0.1:
            discrepancies.append(f"Target zone radius mismatch: expected {expected_target_radius}, found {env_2d.target_zone_radius}")
        
        return discrepancies
    
    def _incremental_synchronization(self, env_2d: SimulationEnv, real_sim_data: Dict[str, Any], discrepancies: List[str]) -> bool:
        """Perform incremental synchronization based on specific discrepancies."""
        print("Performing incremental synchronization...")
        
        try:
            # Analyze discrepancies and apply targeted fixes
            for discrepancy in discrepancies:
                if "objects" in discrepancy.lower():
                    # Object-related discrepancy - update object distribution
                    objects_data = real_sim_data.get('objects', [])
                    self._update_object_distribution(env_2d, objects_data)
                
                elif "agent" in discrepancy.lower():
                    # Agent-related discrepancy - update agent states
                    agents_data = real_sim_data.get('agents', [])
                    self._update_agent_states(env_2d, agents_data)
            
            # Trigger selective environment update
            env_2d.update_environment()
            
            print("Incremental synchronization completed")
            return True
            
        except Exception as e:
            print(f"Error during incremental synchronization: {e}")
            return False
    
    def _move_command_to_dict(self, command: MoveCommand) -> Dict[str, Any]:
        """Convert MoveCommand to dictionary format."""
        return {
            'agent_id': command.agent_id,
            'source_cell': command.source_cell,
            'target_cell': command.target_cell,
            'path_type': command.path_type,
            'expected_objects': command.expected_objects,
            'priority': command.priority
        }
    
    def _get_current_timestamp(self) -> float:
        """Get current timestamp."""
        import time
        return time.time()


# Utility functions for external integration

def create_sample_real_sim_data(grid_size: int = 25) -> Dict[str, Any]:
    """
    Create sample real simulation data for testing.
    
    Args:
        grid_size: Size of the simulation grid
        
    Returns:
        Dictionary with sample real simulation state
    """
    return {
        'timestamp': 1234567890.0,
        'environment_config': {
            'grid_size': grid_size,
            'target_zone_center': (grid_size // 2, grid_size // 2),
            'target_zone_radius': 3.0
        },
        'agents': [
            {
                'agent_id': 0,
                'position': (5.0, 5.0, 0.0),
                'orientation': 45.0,
                'carrying_objects': 2,
                'capacity': 8,
                'status': 'moving'
            }
        ],
        'objects': [
            {'cell_x': 10, 'cell_y': 10, 'num_objects': 5, 'object_type': 'aggregate'},
            {'cell_x': 15, 'cell_y': 8, 'num_objects': 3, 'object_type': 'aggregate'},
            {'cell_x': 20, 'cell_y': 12, 'num_objects': 7, 'object_type': 'aggregate'}
        ]
    }


if __name__ == "__main__":
    # Example usage and testing
    print("Testing RealSimulationInterface...")
    
    # Create interface
    interface = RealSimulationInterface(grid_size=25, target_zone_radius=3.0)
    
    # Create sample data
    sample_data = create_sample_real_sim_data()
    
    # Import state
    env_2d = interface.import_from_real_simulation(sample_data)
    
    # Validate consistency
    validation_report = interface.validate_state_consistency(env_2d, sample_data)
    print(f"Validation result: {validation_report.is_consistent}")
    
    # Create sample moves for export
    sample_moves = [
        {
            'agent_id': 0,
            'path': [env_2d.get_cell(5, 5), env_2d.get_cell(10, 10)],
            'path_type': 'target',
            'expected_objects': 5,
            'estimated_time': 10.0
        }
    ]
    
    # Export moves
    export_data = interface.export_move_commands(sample_moves)
    print(f"Exported {len(export_data['commands'])} commands")
    
    print("RealSimulationInterface testing completed successfully!")