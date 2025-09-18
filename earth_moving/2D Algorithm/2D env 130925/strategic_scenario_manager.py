"""
Scenario Manager for Multi-Scenario Earth Moving Strategic Planning

This module manages multiple scenarios for strategic planning, enabling exploration
of different move sequences and their outcomes. It leverages the existing state
management capabilities in SimulationEnv for efficient scenario branching and rollback.
"""

import json
import time
import copy
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
import uuid
from pathlib import Path

from core_env import SimulationEnv


class ScenarioStatus(Enum):
    """Status of a scenario in the exploration tree."""
    ACTIVE = "active"          # Currently being explored
    COMPLETED = "completed"    # Exploration completed
    PAUSED = "paused"         # Temporarily paused
    FAILED = "failed"         # Failed during exploration
    ARCHIVED = "archived"     # Archived for storage


@dataclass
class ScenarioMetrics:
    """Performance metrics for a scenario."""
    total_moves: int = 0
    objects_transported: int = 0
    total_spillage: float = 0.0
    total_distance: float = 0.0
    total_time: float = 0.0
    efficiency_score: float = 0.0
    completion_percentage: float = 0.0
    
    def calculate_efficiency_score(self) -> float:
        """Calculate overall efficiency score for the scenario."""
        if self.total_moves == 0:
            return 0.0
            
        # Efficiency = (objects_transported / total_moves) * (1 - spillage_ratio) * (1 / normalized_distance)
        spillage_ratio = min(1.0, self.total_spillage / max(1, self.objects_transported))
        distance_factor = 1.0 / max(1.0, self.total_distance / max(1, self.total_moves))
        
        self.efficiency_score = (self.objects_transported / self.total_moves) * (1 - spillage_ratio) * distance_factor
        return self.efficiency_score


@dataclass
class MoveRecord:
    """Record of a single move within a scenario."""
    move_id: str
    timestamp: float
    agent_id: int
    source_cell: Tuple[int, int]
    target_cell: Tuple[int, int]
    path_type: str  # "target" or "highway"
    objects_collected: int
    spillage_amount: float
    distance_traveled: float
    execution_time: float
    pre_move_state_hash: str  # Hash of state before move
    post_move_state_hash: str  # Hash of state after move


@dataclass
class Scenario:
    """Represents a single scenario in the exploration tree."""
    scenario_id: str
    name: str
    parent_id: Optional[str] = None
    children_ids: List[str] = None
    status: ScenarioStatus = ScenarioStatus.ACTIVE
    
    # State management
    creation_timestamp: float = 0.0
    last_updated: float = 0.0
    initial_state: Optional[Dict[str, Any]] = None
    current_state: Optional[Dict[str, Any]] = None
    
    # Move history
    moves: List[MoveRecord] = None
    current_move_index: int = 0
    
    # Performance metrics
    metrics: ScenarioMetrics = None
    
    # Metadata
    description: str = ""
    tags: List[str] = None
    
    def __post_init__(self):
        """Initialize mutable default values."""
        if self.children_ids is None:
            self.children_ids = []
        if self.moves is None:
            self.moves = []
        if self.metrics is None:
            self.metrics = ScenarioMetrics()
        if self.tags is None:
            self.tags = []
        if self.creation_timestamp == 0.0:
            self.creation_timestamp = time.time()
        if self.last_updated == 0.0:
            self.last_updated = self.creation_timestamp


class ScenarioManager:
    """
    Manages multiple scenarios for strategic planning exploration.
    
    Features:
    - Scenario creation and branching
    - State preservation and rollback
    - Move history tracking
    - Performance analytics
    - Scenario comparison
    - Persistence to disk
    """
    
    def __init__(self, base_env: SimulationEnv, storage_path: Optional[str] = None):
        """
        Initialize the scenario manager.
        
        Args:
            base_env: Base environment for scenario creation
            storage_path: Optional path for persistent storage
        """
        self.base_env = base_env
        self.scenarios: Dict[str, Scenario] = {}
        self.active_scenario_id: Optional[str] = None
        self.storage_path = Path(storage_path) if storage_path else None
        
        # Create root scenario from base environment
        self.root_scenario_id = self._create_root_scenario()
        
        # Performance tracking
        self.scenario_creation_count = 0
        self.total_moves_explored = 0
        
        print(f"ScenarioManager initialized with root scenario: {self.root_scenario_id}")
    
    def create_scenario(self, 
                       parent_id: Optional[str] = None, 
                       name: Optional[str] = None,
                       description: str = "",
                       tags: List[str] = None) -> str:
        """
        Create a new scenario.
        
        Args:
            parent_id: ID of parent scenario (None for root scenario)
            name: Human-readable name for the scenario
            description: Optional description
            tags: Optional tags for categorization
            
        Returns:
            str: Unique scenario ID
        """
        scenario_id = str(uuid.uuid4())
        
        if name is None:
            name = f"Scenario_{self.scenario_creation_count + 1}"
        
        # Get parent scenario state for initialization
        if parent_id and parent_id in self.scenarios:
            parent_scenario = self.scenarios[parent_id]
            initial_state = copy.deepcopy(parent_scenario.current_state)
            
            # Add to parent's children
            parent_scenario.children_ids.append(scenario_id)
            parent_scenario.last_updated = time.time()
        else:
            # Use base environment state
            initial_state = self.base_env.get_state()
            parent_id = None
        
        # Create scenario
        scenario = Scenario(
            scenario_id=scenario_id,
            name=name,
            parent_id=parent_id,
            initial_state=initial_state,
            current_state=copy.deepcopy(initial_state),
            description=description,
            tags=tags if tags else []
        )
        
        self.scenarios[scenario_id] = scenario
        self.scenario_creation_count += 1
        
        print(f"Created scenario '{name}' (ID: {scenario_id}) with parent: {parent_id}")
        
        # Save to disk if storage enabled
        if self.storage_path:
            self._save_scenario_to_disk(scenario_id)
        
        return scenario_id
    
    def branch_scenario(self, 
                       source_scenario_id: str, 
                       branch_name: Optional[str] = None,
                       from_move_index: Optional[int] = None) -> str:
        """
        Create a branch scenario from an existing scenario at a specific point.
        
        Args:
            source_scenario_id: ID of scenario to branch from
            branch_name: Name for the new branch
            from_move_index: Move index to branch from (None for current state)
            
        Returns:
            str: ID of new branch scenario
        """
        if source_scenario_id not in self.scenarios:
            raise ValueError(f"Source scenario {source_scenario_id} not found")
        
        source_scenario = self.scenarios[source_scenario_id]
        
        if branch_name is None:
            branch_name = f"{source_scenario.name}_branch_{len(source_scenario.children_ids) + 1}"
        
        # Determine state to branch from
        if from_move_index is None or from_move_index >= len(source_scenario.moves):
            # Branch from current state
            branch_state = copy.deepcopy(source_scenario.current_state)
        else:
            # Branch from specific move - need to reconstruct state at that point
            branch_state = self._reconstruct_state_at_move(source_scenario_id, from_move_index)
        
        # Create branch scenario
        branch_id = str(uuid.uuid4())
        
        branch_scenario = Scenario(
            scenario_id=branch_id,
            name=branch_name,
            parent_id=source_scenario_id,
            initial_state=branch_state,
            current_state=copy.deepcopy(branch_state),
            description=f"Branch from {source_scenario.name} at move {from_move_index if from_move_index is not None else 'current'}"
        )
        
        # Update source scenario
        source_scenario.children_ids.append(branch_id)
        source_scenario.last_updated = time.time()
        
        self.scenarios[branch_id] = branch_scenario
        
        print(f"Created branch scenario '{branch_name}' (ID: {branch_id}) from '{source_scenario.name}'")
        
        return branch_id
    
    def set_active_scenario(self, scenario_id: str) -> bool:
        """
        Set the active scenario for operations.
        
        Args:
            scenario_id: ID of scenario to make active
            
        Returns:
            bool: True if successful
        """
        if scenario_id not in self.scenarios:
            print(f"Error: Scenario {scenario_id} not found")
            return False
        
        self.active_scenario_id = scenario_id
        scenario = self.scenarios[scenario_id]
        scenario.status = ScenarioStatus.ACTIVE
        scenario.last_updated = time.time()
        
        print(f"Set active scenario to '{scenario.name}' (ID: {scenario_id})")
        return True
    
    def get_scenario_environment(self, scenario_id: Optional[str] = None) -> SimulationEnv:
        """
        Get a SimulationEnv instance for the specified scenario.
        
        Args:
            scenario_id: Scenario ID (uses active scenario if None)
            
        Returns:
            SimulationEnv: Environment instance with scenario state
        """
        if scenario_id is None:
            scenario_id = self.active_scenario_id
            
        if scenario_id is None or scenario_id not in self.scenarios:
            raise ValueError(f"Invalid scenario ID: {scenario_id}")
        
        scenario = self.scenarios[scenario_id]
        
        # Create new environment instance with scenario state
        env = SimulationEnv(
            grid_size=scenario.current_state['grid_size'],
            target_zone_radius=scenario.current_state['target_zone_radius'],
            agent_positions=[],  # Will be restored from state
            num_random_objects=0  # Will be restored from state
        )
        
        # Restore scenario state
        env.set_state(scenario.current_state)
        
        return env
    
    def record_move(self, 
                   scenario_id: Optional[str],
                   move_data: Dict[str, Any],
                   post_move_env: SimulationEnv) -> str:
        """
        Record a move execution in the specified scenario.
        
        Args:
            scenario_id: Scenario ID (uses active if None)
            move_data: Move execution data
            post_move_env: Environment state after move execution
            
        Returns:
            str: Move record ID
        """
        if scenario_id is None:
            scenario_id = self.active_scenario_id
            
        if scenario_id is None or scenario_id not in self.scenarios:
            raise ValueError(f"Invalid scenario ID: {scenario_id}")
        
        scenario = self.scenarios[scenario_id]
        move_id = str(uuid.uuid4())
        
        # Create move record
        move_record = MoveRecord(
            move_id=move_id,
            timestamp=time.time(),
            agent_id=move_data.get('agent_id', 0),
            source_cell=move_data.get('source_cell', (0, 0)),
            target_cell=move_data.get('target_cell', (0, 0)),
            path_type=move_data.get('path_type', 'target'),
            objects_collected=move_data.get('objects_collected', 0),
            spillage_amount=move_data.get('spillage_amount', 0.0),
            distance_traveled=move_data.get('distance_traveled', 0.0),
            execution_time=move_data.get('execution_time', 0.0),
            pre_move_state_hash=self._calculate_state_hash(scenario.current_state),
            post_move_state_hash=self._calculate_state_hash(post_move_env.get_state())
        )
        
        # Update scenario
        scenario.moves.append(move_record)
        scenario.current_move_index = len(scenario.moves)
        scenario.current_state = post_move_env.get_state()
        scenario.last_updated = time.time()
        
        # Update metrics
        self._update_scenario_metrics(scenario, move_record)
        
        self.total_moves_explored += 1
        
        print(f"Recorded move {move_id} in scenario '{scenario.name}' (total moves: {len(scenario.moves)})")
        
        return move_id
    
    def rollback_to_move(self, scenario_id: str, move_index: int) -> bool:
        """
        Rollback scenario to a specific move index.
        
        Args:
            scenario_id: Scenario ID
            move_index: Move index to rollback to (0-based)
            
        Returns:
            bool: True if successful
        """
        if scenario_id not in self.scenarios:
            print(f"Error: Scenario {scenario_id} not found")
            return False
        
        scenario = self.scenarios[scenario_id]
        
        if move_index < 0 or move_index >= len(scenario.moves):
            print(f"Error: Invalid move index {move_index} for scenario with {len(scenario.moves)} moves")
            return False
        
        # Reconstruct state at the specified move
        target_state = self._reconstruct_state_at_move(scenario_id, move_index)
        
        if target_state is None:
            print(f"Error: Could not reconstruct state at move {move_index}")
            return False
        
        # Update scenario
        scenario.current_state = target_state
        scenario.current_move_index = move_index
        scenario.last_updated = time.time()
        
        # Trim moves after rollback point
        scenario.moves = scenario.moves[:move_index]
        
        # Recalculate metrics
        self._recalculate_scenario_metrics(scenario)
        
        print(f"Rolled back scenario '{scenario.name}' to move {move_index}")
        return True
    
    def compare_scenarios(self, scenario_ids: List[str]) -> Dict[str, Any]:
        """
        Compare multiple scenarios across different metrics.
        
        Args:
            scenario_ids: List of scenario IDs to compare
            
        Returns:
            Dictionary with comparison results
        """
        if not scenario_ids:
            return {}
        
        valid_scenarios = [self.scenarios[sid] for sid in scenario_ids if sid in self.scenarios]
        
        if not valid_scenarios:
            return {'error': 'No valid scenarios provided'}
        
        comparison = {
            'scenarios': {},
            'summary': {
                'total_scenarios': len(valid_scenarios),
                'best_efficiency': None,
                'best_completion': None,
                'least_spillage': None,
                'shortest_time': None
            }
        }
        
        best_efficiency = 0
        best_completion = 0
        least_spillage = float('inf')
        shortest_time = float('inf')
        
        for scenario in valid_scenarios:
            metrics = scenario.metrics
            scenario_data = {
                'name': scenario.name,
                'status': scenario.status.value,
                'moves': len(scenario.moves),
                'metrics': asdict(metrics),
                'completion_date': scenario.last_updated
            }
            
            comparison['scenarios'][scenario.scenario_id] = scenario_data
            
            # Track bests
            if metrics.efficiency_score > best_efficiency:
                best_efficiency = metrics.efficiency_score
                comparison['summary']['best_efficiency'] = scenario.scenario_id
            
            if metrics.completion_percentage > best_completion:
                best_completion = metrics.completion_percentage
                comparison['summary']['best_completion'] = scenario.scenario_id
            
            if metrics.total_spillage < least_spillage:
                least_spillage = metrics.total_spillage
                comparison['summary']['least_spillage'] = scenario.scenario_id
            
            if metrics.total_time < shortest_time:
                shortest_time = metrics.total_time
                comparison['summary']['shortest_time'] = scenario.scenario_id
        
        return comparison
    
    def get_scenario_tree(self) -> Dict[str, Any]:
        """
        Get the complete scenario exploration tree.
        
        Returns:
            Dictionary representing the scenario tree structure
        """
        def build_tree_node(scenario_id: str) -> Dict[str, Any]:
            scenario = self.scenarios[scenario_id]
            
            node = {
                'scenario_id': scenario_id,
                'name': scenario.name,
                'status': scenario.status.value,
                'moves': len(scenario.moves),
                'efficiency_score': scenario.metrics.efficiency_score,
                'completion_percentage': scenario.metrics.completion_percentage,
                'children': []
            }
            
            for child_id in scenario.children_ids:
                if child_id in self.scenarios:
                    node['children'].append(build_tree_node(child_id))
            
            return node
        
        # Build tree starting from root
        return build_tree_node(self.root_scenario_id)
    
    def save_all_scenarios(self, filepath: str) -> bool:
        """
        Save all scenarios to a file.
        
        Args:
            filepath: Path to save file
            
        Returns:
            bool: True if successful
        """
        try:
            export_data = {
                'scenarios': {},
                'metadata': {
                    'creation_time': time.time(),
                    'total_scenarios': len(self.scenarios),
                    'root_scenario_id': self.root_scenario_id,
                    'active_scenario_id': self.active_scenario_id
                }
            }
            
            # Convert scenarios to serializable format
            for scenario_id, scenario in self.scenarios.items():
                scenario_dict = self._make_serializable(asdict(scenario))
                export_data['scenarios'][scenario_id] = scenario_dict
            
            with open(filepath, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            print(f"Saved {len(self.scenarios)} scenarios to {filepath}")
            return True
            
        except Exception as e:
            print(f"Error saving scenarios: {e}")
            return False
    
    def load_scenarios(self, filepath: str) -> bool:
        """
        Load scenarios from a file.
        
        Args:
            filepath: Path to load file
            
        Returns:
            bool: True if successful
        """
        try:
            with open(filepath, 'r') as f:
                import_data = json.load(f)
            
            # Clear existing scenarios
            self.scenarios.clear()
            
            # Load scenarios
            for scenario_id, scenario_data in import_data['scenarios'].items():
                # Fix deserialized data structure
                fixed_data = self._fix_deserialized_data(scenario_data)
                
                # Convert back to Scenario object
                scenario = Scenario(**fixed_data)
                # Handle enum conversion
                if isinstance(scenario.status, str):
                    scenario.status = ScenarioStatus(scenario.status)
                # Handle nested dataclass
                if isinstance(scenario.metrics, dict):
                    scenario.metrics = ScenarioMetrics(**scenario.metrics)
                # Handle move records
                if scenario.moves:
                    fixed_moves = []
                    for move_data in scenario.moves:
                        if isinstance(move_data, dict):
                            # Convert move data back to MoveRecord
                            fixed_move_data = self._fix_deserialized_data(move_data)
                            move_record = MoveRecord(**fixed_move_data)
                            fixed_moves.append(move_record)
                        else:
                            fixed_moves.append(move_data)
                    scenario.moves = fixed_moves
                
                self.scenarios[scenario_id] = scenario
            
            # Restore metadata
            metadata = import_data.get('metadata', {})
            self.root_scenario_id = metadata.get('root_scenario_id')
            self.active_scenario_id = metadata.get('active_scenario_id')
            
            print(f"Loaded {len(self.scenarios)} scenarios from {filepath}")
            return True
            
        except Exception as e:
            print(f"Error loading scenarios: {e}")
            return False
    
    # Private helper methods
    
    def _create_root_scenario(self) -> str:
        """Create the root scenario from base environment."""
        root_id = "root_" + str(uuid.uuid4())
        
        root_scenario = Scenario(
            scenario_id=root_id,
            name="Root Scenario",
            parent_id=None,
            initial_state=self.base_env.get_state(),
            current_state=self.base_env.get_state(),
            description="Initial state of the environment"
        )
        
        self.scenarios[root_id] = root_scenario
        return root_id
    
    def _reconstruct_state_at_move(self, scenario_id: str, move_index: int) -> Optional[Dict[str, Any]]:
        """
        Reconstruct environment state at a specific move index.
        
        This is a simplified version - in practice, you might want to store
        state snapshots at key points for efficiency.
        """
        scenario = self.scenarios[scenario_id]
        
        if move_index == 0:
            return copy.deepcopy(scenario.initial_state)
        
        # For now, return the current state (would need full replay implementation)
        # TODO: Implement full state replay from move history
        print(f"Warning: State reconstruction not fully implemented - returning current state")
        return copy.deepcopy(scenario.current_state)
    
    def _update_scenario_metrics(self, scenario: Scenario, move_record: MoveRecord):
        """Update scenario metrics with new move data."""
        metrics = scenario.metrics
        
        metrics.total_moves += 1
        metrics.objects_transported += move_record.objects_collected
        metrics.total_spillage += move_record.spillage_amount
        metrics.total_distance += move_record.distance_traveled
        metrics.total_time += move_record.execution_time
        
        # Calculate completion percentage (simplified)
        # TODO: Implement based on actual objective completion
        if metrics.total_moves > 0:
            metrics.completion_percentage = min(100.0, (metrics.objects_transported / max(1, metrics.total_moves)) * 10)
        
        # Update efficiency score
        metrics.calculate_efficiency_score()
    
    def _recalculate_scenario_metrics(self, scenario: Scenario):
        """Recalculate scenario metrics from scratch."""
        metrics = ScenarioMetrics()
        
        for move in scenario.moves:
            metrics.total_moves += 1
            metrics.objects_transported += move.objects_collected
            metrics.total_spillage += move.spillage_amount
            metrics.total_distance += move.distance_traveled
            metrics.total_time += move.execution_time
        
        # Calculate completion percentage and efficiency
        if metrics.total_moves > 0:
            metrics.completion_percentage = min(100.0, (metrics.objects_transported / max(1, metrics.total_moves)) * 10)
        
        metrics.calculate_efficiency_score()
        scenario.metrics = metrics
    
    def _calculate_state_hash(self, state: Dict[str, Any]) -> str:
        """Calculate a hash of the environment state for tracking changes."""
        import hashlib
        
        # Create a simplified state representation for hashing
        hash_data = {
            'grid_size': state['grid_size'],
            'cells_with_objects': state['cells_with_objects_coords'],
            'target_zone_cells': state['target_zone_cells_coords']
        }
        
        state_str = json.dumps(hash_data, sort_keys=True)
        return hashlib.md5(state_str.encode()).hexdigest()
    
    def _make_serializable(self, obj):
        """Convert object to JSON-serializable format."""
        if isinstance(obj, dict):
            # Handle dictionary with tuple keys
            new_dict = {}
            for key, value in obj.items():
                # Convert tuple keys to string representation
                if isinstance(key, tuple):
                    str_key = f"{key[0]},{key[1]}"  # Convert (x,y) to "x,y"
                    new_dict[str_key] = self._make_serializable(value)
                else:
                    new_dict[str(key)] = self._make_serializable(value)
            return new_dict
        elif isinstance(obj, list):
            return [self._make_serializable(item) for item in obj]
        elif isinstance(obj, tuple):
            return list(obj)  # Convert tuples to lists
        elif hasattr(obj, '__dict__'):
            return self._make_serializable(obj.__dict__)
        elif isinstance(obj, (str, int, float, bool, type(None))):
            return obj
        else:
            return str(obj)  # Convert everything else to string
    
    def _fix_deserialized_data(self, data):
        """Fix deserialized data to restore proper types."""
        if isinstance(data, dict):
            fixed = {}
            for key, value in data.items():
                # Convert string keys back to tuple keys for cell_states
                if key == 'cell_states' and isinstance(value, dict):
                    # Convert cell_states keys from "x,y" back to (x,y) tuples
                    cell_states_fixed = {}
                    for cell_key, cell_value in value.items():
                        if ',' in str(cell_key):
                            try:
                                x, y = map(int, str(cell_key).split(','))
                                cell_states_fixed[(x, y)] = self._fix_deserialized_data(cell_value)
                            except ValueError:
                                cell_states_fixed[cell_key] = self._fix_deserialized_data(cell_value)
                        else:
                            cell_states_fixed[cell_key] = self._fix_deserialized_data(cell_value)
                    fixed[key] = cell_states_fixed
                # Convert coordinate lists back to tuples
                elif key in ['source_cell', 'target_cell'] and isinstance(value, list) and len(value) == 2:
                    fixed[key] = tuple(value)
                else:
                    fixed[key] = self._fix_deserialized_data(value)
            return fixed
        elif isinstance(data, list):
            return [self._fix_deserialized_data(item) for item in data]
        else:
            return data
    
    def _save_scenario_to_disk(self, scenario_id: str):
        """Save individual scenario to disk."""
        if not self.storage_path:
            return
        
        scenario = self.scenarios[scenario_id]
        scenario_file = self.storage_path / f"scenario_{scenario_id}.json"
        
        try:
            scenario_dict = self._make_serializable(asdict(scenario))
            with open(scenario_file, 'w') as f:
                json.dump(scenario_dict, f, indent=2)
        except Exception as e:
            print(f"Error saving scenario {scenario_id} to disk: {e}")


if __name__ == "__main__":
    # Example usage and testing
    print("Testing ScenarioManager...")
    
    # Create a mock environment for testing
    from core_env import SimulationEnv
    
    base_env = SimulationEnv(
        grid_size=10,
        target_zone_radius=2,
        agent_positions=[(2, 2, 0)],
        num_random_objects=5,
        seed=42
    )
    
    # Create scenario manager
    manager = ScenarioManager(base_env)
    
    # Create some scenarios
    scenario1 = manager.create_scenario(name="Greedy Strategy")
    scenario2 = manager.branch_scenario(scenario1, "Conservative Branch")
    
    # Set active scenario and get environment
    manager.set_active_scenario(scenario1)
    env = manager.get_scenario_environment()
    
    # Simulate recording moves
    sample_move = {
        'agent_id': 0,
        'source_cell': (2, 2),
        'target_cell': (5, 5),
        'path_type': 'target',
        'objects_collected': 3,
        'spillage_amount': 0.1,
        'distance_traveled': 4.2,
        'execution_time': 2.5
    }
    
    manager.record_move(scenario1, sample_move, env)
    
    # Compare scenarios
    comparison = manager.compare_scenarios([scenario1, scenario2])
    print(f"Scenario comparison: {len(comparison['scenarios'])} scenarios compared")
    
    # Get scenario tree
    tree = manager.get_scenario_tree()
    print(f"Scenario tree created with root: {tree['name']}")
    
    print("ScenarioManager testing completed successfully!")