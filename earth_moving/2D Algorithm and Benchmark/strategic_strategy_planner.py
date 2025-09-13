"""
Strategy Planner for Sequential Move Planning in Earth Moving Operations

This module implements various strategic planning algorithms for determining
optimal sequences of moves to transport all aggregates to the target zone.
It leverages scenario exploration and multi-objective optimization.
"""

import math
import time
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass
from enum import Enum
import heapq
import copy

from core_env import SimulationEnv
from strategic_scenario_manager import ScenarioManager, Scenario


class PlanningStrategy(Enum):
    """Available planning strategies for move sequence optimization."""
    GREEDY_NEAREST = "greedy_nearest"                # Always collect closest/highest objects
    GREEDY_EFFICIENT = "greedy_efficient"            # Prioritize efficiency (objects/distance ratio)
    HIGHWAY_FORMATION = "highway_formation"          # Build highways then utilize them
    COORDINATED_SWEEP = "coordinated_sweep"          # Multi-agent coordinated collection
    SPILLAGE_MINIMIZATION = "spillage_minimization"  # Minimize total spillage losses
    BALANCED_OPTIMIZATION = "balanced_optimization"   # Balance multiple objectives
    ADAPTIVE_HYBRID = "adaptive_hybrid"              # Adapt strategy based on current state


class ObjectiveType(Enum):
    """Types of optimization objectives."""
    MINIMIZE_MOVES = "minimize_moves"              # Minimize total number of moves
    MINIMIZE_TIME = "minimize_time"                # Minimize total execution time
    MINIMIZE_SPILLAGE = "minimize_spillage"        # Minimize object spillage
    MAXIMIZE_EFFICIENCY = "maximize_efficiency"    # Maximize objects/move ratio
    MINIMIZE_DISTANCE = "minimize_distance"        # Minimize total travel distance
    MAXIMIZE_HIGHWAY_USAGE = "maximize_highway_usage"  # Prefer highway formation


@dataclass
class ObjectiveWeight:
    """Weight configuration for multi-objective optimization."""
    objective: ObjectiveType
    weight: float
    target_value: Optional[float] = None  # Target value for normalization


@dataclass
class PlanningConstraint:
    """Constraint for planning optimization."""
    constraint_type: str  # "max_moves", "max_time", "max_spillage", etc.
    value: float
    is_hard: bool = True  # Hard constraint (must satisfy) vs soft (penalty)


@dataclass
class MoveCandidate:
    """Candidate move for planning consideration."""
    agent_id: int
    source_cell: Tuple[int, int]
    target_path: List[Tuple[int, int]]
    path_type: str  # "target" or "highway"
    
    # Expected outcomes
    expected_objects: int
    expected_spillage: float
    expected_distance: float
    expected_time: float
    
    # Scoring
    raw_score: float = 0.0
    normalized_score: float = 0.0
    
    # Additional metadata
    priority: int = 0
    conflicts: List[int] = None  # Agent IDs that conflict with this move
    
    def __post_init__(self):
        if self.conflicts is None:
            self.conflicts = []


@dataclass
class PlanningResult:
    """Result of a planning operation."""
    strategy: PlanningStrategy
    move_sequence: List[MoveCandidate]
    total_moves: int
    estimated_objects: int
    estimated_spillage: float
    estimated_distance: float
    estimated_time: float
    
    # Multi-objective scores
    objective_scores: Dict[ObjectiveType, float]
    overall_score: float
    
    # Planning metadata
    planning_time: float
    scenarios_explored: int
    convergence_achieved: bool


class StrategyPlanner:
    """
    Strategic planner for sequential move planning in earth moving operations.
    
    Features:
    - Multiple planning strategies
    - Multi-objective optimization
    - Constraint satisfaction
    - Scenario-based exploration
    - Adaptive strategy selection
    """
    
    def __init__(self, scenario_manager: ScenarioManager):
        """
        Initialize the strategy planner.
        
        Args:
            scenario_manager: ScenarioManager for scenario exploration
        """
        self.scenario_manager = scenario_manager
        self.planning_cache = {}  # Cache for computed plans
        self.strategy_performance = {}  # Performance tracking per strategy
        
        # Default objective weights (can be customized)
        self.default_objectives = [
            ObjectiveWeight(ObjectiveType.MINIMIZE_MOVES, 0.3),
            ObjectiveWeight(ObjectiveType.MAXIMIZE_EFFICIENCY, 0.3),
            ObjectiveWeight(ObjectiveType.MINIMIZE_SPILLAGE, 0.2),
            ObjectiveWeight(ObjectiveType.MINIMIZE_TIME, 0.2)
        ]
        
        print("StrategyPlanner initialized")
    
    def plan_complete_transportation(self, 
                                   scenario_id: str,
                                   strategy: PlanningStrategy = PlanningStrategy.BALANCED_OPTIMIZATION,
                                   objectives: List[ObjectiveWeight] = None,
                                   constraints: List[PlanningConstraint] = None,
                                   max_planning_time: float = 300.0,
                                   lookahead_depth: int = 3) -> PlanningResult:
        """
        Plan complete transportation of all objects to target zone.
        
        Args:
            scenario_id: Scenario to plan for
            strategy: Planning strategy to use
            objectives: Multi-objective weights
            constraints: Planning constraints
            max_planning_time: Maximum planning time in seconds
            lookahead_depth: How many moves to look ahead
            
        Returns:
            PlanningResult: Complete planning result
        """
        start_time = time.time()
        
        if objectives is None:
            objectives = self.default_objectives
        
        print(f"Planning complete transportation using {strategy.value} strategy")
        print(f"Lookahead depth: {lookahead_depth}, Max time: {max_planning_time}s")
        
        # Get current environment state
        env = self.scenario_manager.get_scenario_environment(scenario_id)
        
        # Initialize planning result
        result = PlanningResult(
            strategy=strategy,
            move_sequence=[],
            total_moves=0,
            estimated_objects=0,
            estimated_spillage=0.0,
            estimated_distance=0.0,
            estimated_time=0.0,
            objective_scores={},
            overall_score=0.0,
            planning_time=0.0,
            scenarios_explored=0,
            convergence_achieved=False
        )
        
        # Check if all objects are already transported
        if self._is_transportation_complete(env):
            print("Transportation already complete!")
            result.planning_time = time.time() - start_time
            return result
        
        # Execute strategy-specific planning
        if strategy == PlanningStrategy.GREEDY_NEAREST:
            result = self._plan_greedy_nearest(scenario_id, env, objectives, constraints, max_planning_time, result)
        elif strategy == PlanningStrategy.GREEDY_EFFICIENT:
            result = self._plan_greedy_efficient(scenario_id, env, objectives, constraints, max_planning_time, result)
        elif strategy == PlanningStrategy.HIGHWAY_FORMATION:
            result = self._plan_highway_formation(scenario_id, env, objectives, constraints, max_planning_time, result)
        elif strategy == PlanningStrategy.SPILLAGE_MINIMIZATION:
            result = self._plan_spillage_minimization(scenario_id, env, objectives, constraints, max_planning_time, result)
        elif strategy == PlanningStrategy.BALANCED_OPTIMIZATION:
            result = self._plan_balanced_optimization(scenario_id, env, objectives, constraints, max_planning_time, result)
        elif strategy == PlanningStrategy.ADAPTIVE_HYBRID:
            result = self._plan_adaptive_hybrid(scenario_id, env, objectives, constraints, max_planning_time, result)
        else:
            print(f"Strategy {strategy.value} not implemented, using balanced optimization")
            result = self._plan_balanced_optimization(scenario_id, env, objectives, constraints, max_planning_time, result)
        
        # Calculate final scores
        result = self._calculate_objective_scores(result, objectives)
        result.planning_time = time.time() - start_time
        
        print(f"Planning completed in {result.planning_time:.2f}s: {result.total_moves} moves, score: {result.overall_score:.3f}")
        
        # Update strategy performance tracking
        self._update_strategy_performance(strategy, result)
        
        return result
    
    def optimize_next_n_moves(self, 
                             scenario_id: str,
                             n: int = 5,
                             strategy: PlanningStrategy = PlanningStrategy.BALANCED_OPTIMIZATION) -> List[MoveCandidate]:
        """
        Optimize the next N moves using scenario exploration.
        
        Args:
            scenario_id: Scenario to optimize for
            n: Number of moves to optimize
            strategy: Strategy to use
            
        Returns:
            List of optimized move candidates
        """
        print(f"Optimizing next {n} moves using {strategy.value}")
        
        env = self.scenario_manager.get_scenario_environment(scenario_id)
        
        # Generate move candidates
        candidates = self._generate_move_candidates(env)
        
        if not candidates:
            print("No valid move candidates found")
            return []
        
        # Use scenario exploration to evaluate move sequences
        best_sequence = []
        best_score = float('-inf')
        
        # Explore different combinations of moves
        for candidate in candidates[:min(10, len(candidates))]:  # Limit exploration for performance
            # Create branch scenario
            branch_id = self.scenario_manager.branch_scenario(
                scenario_id, 
                f"explore_{candidate.source_cell}"
            )
            
            # Simulate the move sequence
            sequence_score = self._evaluate_move_sequence([candidate], branch_id, n - 1)
            
            if sequence_score > best_score:
                best_score = sequence_score
                best_sequence = [candidate]
        
        print(f"Selected {len(best_sequence)} moves with score {best_score:.3f}")
        return best_sequence
    
    def evaluate_strategy_performance(self) -> Dict[str, Dict[str, float]]:
        """
        Evaluate performance of different strategies.
        
        Returns:
            Dictionary with strategy performance metrics
        """
        return copy.deepcopy(self.strategy_performance)
    
    def recommend_strategy(self, scenario_id: str) -> PlanningStrategy:
        """
        Recommend the best strategy based on current environment state.
        
        Args:
            scenario_id: Scenario to analyze
            
        Returns:
            Recommended planning strategy
        """
        env = self.scenario_manager.get_scenario_environment(scenario_id)
        
        # Analyze environment characteristics
        total_objects = sum(cell.num_objects for cell in env.cells_with_objects)
        num_cells_with_objects = len(env.cells_with_objects)
        avg_objects_per_cell = total_objects / max(1, num_cells_with_objects)
        
        # Calculate environment density
        grid_area = env.grid_size * env.grid_size
        density = num_cells_with_objects / grid_area
        
        # Calculate agent to objects ratio
        agent_count = len(env.agents)
        agent_object_ratio = agent_count / max(1, total_objects)
        
        print(f"Environment analysis: {total_objects} objects, {num_cells_with_objects} cells, density: {density:.3f}")
        
        # Strategy recommendation logic
        if density < 0.1:  # Sparse environment
            if agent_object_ratio > 0.1:
                return PlanningStrategy.COORDINATED_SWEEP
            else:
                return PlanningStrategy.GREEDY_NEAREST
        
        elif avg_objects_per_cell > 5:  # High object density per cell
            return PlanningStrategy.HIGHWAY_FORMATION
        
        elif total_objects < 20:  # Small problem
            return PlanningStrategy.GREEDY_EFFICIENT
        
        else:  # General case
            return PlanningStrategy.BALANCED_OPTIMIZATION
    
    # Strategy-specific planning methods
    
    def _plan_greedy_nearest(self, scenario_id: str, env: SimulationEnv, 
                           objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                           max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using greedy nearest strategy."""
        print("Executing greedy nearest strategy")
        
        move_sequence = []
        current_env = env
        
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            # Find nearest cell with objects for each agent
            candidates = self._generate_move_candidates(current_env)
            
            if not candidates:
                break
            
            # Select candidate with shortest distance
            best_candidate = min(candidates, key=lambda c: c.expected_distance)
            move_sequence.append(best_candidate)
            
            # Simulate move execution (simplified)
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    def _plan_greedy_efficient(self, scenario_id: str, env: SimulationEnv,
                             objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                             max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using greedy efficient strategy."""
        print("Executing greedy efficient strategy")
        
        move_sequence = []
        current_env = env
        
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            candidates = self._generate_move_candidates(current_env)
            
            if not candidates:
                break
            
            # Calculate efficiency score for each candidate
            for candidate in candidates:
                if candidate.expected_distance > 0:
                    candidate.raw_score = candidate.expected_objects / candidate.expected_distance
                else:
                    candidate.raw_score = candidate.expected_objects * 1000  # High score for zero distance
            
            # Select most efficient candidate
            best_candidate = max(candidates, key=lambda c: c.raw_score)
            move_sequence.append(best_candidate)
            
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    def _plan_highway_formation(self, scenario_id: str, env: SimulationEnv,
                              objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                              max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using highway formation strategy."""
        print("Executing highway formation strategy")
        
        move_sequence = []
        
        # Phase 1: Build highways by prioritizing high-potential cells
        highway_moves = self._generate_highway_building_moves(env)
        move_sequence.extend(highway_moves)
        
        # Phase 2: Utilize highways for efficient transport
        current_env = env
        for move in highway_moves:
            current_env = self._simulate_move_execution(current_env, move)
        
        # Continue with highway utilization
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            candidates = self._generate_move_candidates(current_env)
            
            # Prefer highway paths
            highway_candidates = [c for c in candidates if c.path_type == "highway"]
            
            if highway_candidates:
                best_candidate = max(highway_candidates, key=lambda c: c.expected_objects)
            elif candidates:
                best_candidate = max(candidates, key=lambda c: c.expected_objects)
            else:
                break
            
            move_sequence.append(best_candidate)
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    def _plan_spillage_minimization(self, scenario_id: str, env: SimulationEnv,
                                  objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                                  max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using spillage minimization strategy."""
        print("Executing spillage minimization strategy")
        
        move_sequence = []
        current_env = env
        
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            candidates = self._generate_move_candidates(current_env)
            
            if not candidates:
                break
            
            # Select candidate with minimum spillage
            best_candidate = min(candidates, key=lambda c: c.expected_spillage)
            move_sequence.append(best_candidate)
            
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    def _plan_balanced_optimization(self, scenario_id: str, env: SimulationEnv,
                                  objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                                  max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using balanced multi-objective optimization."""
        print("Executing balanced optimization strategy")
        
        move_sequence = []
        current_env = env
        
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            candidates = self._generate_move_candidates(current_env)
            
            if not candidates:
                break
            
            # Score candidates using multi-objective function
            self._score_candidates_multi_objective(candidates, objectives)
            
            # Select best candidate
            best_candidate = max(candidates, key=lambda c: c.normalized_score)
            move_sequence.append(best_candidate)
            
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    def _plan_adaptive_hybrid(self, scenario_id: str, env: SimulationEnv,
                            objectives: List[ObjectiveWeight], constraints: List[PlanningConstraint],
                            max_time: float, result: PlanningResult) -> PlanningResult:
        """Plan using adaptive hybrid strategy that switches based on state."""
        print("Executing adaptive hybrid strategy")
        
        move_sequence = []
        current_env = env
        move_count = 0
        
        while not self._is_transportation_complete(current_env) and time.time() - result.planning_time < max_time:
            # Adapt strategy based on current state
            remaining_objects = sum(cell.num_objects for cell in current_env.cells_with_objects)
            
            if move_count < 5:  # Early phase - build highways
                current_strategy = PlanningStrategy.HIGHWAY_FORMATION
            elif remaining_objects > 50:  # Many objects remaining - be efficient
                current_strategy = PlanningStrategy.GREEDY_EFFICIENT
            elif remaining_objects > 10:  # Moderate objects - balance objectives
                current_strategy = PlanningStrategy.BALANCED_OPTIMIZATION
            else:  # Few objects remaining - minimize spillage
                current_strategy = PlanningStrategy.SPILLAGE_MINIMIZATION
            
            # Generate and score candidates based on current strategy
            candidates = self._generate_move_candidates(current_env)
            
            if not candidates:
                break
            
            best_candidate = self._select_candidate_by_strategy(candidates, current_strategy, objectives)
            move_sequence.append(best_candidate)
            
            current_env = self._simulate_move_execution(current_env, best_candidate)
            result.scenarios_explored += 1
            move_count += 1
        
        result.move_sequence = move_sequence
        result.total_moves = len(move_sequence)
        
        return result
    
    # Helper methods
    
    def _generate_move_candidates(self, env: SimulationEnv) -> List[MoveCandidate]:
        """Generate all valid move candidates for current environment state."""
        candidates = []
        
        for agent_idx, agent in enumerate(env.agents):
            agent_x, agent_y = agent["position"]
            
            # Find nearby cells with objects
            for cell in env.cells_with_objects:
                if cell.num_objects == 0:
                    continue
                
                # Calculate path and expected outcomes
                # Simplified - in practice would use actual pathfinding
                distance = math.hypot(cell.x - agent_x, cell.y - agent_y)
                
                # Create candidate for target path
                if hasattr(cell, 'best_path_target') and cell.best_path_target:
                    target_path = [(c.x, c.y) for c in cell.best_path_target]
                    
                    candidate = MoveCandidate(
                        agent_id=agent_idx,
                        source_cell=(agent_x, agent_y),
                        target_path=target_path,
                        path_type="target",
                        expected_objects=min(cell.num_objects, agent.get("capacity", 8)),
                        expected_spillage=0.05 * min(cell.num_objects, agent.get("capacity", 8)),  # Simplified
                        expected_distance=distance,
                        expected_time=distance * 1.2  # Simplified time estimation
                    )
                    candidates.append(candidate)
                
                # Create candidate for highway path if available
                if hasattr(cell, 'best_path_highway') and cell.best_path_highway:
                    highway_path = [(c.x, c.y) for c in cell.best_path_highway]
                    
                    candidate = MoveCandidate(
                        agent_id=agent_idx,
                        source_cell=(agent_x, agent_y),
                        target_path=highway_path,
                        path_type="highway",
                        expected_objects=min(cell.num_objects, agent.get("capacity", 8)),
                        expected_spillage=0.02 * min(cell.num_objects, agent.get("capacity", 8)),  # Less spillage
                        expected_distance=distance * 0.8,  # Highway efficiency
                        expected_time=distance * 0.9  # Faster highway travel
                    )
                    candidates.append(candidate)
        
        return candidates
    
    def _generate_highway_building_moves(self, env: SimulationEnv) -> List[MoveCandidate]:
        """Generate moves that focus on building highway infrastructure."""
        highway_moves = []
        
        # Identify high-potential cells that should be prioritized for highway formation
        high_potential_cells = []
        
        for cell in env.cells_with_objects:
            # Use heat map value as indicator of highway potential
            potential = getattr(cell, 'heat_map', 0)
            if potential > env.highway_threshold * 0.5:  # Above average highway potential
                high_potential_cells.append((cell, potential))
        
        # Sort by potential (highest first)
        high_potential_cells.sort(key=lambda x: x[1], reverse=True)
        
        # Create moves for top potential cells
        for cell, potential in high_potential_cells[:5]:  # Limit to top 5
            if hasattr(cell, 'best_path_highway') and cell.best_path_highway:
                highway_path = [(c.x, c.y) for c in cell.best_path_highway]
                
                move = MoveCandidate(
                    agent_id=0,  # Use first agent
                    source_cell=(0, 0),  # Simplified
                    target_path=highway_path,
                    path_type="highway",
                    expected_objects=min(cell.num_objects, 8),
                    expected_spillage=0.01,  # Low spillage for highway building
                    expected_distance=len(highway_path),
                    expected_time=len(highway_path) * 0.8,
                    priority=1  # High priority for highway building
                )
                highway_moves.append(move)
        
        return highway_moves
    
    def _score_candidates_multi_objective(self, candidates: List[MoveCandidate], 
                                        objectives: List[ObjectiveWeight]):
        """Score candidates using multi-objective optimization."""
        if not candidates:
            return
        
        # Normalize each objective across all candidates
        objective_values = {obj.objective: [] for obj in objectives}
        
        for candidate in candidates:
            for obj in objectives:
                if obj.objective == ObjectiveType.MAXIMIZE_EFFICIENCY:
                    if candidate.expected_distance > 0:
                        value = candidate.expected_objects / candidate.expected_distance
                    else:
                        value = candidate.expected_objects * 1000
                elif obj.objective == ObjectiveType.MINIMIZE_SPILLAGE:
                    value = -candidate.expected_spillage  # Negative for minimization
                elif obj.objective == ObjectiveType.MINIMIZE_DISTANCE:
                    value = -candidate.expected_distance  # Negative for minimization
                elif obj.objective == ObjectiveType.MINIMIZE_TIME:
                    value = -candidate.expected_time  # Negative for minimization
                else:
                    value = candidate.expected_objects  # Default to object count
                
                objective_values[obj.objective].append(value)
        
        # Normalize objective values
        normalized_values = {}
        for obj_type, values in objective_values.items():
            if values:
                min_val, max_val = min(values), max(values)
                if max_val > min_val:
                    normalized_values[obj_type] = [(v - min_val) / (max_val - min_val) for v in values]
                else:
                    normalized_values[obj_type] = [1.0] * len(values)
            else:
                normalized_values[obj_type] = [0.0] * len(candidates)
        
        # Calculate weighted scores
        for i, candidate in enumerate(candidates):
            weighted_score = 0.0
            for obj in objectives:
                weighted_score += obj.weight * normalized_values[obj.objective][i]
            
            candidate.normalized_score = weighted_score
    
    def _select_candidate_by_strategy(self, candidates: List[MoveCandidate], 
                                    strategy: PlanningStrategy,
                                    objectives: List[ObjectiveWeight]) -> MoveCandidate:
        """Select best candidate based on specified strategy."""
        if not candidates:
            return None
        
        if strategy == PlanningStrategy.GREEDY_NEAREST:
            return min(candidates, key=lambda c: c.expected_distance)
        elif strategy == PlanningStrategy.GREEDY_EFFICIENT:
            return max(candidates, key=lambda c: c.expected_objects / max(1, c.expected_distance))
        elif strategy == PlanningStrategy.SPILLAGE_MINIMIZATION:
            return min(candidates, key=lambda c: c.expected_spillage)
        elif strategy == PlanningStrategy.HIGHWAY_FORMATION:
            highway_candidates = [c for c in candidates if c.path_type == "highway"]
            if highway_candidates:
                return max(highway_candidates, key=lambda c: c.expected_objects)
            return max(candidates, key=lambda c: c.expected_objects)
        else:  # Balanced optimization
            self._score_candidates_multi_objective(candidates, objectives)
            return max(candidates, key=lambda c: c.normalized_score)
    
    def _simulate_move_execution(self, env: SimulationEnv, move: MoveCandidate) -> SimulationEnv:
        """
        Simulate execution of a move and return updated environment.
        Simplified version - in practice would use full simulation.
        """
        # Create a copy of the environment
        new_env = env.copy_state()
        
        # Find source cell and reduce objects
        if move.target_path:
            target_x, target_y = move.target_path[0]
            target_cell = new_env.get_cell(target_x, target_y)
            if target_cell and target_cell.num_objects > 0:
                objects_taken = min(target_cell.num_objects, move.expected_objects)
                target_cell.num_objects -= objects_taken
                target_cell.current_objects = target_cell.num_objects
        
        return new_env
    
    def _evaluate_move_sequence(self, sequence: List[MoveCandidate], scenario_id: str, remaining_depth: int) -> float:
        """Evaluate a sequence of moves using scenario exploration."""
        if remaining_depth <= 0 or not sequence:
            return sum(move.expected_objects for move in sequence)
        
        # Simplified evaluation - in practice would do full lookahead
        total_score = 0.0
        for move in sequence:
            total_score += move.expected_objects - move.expected_spillage * 2  # Penalize spillage
        
        return total_score
    
    def _is_transportation_complete(self, env: SimulationEnv) -> bool:
        """Check if all objects have been transported to target zone."""
        return len(env.cells_with_objects) == 0 or all(cell.num_objects == 0 for cell in env.cells_with_objects)
    
    def _calculate_objective_scores(self, result: PlanningResult, objectives: List[ObjectiveWeight]) -> PlanningResult:
        """Calculate objective scores for the planning result."""
        result.objective_scores = {}
        
        for obj in objectives:
            if obj.objective == ObjectiveType.MINIMIZE_MOVES:
                result.objective_scores[obj.objective] = -result.total_moves  # Negative for minimization
            elif obj.objective == ObjectiveType.MINIMIZE_SPILLAGE:
                result.objective_scores[obj.objective] = -result.estimated_spillage
            elif obj.objective == ObjectiveType.MINIMIZE_TIME:
                result.objective_scores[obj.objective] = -result.estimated_time
            elif obj.objective == ObjectiveType.MINIMIZE_DISTANCE:
                result.objective_scores[obj.objective] = -result.estimated_distance
            elif obj.objective == ObjectiveType.MAXIMIZE_EFFICIENCY:
                if result.total_moves > 0:
                    result.objective_scores[obj.objective] = result.estimated_objects / result.total_moves
                else:
                    result.objective_scores[obj.objective] = 0
            else:
                result.objective_scores[obj.objective] = result.estimated_objects
        
        # Calculate overall weighted score
        result.overall_score = sum(obj.weight * result.objective_scores.get(obj.objective, 0) 
                                 for obj in objectives)
        
        return result
    
    def _update_strategy_performance(self, strategy: PlanningStrategy, result: PlanningResult):
        """Update performance tracking for a strategy."""
        if strategy.value not in self.strategy_performance:
            self.strategy_performance[strategy.value] = {
                'total_uses': 0,
                'avg_score': 0.0,
                'avg_moves': 0.0,
                'avg_efficiency': 0.0,
                'success_rate': 0.0
            }
        
        stats = self.strategy_performance[strategy.value]
        stats['total_uses'] += 1
        
        # Update running averages
        n = stats['total_uses']
        stats['avg_score'] = (stats['avg_score'] * (n-1) + result.overall_score) / n
        stats['avg_moves'] = (stats['avg_moves'] * (n-1) + result.total_moves) / n
        
        if result.total_moves > 0:
            efficiency = result.estimated_objects / result.total_moves
            stats['avg_efficiency'] = (stats['avg_efficiency'] * (n-1) + efficiency) / n
        
        # Success rate based on convergence
        if result.convergence_achieved:
            stats['success_rate'] = (stats['success_rate'] * (n-1) + 1.0) / n
        else:
            stats['success_rate'] = (stats['success_rate'] * (n-1) + 0.0) / n


if __name__ == "__main__":
    # Example usage and testing
    print("Testing StrategyPlanner...")
    
    # Create mock environment and scenario manager
    from core_env import SimulationEnv
    from strategic_scenario_manager import ScenarioManager
    
    env = SimulationEnv(grid_size=15, target_zone_radius=3, agent_positions=[(5, 5, 0)], num_random_objects=10)
    scenario_manager = ScenarioManager(env)
    
    # Create strategy planner
    planner = StrategyPlanner(scenario_manager)
    
    # Test strategy recommendation
    recommended_strategy = planner.recommend_strategy(scenario_manager.root_scenario_id)
    print(f"Recommended strategy: {recommended_strategy.value}")
    
    # Test planning
    result = planner.plan_complete_transportation(
        scenario_manager.root_scenario_id,
        strategy=PlanningStrategy.BALANCED_OPTIMIZATION,
        max_planning_time=10.0
    )
    
    print(f"Planning result: {result.total_moves} moves, score: {result.overall_score:.3f}")
    print(f"Estimated objects: {result.estimated_objects}, spillage: {result.estimated_spillage:.2f}")
    
    # Test optimization of next moves
    next_moves = planner.optimize_next_n_moves(scenario_manager.root_scenario_id, n=3)
    print(f"Optimized next {len(next_moves)} moves")
    
    print("StrategyPlanner testing completed successfully!")