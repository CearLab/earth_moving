"""
Strategic Orchestrator for Complete Earth Moving Operations

This module serves as the main coordinator for strategic planning and execution
of complete aggregate transportation. It integrates all components: scenario management,
strategy planning, real simulation interface, and move history tracking.
"""

import time
import uuid
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum
import json

from core_env import SimulationEnv
from strategic_scenario_manager import ScenarioManager, ScenarioStatus
from strategic_strategy_planner import StrategyPlanner, PlanningStrategy, PlanningResult, ObjectiveWeight, ObjectiveType
from interface_real_sim import RealSimulationInterface, ValidationReport, SimulationSyncMode
from strategic_move_history import MoveHistoryTracker, MoveStatus


class OperationMode(Enum):
    """Operation modes for the strategic orchestrator."""
    STANDALONE = "standalone"              # Pure 2D algorithm simulation
    INTEGRATED = "integrated"              # Connected to 3D simulation
    ANALYSIS_ONLY = "analysis_only"        # Analysis of existing data
    PLANNING_ONLY = "planning_only"        # Planning without execution


class OperationStatus(Enum):
    """Status of the strategic operation."""
    INITIALIZING = "initializing"
    READY = "ready"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    PAUSED = "paused"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class OperationConfig:
    """Configuration for strategic operations."""
    operation_mode: OperationMode = OperationMode.STANDALONE
    planning_strategy: PlanningStrategy = PlanningStrategy.BALANCED_OPTIMIZATION
    
    # Objectives and constraints
    objectives: List[ObjectiveWeight] = field(default_factory=list)
    max_moves_allowed: int = 1000
    max_operation_time: float = 3600.0  # 1 hour
    spillage_tolerance: float = 0.1
    
    # Integration settings
    sync_mode: SimulationSyncMode = SimulationSyncMode.FULL_SYNC
    sync_frequency: float = 30.0  # seconds
    validation_frequency: float = 60.0  # seconds
    
    # Planning parameters
    lookahead_depth: int = 3
    scenario_exploration_depth: int = 2
    adaptive_strategy_switching: bool = True
    
    # Execution settings
    auto_execute_moves: bool = False
    move_validation_required: bool = True
    rollback_on_failure: bool = True
    
    # Logging and analysis
    detailed_logging: bool = True
    performance_tracking: bool = True
    auto_save_history: bool = True
    
    def __post_init__(self):
        """Set default objectives if none provided."""
        if not self.objectives:
            self.objectives = [
                ObjectiveWeight(ObjectiveType.MINIMIZE_MOVES, 0.25),
                ObjectiveWeight(ObjectiveType.MAXIMIZE_EFFICIENCY, 0.25),
                ObjectiveWeight(ObjectiveType.MINIMIZE_SPILLAGE, 0.25),
                ObjectiveWeight(ObjectiveType.MINIMIZE_TIME, 0.25)
            ]


@dataclass
class OperationSummary:
    """Summary of a completed strategic operation."""
    operation_id: str
    operation_mode: OperationMode
    start_time: float
    end_time: float
    total_duration: float
    
    # Planning results
    strategy_used: PlanningStrategy
    total_moves_planned: int
    total_moves_executed: int
    
    # Outcomes
    initial_objects: int
    objects_transported: int
    total_spillage: float
    completion_percentage: float
    
    # Performance metrics
    average_move_efficiency: float
    total_distance_traveled: float
    average_spillage_per_move: float
    
    # Scenario exploration
    scenarios_created: int
    scenarios_explored: int
    
    # Success metrics
    operation_successful: bool
    failure_reasons: List[str] = field(default_factory=list)
    
    # Recommendations
    recommendations: List[str] = field(default_factory=list)


class StrategicOrchestrator:
    """
    Main coordinator for strategic earth moving operations.
    
    This class integrates all components to provide a complete solution for:
    - Strategic planning with multiple scenarios
    - Integration with real 3D simulation
    - Complete execution monitoring and control
    - Performance analysis and optimization
    - Adaptive strategy selection
    - Comprehensive logging and history tracking
    """
    
    def __init__(self, 
                 base_env: SimulationEnv, 
                 config: Optional[OperationConfig] = None,
                 storage_path: Optional[str] = None):
        """
        Initialize the strategic orchestrator.
        
        Args:
            base_env: Base environment for operations
            config: Operation configuration
            storage_path: Optional path for data storage
        """
        self.base_env = base_env
        self.config = config or OperationConfig()
        self.storage_path = storage_path
        
        # Initialize components
        self.scenario_manager = ScenarioManager(base_env, storage_path)
        self.strategy_planner = StrategyPlanner(self.scenario_manager)
        self.move_history = MoveHistoryTracker(storage_path)
        
        # Initialize real simulation interface if needed
        self.real_sim_interface = None
        if self.config.operation_mode == OperationMode.INTEGRATED:
            self.real_sim_interface = RealSimulationInterface(
                base_env.grid_size, 
                base_env.target_zone_radius
            )
        
        # Operation state
        self.operation_id = str(uuid.uuid4())
        self.status = OperationStatus.INITIALIZING
        self.current_scenario_id = None
        self.current_plan = None
        
        # Execution tracking
        self.operation_start_time = None
        self.operation_end_time = None
        self.moves_executed = 0
        self.moves_planned = 0
        self.last_sync_time = 0
        self.last_validation_time = 0
        
        # Performance metrics
        self.operation_metrics = {}
        self.adaptive_adjustments = 0
        
        print(f"StrategicOrchestrator initialized (ID: {self.operation_id[:8]})")
        print(f"Mode: {self.config.operation_mode.value}, Strategy: {self.config.planning_strategy.value}")
        
        self.status = OperationStatus.READY
    
    def start_complete_transportation_operation(self, 
                                              target_completion: float = 100.0,
                                              custom_objectives: Optional[List[ObjectiveWeight]] = None) -> OperationSummary:
        """
        Start complete transportation operation to move all objects to target zone.
        
        Args:
            target_completion: Target completion percentage (0-100)
            custom_objectives: Custom objectives for this operation
            
        Returns:
            OperationSummary: Complete operation results
        """
        print(f"\n>>> Starting complete transportation operation")
        print(f"Target completion: {target_completion}%, Mode: {self.config.operation_mode.value}")
        
        self.operation_start_time = time.time()
        self.status = OperationStatus.PLANNING
        
        objectives = custom_objectives or self.config.objectives
        
        try:
            # Phase 1: Initialize operation scenario
            print("\n>>> Phase 1: Initializing operation scenario...")
            operation_scenario_id = self._initialize_operation_scenario()
            
            # Phase 2: Strategic planning
            print("\n>>> Phase 2: Strategic planning...")
            planning_result = self._execute_strategic_planning(operation_scenario_id, objectives)
            
            if not planning_result or planning_result.total_moves == 0:
                print("FAILED Strategic planning failed - no valid moves found")
                return self._create_failure_summary("No valid moves found in strategic planning")
            
            self.current_plan = planning_result
            self.moves_planned = planning_result.total_moves
            
            print(f"SUCCESS Strategic planning completed: {planning_result.total_moves} moves planned")
            print(f"Expected outcomes: {planning_result.estimated_objects} objects, {planning_result.estimated_spillage:.2f} spillage")
            
            # Phase 3: Execution (if not planning-only mode)
            if self.config.operation_mode != OperationMode.PLANNING_ONLY:
                print("\n>>> Phase 3: Move execution...")
                execution_successful = self._execute_planned_moves(operation_scenario_id, planning_result)
                
                if not execution_successful:
                    print("FAILED Move execution encountered failures")
                    return self._create_failure_summary("Move execution failed")
            else:
                print("\n>>> Planning-only mode - skipping execution")
                execution_successful = True
            
            # Phase 4: Final analysis and summary
            print("\n>>> Phase 4: Final analysis...")
            operation_summary = self._create_operation_summary(
                operation_scenario_id, 
                planning_result, 
                execution_successful
            )
            
            self.status = OperationStatus.COMPLETED if execution_successful else OperationStatus.FAILED
            print(f"\n>>> Operation completed: {operation_summary.completion_percentage:.1f}% completion")
            
            return operation_summary
            
        except Exception as e:
            print(f"FAILED Operation failed with error: {e}")
            self.status = OperationStatus.FAILED
            return self._create_failure_summary(f"Operation error: {e}")
    
    def execute_single_planning_cycle(self, scenario_id: Optional[str] = None) -> PlanningResult:
        """
        Execute a single planning cycle for immediate decision making.
        
        Args:
            scenario_id: Scenario to plan for (uses active if None)
            
        Returns:
            PlanningResult: Planning results for immediate use
        """
        print(">>> Executing single planning cycle...")
        
        if scenario_id is None:
            scenario_id = self.scenario_manager.active_scenario_id or self.scenario_manager.root_scenario_id
        
        # Sync with real simulation if in integrated mode
        if self.config.operation_mode == OperationMode.INTEGRATED:
            self._sync_with_real_simulation(scenario_id)
        
        # Execute strategic planning
        planning_result = self.strategy_planner.plan_complete_transportation(
            scenario_id=scenario_id,
            strategy=self.config.planning_strategy,
            objectives=self.config.objectives,
            max_planning_time=min(30.0, self.config.max_operation_time),
            lookahead_depth=self.config.lookahead_depth
        )
        
        print(f"Planning cycle completed: {planning_result.total_moves} moves, score: {planning_result.overall_score:.3f}")
        
        return planning_result
    
    def execute_next_best_move(self, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Execute the next best move based on current strategic analysis.
        
        Args:
            scenario_id: Scenario to execute in
            
        Returns:
            Dictionary with execution results
        """
        print(">>> Executing next best move...")
        
        if scenario_id is None:
            scenario_id = self.scenario_manager.active_scenario_id or self.scenario_manager.root_scenario_id
        
        # Get optimized next moves
        next_moves = self.strategy_planner.optimize_next_n_moves(
            scenario_id=scenario_id,
            n=1,
            strategy=self.config.planning_strategy
        )
        
        if not next_moves:
            return {'success': False, 'error': 'No valid moves available'}
        
        best_move = next_moves[0]
        
        # Execute the move
        execution_result = self._execute_single_move(scenario_id, best_move)
        
        # Update adaptive strategy if enabled
        if self.config.adaptive_strategy_switching:
            self._update_adaptive_strategy(execution_result)
        
        return execution_result
    
    def analyze_current_situation(self, scenario_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Analyze current situation and provide strategic recommendations.
        
        Args:
            scenario_id: Scenario to analyze
            
        Returns:
            Dictionary with analysis results and recommendations
        """
        print(">>> Analyzing current situation...")
        
        if scenario_id is None:
            scenario_id = self.scenario_manager.active_scenario_id or self.scenario_manager.root_scenario_id
        
        env = self.scenario_manager.get_scenario_environment(scenario_id)
        
        # Environmental analysis
        total_objects = sum(cell.num_objects for cell in env.cells_with_objects)
        num_cells = len(env.cells_with_objects)
        avg_objects_per_cell = total_objects / max(1, num_cells)
        
        # Calculate completion status
        initial_objects = getattr(self, 'initial_objects', total_objects)
        completion_percentage = max(0, (initial_objects - total_objects) / max(1, initial_objects) * 100)
        
        # Strategy performance analysis
        strategy_performance = self.move_history.get_strategy_performance(self.config.planning_strategy.value)
        
        # Get recommended strategy
        recommended_strategy = self.strategy_planner.recommend_strategy(scenario_id)
        
        analysis = {
            'scenario_id': scenario_id,
            'timestamp': time.time(),
            
            # Current state
            'current_state': {
                'total_objects_remaining': total_objects,
                'cells_with_objects': num_cells,
                'avg_objects_per_cell': avg_objects_per_cell,
                'completion_percentage': completion_percentage
            },
            
            # Performance metrics
            'performance': strategy_performance,
            
            # Strategic recommendations
            'recommendations': {
                'recommended_strategy': recommended_strategy.value,
                'strategy_change_needed': recommended_strategy != self.config.planning_strategy,
                'estimated_moves_remaining': max(1, total_objects // max(1, int(avg_objects_per_cell))),
                'priority_actions': []
            },
            
            # Scenario analysis
            'scenario_analysis': {
                'scenarios_available': len(self.scenario_manager.scenarios),
                'current_scenario_moves': len(self.scenario_manager.scenarios[scenario_id].moves) if scenario_id in self.scenario_manager.scenarios else 0,
                'exploration_opportunities': self._identify_exploration_opportunities(scenario_id)
            }
        }
        
        # Add priority action recommendations
        if completion_percentage < 10:
            analysis['recommendations']['priority_actions'].append("Focus on initial object collection")
        elif completion_percentage < 50:
            analysis['recommendations']['priority_actions'].append("Build highway infrastructure")
        elif completion_percentage < 90:
            analysis['recommendations']['priority_actions'].append("Utilize highways for efficient transport")
        else:
            analysis['recommendations']['priority_actions'].append("Minimize spillage for final objects")
        
        if strategy_performance.get('avg_spillage_ratio', 0) > 0.15:
            analysis['recommendations']['priority_actions'].append("Reduce spillage - consider path optimization")
        
        return analysis
    
    def generate_operation_report(self) -> Dict[str, Any]:
        """Generate comprehensive operation report."""
        print(">>> Generating comprehensive operation report...")
        
        # Collect performance data
        all_strategies = list(self.move_history.strategy_history.keys())
        strategy_comparison = self.move_history.analyze_strategy_comparison(all_strategies) if all_strategies else {}
        
        # Scenario tree analysis
        scenario_tree = self.scenario_manager.get_scenario_tree()
        scenario_comparison = self.scenario_manager.compare_scenarios(
            list(self.scenario_manager.scenarios.keys())
        )
        
        # Efficiency analysis
        efficiency_analysis = self.move_history.analyze_efficiency_trends()
        
        report = {
            'report_id': str(uuid.uuid4()),
            'generated_at': time.time(),
            'operation_id': self.operation_id,
            
            # Operation summary
            'operation_summary': {
                'mode': self.config.operation_mode.value,
                'status': self.status.value,
                'total_duration': time.time() - (self.operation_start_time or time.time()),
                'moves_planned': self.moves_planned,
                'moves_executed': self.moves_executed
            },
            
            # Performance analysis
            'performance_analysis': {
                'strategy_comparison': strategy_comparison,
                'efficiency_trends': {
                    'total_moves_analyzed': efficiency_analysis.total_moves_analyzed,
                    'metrics': efficiency_analysis.metrics,
                    'recommendations': efficiency_analysis.recommendations
                }
            },
            
            # Scenario analysis
            'scenario_analysis': {
                'total_scenarios': len(self.scenario_manager.scenarios),
                'scenario_tree': scenario_tree,
                'best_scenarios': scenario_comparison.get('summary', {})
            },
            
            # Configuration analysis
            'configuration_effectiveness': self._analyze_configuration_effectiveness(),
            
            # Recommendations
            'strategic_recommendations': self._generate_strategic_recommendations()
        }
        
        return report
    
    # Private methods for operation phases
    
    def _initialize_operation_scenario(self) -> str:
        """Initialize main operation scenario."""
        # Sync with real simulation if in integrated mode
        if self.config.operation_mode == OperationMode.INTEGRATED and self.real_sim_interface:
            print(">>> Syncing with real simulation...")
            # In practice, would receive real simulation data
            sample_data = self.real_sim_interface.create_sample_real_sim_data(self.base_env.grid_size)
            updated_env = self.real_sim_interface.import_from_real_simulation(sample_data)
            
            # Update base environment
            self.base_env = updated_env
            self.scenario_manager.base_env = updated_env
        
        # Create main operation scenario
        operation_scenario_id = self.scenario_manager.create_scenario(
            name=f"Operation_{self.operation_id[:8]}",
            description=f"Main operation scenario for complete transportation",
            tags=['operation', 'main', self.config.planning_strategy.value]
        )
        
        self.scenario_manager.set_active_scenario(operation_scenario_id)
        self.current_scenario_id = operation_scenario_id
        
        # Record initial state metrics
        env = self.scenario_manager.get_scenario_environment(operation_scenario_id)
        self.initial_objects = sum(cell.num_objects for cell in env.cells_with_objects)
        
        return operation_scenario_id
    
    def _execute_strategic_planning(self, scenario_id: str, objectives: List[ObjectiveWeight]) -> Optional[PlanningResult]:
        """Execute strategic planning phase."""
        # Determine optimal strategy if adaptive is enabled
        if self.config.adaptive_strategy_switching:
            recommended_strategy = self.strategy_planner.recommend_strategy(scenario_id)
            print(f">>> Adaptive strategy recommendation: {recommended_strategy.value}")
            
            if recommended_strategy != self.config.planning_strategy:
                print(f">>> Switching strategy from {self.config.planning_strategy.value} to {recommended_strategy.value}")
                self.config.planning_strategy = recommended_strategy
                self.adaptive_adjustments += 1
        
        # Execute planning
        planning_result = self.strategy_planner.plan_complete_transportation(
            scenario_id=scenario_id,
            strategy=self.config.planning_strategy,
            objectives=objectives,
            max_planning_time=min(60.0, self.config.max_operation_time * 0.1),
            lookahead_depth=self.config.lookahead_depth
        )
        
        # Validate planning result
        if planning_result.total_moves > self.config.max_moves_allowed:
            print(f"⚠️  Planning exceeded move limit: {planning_result.total_moves} > {self.config.max_moves_allowed}")
            # Could implement plan optimization here
        
        return planning_result
    
    def _execute_planned_moves(self, scenario_id: str, planning_result: PlanningResult) -> bool:
        """Execute all planned moves."""
        self.status = OperationStatus.EXECUTING
        
        successful_moves = 0
        total_moves = len(planning_result.move_sequence)
        
        for i, move in enumerate(planning_result.move_sequence):
            print(f">>> Executing move {i+1}/{total_moves}: Agent {move.agent_id} -> {move.target_path[-1] if move.target_path else 'unknown'}")
            
            # Check time limit
            if time.time() - self.operation_start_time > self.config.max_operation_time:
                print(">>> Operation time limit reached")
                break
            
            # Execute single move
            result = self._execute_single_move(scenario_id, move)
            
            if result['success']:
                successful_moves += 1
                self.moves_executed += 1
            else:
                print(f"FAILED Move {i+1} failed: {result.get('error', 'Unknown error')}")
                
                if not self.config.rollback_on_failure:
                    continue
                else:
                    print(">>> Rollback enabled - stopping execution")
                    break
            
            # Periodic sync with real simulation
            if (self.config.operation_mode == OperationMode.INTEGRATED and 
                time.time() - self.last_sync_time > self.config.sync_frequency):
                self._sync_with_real_simulation(scenario_id)
        
        success_rate = successful_moves / max(1, total_moves)
        print(f">>> Execution completed: {successful_moves}/{total_moves} moves successful ({success_rate:.1%})")
        
        return success_rate >= 0.8  # Consider 80%+ success rate as successful
    
    def _execute_single_move(self, scenario_id: str, move) -> Dict[str, Any]:
        """Execute a single move and record results."""
        move_id = str(uuid.uuid4())
        
        # Get pre-move environment
        pre_move_env = self.scenario_manager.get_scenario_environment(scenario_id)
        
        # Record move plan
        self.move_history.record_move_plan(
            move_id=move_id,
            scenario_id=scenario_id,
            strategy=self.config.planning_strategy.value,
            agent_id=move.agent_id,
            source_cell=move.source_cell,
            target_cell=move.target_path[-1] if move.target_path else (0, 0),
            path_coords=move.target_path,
            path_type=move.path_type,
            expected_outcomes={
                'objects': move.expected_objects,
                'spillage': move.expected_spillage,
                'distance': move.expected_distance,
                'time': move.expected_time
            },
            pre_move_env=pre_move_env
        )
        
        # Start execution
        self.move_history.record_move_execution_start(move_id)
        
        try:
            # Simulate move execution (simplified)
            # In practice, this would interface with real simulation or detailed 2D execution
            time.sleep(0.1)  # Simulate execution time
            
            # Create post-move environment (simplified simulation)
            post_move_env = self._simulate_move_execution(pre_move_env, move)
            
            # Record completion
            actual_outcomes = {
                'objects': max(0, move.expected_objects - 1),  # Simplified: slightly less than expected
                'spillage': move.expected_spillage * 1.1,     # Simplified: slightly more spillage
                'distance': move.expected_distance
            }
            
            self.move_history.record_move_execution_complete(
                move_id=move_id,
                actual_outcomes=actual_outcomes,
                post_move_env=post_move_env
            )
            
            # Update scenario with results
            self.scenario_manager.record_move(scenario_id, {
                'agent_id': move.agent_id,
                'source_cell': move.source_cell,
                'target_cell': move.target_path[-1] if move.target_path else (0, 0),
                'path_type': move.path_type,
                'objects_collected': actual_outcomes['objects'],
                'spillage_amount': actual_outcomes['spillage'],
                'distance_traveled': actual_outcomes['distance'],
                'execution_time': 0.1
            }, post_move_env)
            
            return {
                'success': True,
                'move_id': move_id,
                'objects_collected': actual_outcomes['objects'],
                'spillage': actual_outcomes['spillage']
            }
            
        except Exception as e:
            # Record failure
            self.move_history.record_move_failure(move_id, str(e))
            
            return {
                'success': False,
                'move_id': move_id,
                'error': str(e)
            }
    
    def _simulate_move_execution(self, env: SimulationEnv, move) -> SimulationEnv:
        """Simplified move execution simulation."""
        # Create copy of environment
        new_env = env.copy_state()
        
        # Simulate object collection (simplified)
        if move.target_path:
            target_x, target_y = move.target_path[0]
            target_cell = new_env.get_cell(target_x, target_y)
            if target_cell and target_cell.num_objects > 0:
                objects_taken = min(target_cell.num_objects, move.expected_objects)
                target_cell.num_objects -= objects_taken
                target_cell.current_objects = target_cell.num_objects
        
        # Update environment tracking
        new_env._update_cells_with_objects_tracking()
        
        return new_env
    
    def _sync_with_real_simulation(self, scenario_id: str) -> bool:
        """Sync with real simulation if in integrated mode."""
        if not self.real_sim_interface:
            return True
        
        try:
            # Get current 2D state
            env_2d = self.scenario_manager.get_scenario_environment(scenario_id)
            
            # Create sample real simulation data (in practice, would receive from real sim)
            real_sim_data = self.real_sim_interface.create_sample_real_sim_data(env_2d.grid_size)
            
            # Validate consistency
            validation_report = self.real_sim_interface.validate_state_consistency(env_2d, real_sim_data)
            
            if not validation_report.is_consistent:
                print(f"⚠️  State inconsistency detected: {len(validation_report.discrepancies)} discrepancies")
                
                # Synchronize states
                sync_success = self.real_sim_interface.synchronize_states(
                    env_2d, real_sim_data, validation_report, self.config.sync_mode
                )
                
                if not sync_success:
                    print("FAILED State synchronization failed")
                    return False
            
            self.last_sync_time = time.time()
            return True
            
        except Exception as e:
            print(f"FAILED Real simulation sync failed: {e}")
            return False
    
    def _update_adaptive_strategy(self, execution_result: Dict[str, Any]):
        """Update strategy adaptively based on execution results."""
        if not execution_result.get('success', False):
            return
        
        # Simple adaptive logic - in practice would be more sophisticated
        spillage = execution_result.get('spillage', 0)
        
        if spillage > self.config.spillage_tolerance:
            # High spillage - switch to spillage minimization
            if self.config.planning_strategy != PlanningStrategy.SPILLAGE_MINIMIZATION:
                print(">>> Adaptive switch: High spillage detected, switching to spillage minimization")
                self.config.planning_strategy = PlanningStrategy.SPILLAGE_MINIMIZATION
                self.adaptive_adjustments += 1
    
    def _identify_exploration_opportunities(self, scenario_id: str) -> List[str]:
        """Identify opportunities for scenario exploration."""
        opportunities = []
        
        scenario = self.scenario_manager.scenarios.get(scenario_id)
        if not scenario:
            return opportunities
        
        # Check for alternative strategies
        if len(scenario.children_ids) == 0:
            opportunities.append("Create strategy comparison branches")
        
        # Check for rollback opportunities
        if len(scenario.moves) > 3:
            opportunities.append("Explore rollback and alternative paths")
        
        # Check for parallel scenario exploration
        if len(self.scenario_manager.scenarios) < 5:
            opportunities.append("Create parallel scenario exploration")
        
        return opportunities
    
    def _analyze_configuration_effectiveness(self) -> Dict[str, Any]:
        """Analyze effectiveness of current configuration."""
        return {
            'strategy_effectiveness': 'high' if self.adaptive_adjustments < 3 else 'medium',
            'adaptive_adjustments_made': self.adaptive_adjustments,
            'objective_balance': 'balanced',  # Simplified analysis
            'configuration_recommendations': [
                'Consider increasing lookahead depth for complex scenarios',
                'Evaluate spillage tolerance based on actual results'
            ]
        }
    
    def _generate_strategic_recommendations(self) -> List[str]:
        """Generate strategic recommendations based on operation analysis."""
        recommendations = []
        
        if self.moves_executed > 0:
            success_rate = self.moves_executed / max(1, self.moves_planned)
            if success_rate < 0.8:
                recommendations.append("Improve move validation and planning accuracy")
        
        if self.adaptive_adjustments > 2:
            recommendations.append("Consider more stable initial strategy selection")
        
        recommendations.extend([
            "Implement more sophisticated spillage prediction models",
            "Enhance highway formation strategy for better efficiency",
            "Consider multi-agent coordination improvements"
        ])
        
        return recommendations
    
    def _create_operation_summary(self, scenario_id: str, planning_result: PlanningResult, successful: bool) -> OperationSummary:
        """Create comprehensive operation summary."""
        end_time = time.time()
        
        # Get final environment state
        final_env = self.scenario_manager.get_scenario_environment(scenario_id)
        final_objects = sum(cell.num_objects for cell in final_env.cells_with_objects)
        
        objects_transported = max(0, self.initial_objects - final_objects)
        completion_percentage = (objects_transported / max(1, self.initial_objects)) * 100
        
        summary = OperationSummary(
            operation_id=self.operation_id,
            operation_mode=self.config.operation_mode,
            start_time=self.operation_start_time,
            end_time=end_time,
            total_duration=end_time - self.operation_start_time,
            
            strategy_used=self.config.planning_strategy,
            total_moves_planned=planning_result.total_moves,
            total_moves_executed=self.moves_executed,
            
            initial_objects=self.initial_objects,
            objects_transported=objects_transported,
            total_spillage=planning_result.estimated_spillage,
            completion_percentage=completion_percentage,
            
            average_move_efficiency=objects_transported / max(1, self.moves_executed),
            total_distance_traveled=planning_result.estimated_distance,
            average_spillage_per_move=planning_result.estimated_spillage / max(1, self.moves_executed),
            
            scenarios_created=len(self.scenario_manager.scenarios),
            scenarios_explored=planning_result.scenarios_explored,
            
            operation_successful=successful and completion_percentage >= 90.0
        )
        
        # Add recommendations
        if completion_percentage < 90.0:
            summary.recommendations.append("Consider alternative strategies for remaining objects")
        if self.adaptive_adjustments > 0:
            summary.recommendations.append(f"Strategy was adapted {self.adaptive_adjustments} times during operation")
        
        return summary
    
    def _create_failure_summary(self, reason: str) -> OperationSummary:
        """Create summary for failed operation."""
        end_time = time.time()
        
        return OperationSummary(
            operation_id=self.operation_id,
            operation_mode=self.config.operation_mode,
            start_time=self.operation_start_time or end_time,
            end_time=end_time,
            total_duration=end_time - (self.operation_start_time or end_time),
            
            strategy_used=self.config.planning_strategy,
            total_moves_planned=self.moves_planned,
            total_moves_executed=self.moves_executed,
            
            initial_objects=getattr(self, 'initial_objects', 0),
            objects_transported=0,
            total_spillage=0.0,
            completion_percentage=0.0,
            
            average_move_efficiency=0.0,
            total_distance_traveled=0.0,
            average_spillage_per_move=0.0,
            
            scenarios_created=len(self.scenario_manager.scenarios),
            scenarios_explored=0,
            
            operation_successful=False,
            failure_reasons=[reason]
        )


if __name__ == "__main__":
    # Example usage and comprehensive testing
    print(">>> Testing StrategicOrchestrator...")
    
    # Create base environment
    from core_env import SimulationEnv
    
    base_env = SimulationEnv(
        grid_size=20,
        target_zone_radius=3,
        agent_positions=[(5, 5, 0), (15, 15, 45)],
        num_random_objects=25,
        seed=42
    )
    
    # Create configuration
    config = OperationConfig(
        operation_mode=OperationMode.STANDALONE,
        planning_strategy=PlanningStrategy.BALANCED_OPTIMIZATION,
        max_moves_allowed=50,
        max_operation_time=120.0,
        adaptive_strategy_switching=True,
        auto_execute_moves=True
    )
    
    # Create orchestrator
    orchestrator = StrategicOrchestrator(base_env, config)
    
    # Test single planning cycle
    print("\n>>> Testing single planning cycle...")
    planning_result = orchestrator.execute_single_planning_cycle()
    print(f"Planning result: {planning_result.total_moves} moves, score: {planning_result.overall_score:.3f}")
    
    # Test situation analysis
    print("\n>>> Testing situation analysis...")
    analysis = orchestrator.analyze_current_situation()
    print(f"Current completion: {analysis['current_state']['completion_percentage']:.1f}%")
    print(f"Recommended strategy: {analysis['recommendations']['recommended_strategy']}")
    
    # Test next move execution
    print("\n>>> Testing next move execution...")
    move_result = orchestrator.execute_next_best_move()
    print(f"Move execution: {'SUCCESS' if move_result['success'] else 'FAILED'}")
    
    # Test complete operation (limited for testing)
    print("\n>>> Testing complete transportation operation...")
    config.max_moves_allowed = 10  # Limit for testing
    config.max_operation_time = 30.0
    
    operation_summary = orchestrator.start_complete_transportation_operation(
        target_completion=50.0  # Partial completion for testing
    )
    
    print(f"\n>>> Operation Summary:")
    print(f"Status: {'SUCCESS' if operation_summary.operation_successful else 'FAILED'}")
    print(f"Completion: {operation_summary.completion_percentage:.1f}%")
    print(f"Moves: {operation_summary.total_moves_executed}/{operation_summary.total_moves_planned}")
    print(f"Objects transported: {operation_summary.objects_transported}")
    print(f"Duration: {operation_summary.total_duration:.1f}s")
    
    # Generate comprehensive report
    print("\n>>> Generating operation report...")
    report = orchestrator.generate_operation_report()
    print(f"Report generated: {len(report['strategic_recommendations'])} recommendations")
    
    print("\n>>> StrategicOrchestrator testing completed successfully!")
    print(f"Operation ID: {orchestrator.operation_id}")
    print(f"Final Status: {orchestrator.status.value}")