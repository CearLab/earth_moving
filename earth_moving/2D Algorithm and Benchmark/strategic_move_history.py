"""
Move History Tracker for Earth Moving Strategic Planning

This module provides comprehensive tracking and analysis of move executions,
state transitions, and performance metrics across multiple scenarios and strategies.
It enables detailed post-analysis and learning from executed strategies.
"""

import json
import time
import copy
import hashlib
from typing import Dict, List, Optional, Any, Tuple
from dataclasses import dataclass, asdict, field
from enum import Enum
from pathlib import Path
import statistics

from core_env import SimulationEnv


class MoveStatus(Enum):
    """Status of a move execution."""
    PLANNED = "planned"           # Move is planned but not executed
    EXECUTING = "executing"       # Move is currently being executed
    COMPLETED = "completed"       # Move completed successfully
    FAILED = "failed"            # Move failed during execution
    CANCELLED = "cancelled"       # Move was cancelled before execution
    PARTIAL = "partial"          # Move partially completed


class AnalysisType(Enum):
    """Types of analysis that can be performed on move history."""
    EFFICIENCY_TRENDS = "efficiency_trends"
    SPILLAGE_PATTERNS = "spillage_patterns"
    STRATEGY_PERFORMANCE = "strategy_performance"
    AGENT_UTILIZATION = "agent_utilization"
    PATH_OPTIMIZATION = "path_optimization"
    TIME_SERIES = "time_series"


@dataclass
class StateSnapshot:
    """Snapshot of environment state at a specific point in time."""
    timestamp: float
    move_id: Optional[str]
    total_objects: int
    objects_in_target: int
    objects_transported: int
    agent_positions: List[Tuple[float, float]]
    heat_map_max: float
    heat_map_avg: float
    highway_threshold: float
    cells_with_objects_count: int
    state_hash: str
    
    # Derived metrics
    completion_percentage: float = field(default=0.0)
    transportation_efficiency: float = field(default=0.0)
    
    def __post_init__(self):
        """Calculate derived metrics."""
        if self.total_objects > 0:
            self.completion_percentage = (self.objects_transported / self.total_objects) * 100
        else:
            self.completion_percentage = 100.0
        
        if self.cells_with_objects_count > 0:
            self.transportation_efficiency = self.objects_transported / self.cells_with_objects_count
        else:
            self.transportation_efficiency = 0.0


@dataclass
class DetailedMoveRecord:
    """Comprehensive record of a single move execution."""
    # Basic move information (required fields first)
    move_id: str
    scenario_id: str
    strategy_used: str
    agent_id: int
    move_index: int  # Position in sequence
    planned_timestamp: float
    source_cell: Tuple[int, int]
    target_cell: Tuple[int, int]
    path_coordinates: List[Tuple[int, int]]
    path_type: str  # "target" or "highway"
    
    # Optional timing fields
    execution_start: Optional[float] = None
    execution_end: Optional[float] = None
    total_duration: float = field(default=0.0)
    path_length: int = field(default=0)
    
    # Expected vs actual outcomes
    expected_objects: int = 0
    actual_objects: int = 0
    expected_spillage: float = 0.0
    actual_spillage: float = 0.0
    expected_distance: float = 0.0
    actual_distance: float = 0.0
    expected_time: float = 0.0
    
    # Execution details
    status: MoveStatus = MoveStatus.PLANNED
    execution_phases: List[str] = field(default_factory=list)
    collision_events: List[str] = field(default_factory=list)
    error_messages: List[str] = field(default_factory=list)
    
    # State context
    pre_move_state: Optional[StateSnapshot] = None
    post_move_state: Optional[StateSnapshot] = None
    
    # Performance metrics
    efficiency_score: float = field(default=0.0)
    spillage_ratio: float = field(default=0.0)
    prediction_accuracy: float = field(default=0.0)
    
    # Additional metadata
    tags: List[str] = field(default_factory=list)
    notes: str = ""
    
    def __post_init__(self):
        """Calculate derived metrics."""
        self.path_length = len(self.path_coordinates)
        
        if self.execution_start and self.execution_end:
            self.total_duration = self.execution_end - self.execution_start
        
        # Calculate efficiency score
        if self.total_duration > 0 and self.actual_objects > 0:
            self.efficiency_score = self.actual_objects / self.total_duration
        
        # Calculate spillage ratio
        if self.actual_objects + self.actual_spillage > 0:
            self.spillage_ratio = self.actual_spillage / (self.actual_objects + self.actual_spillage)
        
        # Calculate prediction accuracy
        if self.expected_objects > 0:
            objects_accuracy = 1.0 - abs(self.expected_objects - self.actual_objects) / self.expected_objects
            spillage_accuracy = 1.0 - abs(self.expected_spillage - self.actual_spillage) / max(0.1, self.expected_spillage)
            self.prediction_accuracy = (objects_accuracy + spillage_accuracy) / 2.0
        else:
            self.prediction_accuracy = 0.0


@dataclass
class PerformanceAnalysis:
    """Analysis results for move history."""
    analysis_type: AnalysisType
    analysis_timestamp: float
    total_moves_analyzed: int
    time_period: Tuple[float, float]  # (start, end) timestamps
    
    # Metrics
    metrics: Dict[str, float] = field(default_factory=dict)
    trends: Dict[str, List[float]] = field(default_factory=dict)
    correlations: Dict[str, float] = field(default_factory=dict)
    
    # Summary statistics
    summary: Dict[str, Any] = field(default_factory=dict)
    
    # Recommendations
    recommendations: List[str] = field(default_factory=list)


class MoveHistoryTracker:
    """
    Comprehensive tracking and analysis system for move executions.
    
    Features:
    - Detailed move recording with state snapshots
    - Performance analytics and trend analysis
    - Strategy effectiveness evaluation
    - Prediction accuracy tracking
    - Historical data persistence
    - Interactive analysis tools
    """
    
    def __init__(self, storage_path: Optional[str] = None):
        """
        Initialize the move history tracker.
        
        Args:
            storage_path: Optional path for persistent storage
        """
        self.move_history: Dict[str, DetailedMoveRecord] = {}
        self.scenario_history: Dict[str, List[str]] = {}  # scenario_id -> move_ids
        self.strategy_history: Dict[str, List[str]] = {}  # strategy -> move_ids
        self.agent_history: Dict[int, List[str]] = {}    # agent_id -> move_ids
        
        self.state_snapshots: List[StateSnapshot] = []
        self.analysis_cache: Dict[str, PerformanceAnalysis] = {}
        
        self.storage_path = Path(storage_path) if storage_path else None
        self.auto_save = True
        
        # Performance tracking
        self.total_moves_recorded = 0
        self.total_scenarios_tracked = 0
        self.tracking_start_time = time.time()
        
        print("MoveHistoryTracker initialized")
    
    def record_move_plan(self, 
                        move_id: str,
                        scenario_id: str,
                        strategy: str,
                        agent_id: int,
                        source_cell: Tuple[int, int],
                        target_cell: Tuple[int, int],
                        path_coords: List[Tuple[int, int]],
                        path_type: str,
                        expected_outcomes: Dict[str, Any],
                        pre_move_env: SimulationEnv) -> bool:
        """
        Record a planned move before execution.
        
        Args:
            move_id: Unique move identifier
            scenario_id: Scenario this move belongs to
            strategy: Strategy being used
            agent_id: ID of agent performing move
            source_cell: Starting cell coordinates
            target_cell: Target cell coordinates
            path_coords: Complete path coordinates
            path_type: "target" or "highway"
            expected_outcomes: Dictionary of expected results
            pre_move_env: Environment state before move
            
        Returns:
            bool: True if successfully recorded
        """
        try:
            # Create state snapshot
            pre_state = self._create_state_snapshot(pre_move_env, move_id)
            
            # Create move record
            move_record = DetailedMoveRecord(
                move_id=move_id,
                scenario_id=scenario_id,
                strategy_used=strategy,
                agent_id=agent_id,
                move_index=len(self.scenario_history.get(scenario_id, [])),
                planned_timestamp=time.time(),
                source_cell=source_cell,
                target_cell=target_cell,
                path_coordinates=path_coords,
                path_type=path_type,
                expected_objects=expected_outcomes.get('objects', 0),
                expected_spillage=expected_outcomes.get('spillage', 0.0),
                expected_distance=expected_outcomes.get('distance', 0.0),
                expected_time=expected_outcomes.get('time', 0.0),
                pre_move_state=pre_state,
                status=MoveStatus.PLANNED
            )
            
            # Store move record
            self.move_history[move_id] = move_record
            
            # Update tracking structures
            if scenario_id not in self.scenario_history:
                self.scenario_history[scenario_id] = []
                self.total_scenarios_tracked += 1
            self.scenario_history[scenario_id].append(move_id)
            
            if strategy not in self.strategy_history:
                self.strategy_history[strategy] = []
            self.strategy_history[strategy].append(move_id)
            
            if agent_id not in self.agent_history:
                self.agent_history[agent_id] = []
            self.agent_history[agent_id].append(move_id)
            
            # Store state snapshot
            self.state_snapshots.append(pre_state)
            
            self.total_moves_recorded += 1
            
            print(f"Recorded move plan: {move_id} for agent {agent_id} in scenario {scenario_id}")
            
            if self.auto_save and self.storage_path:
                self._auto_save()
            
            return True
            
        except Exception as e:
            print(f"Error recording move plan: {e}")
            return False
    
    def record_move_execution_start(self, move_id: str) -> bool:
        """Record the start of move execution."""
        if move_id not in self.move_history:
            print(f"Error: Move {move_id} not found in history")
            return False
        
        move_record = self.move_history[move_id]
        move_record.execution_start = time.time()
        move_record.status = MoveStatus.EXECUTING
        move_record.execution_phases.append("execution_started")
        
        print(f"Move {move_id} execution started")
        return True
    
    def record_move_execution_complete(self,
                                     move_id: str,
                                     actual_outcomes: Dict[str, Any],
                                     post_move_env: SimulationEnv,
                                     execution_details: Optional[Dict[str, Any]] = None) -> bool:
        """
        Record completion of move execution.
        
        Args:
            move_id: Move identifier
            actual_outcomes: Dictionary of actual results
            post_move_env: Environment state after move
            execution_details: Optional additional execution details
            
        Returns:
            bool: True if successfully recorded
        """
        if move_id not in self.move_history:
            print(f"Error: Move {move_id} not found in history")
            return False
        
        try:
            move_record = self.move_history[move_id]
            
            # Record completion timing
            move_record.execution_end = time.time()
            move_record.status = MoveStatus.COMPLETED
            move_record.execution_phases.append("execution_completed")
            
            # Record actual outcomes
            move_record.actual_objects = actual_outcomes.get('objects', 0)
            move_record.actual_spillage = actual_outcomes.get('spillage', 0.0)
            move_record.actual_distance = actual_outcomes.get('distance', 0.0)
            
            # Record post-move state
            post_state = self._create_state_snapshot(post_move_env, move_id)
            move_record.post_move_state = post_state
            self.state_snapshots.append(post_state)
            
            # Record execution details if provided
            if execution_details:
                if 'collision_events' in execution_details:
                    move_record.collision_events.extend(execution_details['collision_events'])
                if 'error_messages' in execution_details:
                    move_record.error_messages.extend(execution_details['error_messages'])
                if 'notes' in execution_details:
                    move_record.notes = execution_details['notes']
            
            # Recalculate derived metrics (handled in __post_init__)
            move_record.__post_init__()
            
            print(f"Move {move_id} execution completed: {move_record.actual_objects} objects, "
                  f"{move_record.actual_spillage:.2f} spillage, efficiency: {move_record.efficiency_score:.2f}")
            
            if self.auto_save and self.storage_path:
                self._auto_save()
            
            return True
            
        except Exception as e:
            print(f"Error recording move completion: {e}")
            return False
    
    def record_move_failure(self, move_id: str, error_message: str, failure_details: Optional[Dict[str, Any]] = None) -> bool:
        """Record a failed move execution."""
        if move_id not in self.move_history:
            print(f"Error: Move {move_id} not found in history")
            return False
        
        move_record = self.move_history[move_id]
        move_record.execution_end = time.time()
        move_record.status = MoveStatus.FAILED
        move_record.execution_phases.append("execution_failed")
        move_record.error_messages.append(error_message)
        
        if failure_details:
            if 'partial_objects' in failure_details:
                move_record.actual_objects = failure_details['partial_objects']
            if 'additional_errors' in failure_details:
                move_record.error_messages.extend(failure_details['additional_errors'])
        
        print(f"Move {move_id} failed: {error_message}")
        return True
    
    def get_scenario_history(self, scenario_id: str) -> List[DetailedMoveRecord]:
        """Get complete move history for a scenario."""
        if scenario_id not in self.scenario_history:
            return []
        
        return [self.move_history[move_id] for move_id in self.scenario_history[scenario_id] 
                if move_id in self.move_history]
    
    def get_strategy_performance(self, strategy: str) -> Dict[str, Any]:
        """Get performance metrics for a specific strategy."""
        if strategy not in self.strategy_history:
            return {'error': f'No history found for strategy: {strategy}'}
        
        move_ids = self.strategy_history[strategy]
        moves = [self.move_history[mid] for mid in move_ids if mid in self.move_history]
        
        if not moves:
            return {'error': f'No valid moves found for strategy: {strategy}'}
        
        # Calculate performance metrics
        completed_moves = [m for m in moves if m.status == MoveStatus.COMPLETED]
        
        if not completed_moves:
            return {
                'strategy': strategy,
                'total_moves': len(moves),
                'completed_moves': 0,
                'success_rate': 0.0
            }
        
        metrics = {
            'strategy': strategy,
            'total_moves': len(moves),
            'completed_moves': len(completed_moves),
            'success_rate': len(completed_moves) / len(moves),
            
            # Efficiency metrics
            'avg_objects_per_move': statistics.mean(m.actual_objects for m in completed_moves),
            'total_objects_transported': sum(m.actual_objects for m in completed_moves),
            'avg_efficiency_score': statistics.mean(m.efficiency_score for m in completed_moves),
            
            # Spillage metrics
            'avg_spillage_per_move': statistics.mean(m.actual_spillage for m in completed_moves),
            'total_spillage': sum(m.actual_spillage for m in completed_moves),
            'avg_spillage_ratio': statistics.mean(m.spillage_ratio for m in completed_moves),
            
            # Timing metrics
            'avg_move_duration': statistics.mean(m.total_duration for m in completed_moves),
            'total_execution_time': sum(m.total_duration for m in completed_moves),
            
            # Prediction accuracy
            'avg_prediction_accuracy': statistics.mean(m.prediction_accuracy for m in completed_moves),
            
            # Path metrics
            'avg_path_length': statistics.mean(m.path_length for m in completed_moves),
            'highway_usage_rate': sum(1 for m in completed_moves if m.path_type == "highway") / len(completed_moves)
        }
        
        return metrics
    
    def analyze_efficiency_trends(self, 
                                 scenario_id: Optional[str] = None,
                                 time_window: Optional[Tuple[float, float]] = None) -> PerformanceAnalysis:
        """Analyze efficiency trends over time."""
        # Filter moves based on criteria
        moves = self._filter_moves(scenario_id=scenario_id, time_window=time_window)
        completed_moves = [m for m in moves if m.status == MoveStatus.COMPLETED]
        
        if not completed_moves:
            return PerformanceAnalysis(
                analysis_type=AnalysisType.EFFICIENCY_TRENDS,
                analysis_timestamp=time.time(),
                total_moves_analyzed=0,
                time_period=time_window or (0, time.time()),
                summary={'error': 'No completed moves found for analysis'}
            )
        
        # Sort moves by timestamp
        completed_moves.sort(key=lambda m: m.planned_timestamp)
        
        # Calculate trends
        efficiency_scores = [m.efficiency_score for m in completed_moves]
        objects_per_move = [m.actual_objects for m in completed_moves]
        spillage_ratios = [m.spillage_ratio for m in completed_moves]
        
        # Calculate moving averages
        window_size = min(5, len(completed_moves))
        efficiency_trend = self._calculate_moving_average(efficiency_scores, window_size)
        objects_trend = self._calculate_moving_average(objects_per_move, window_size)
        spillage_trend = self._calculate_moving_average(spillage_ratios, window_size)
        
        analysis = PerformanceAnalysis(
            analysis_type=AnalysisType.EFFICIENCY_TRENDS,
            analysis_timestamp=time.time(),
            total_moves_analyzed=len(completed_moves),
            time_period=(completed_moves[0].planned_timestamp, completed_moves[-1].planned_timestamp),
            
            metrics={
                'avg_efficiency': statistics.mean(efficiency_scores),
                'efficiency_std': statistics.stdev(efficiency_scores) if len(efficiency_scores) > 1 else 0,
                'avg_objects_per_move': statistics.mean(objects_per_move),
                'avg_spillage_ratio': statistics.mean(spillage_ratios),
                'trend_slope_efficiency': self._calculate_trend_slope(efficiency_trend),
                'trend_slope_objects': self._calculate_trend_slope(objects_trend)
            },
            
            trends={
                'efficiency_scores': efficiency_scores,
                'objects_per_move': objects_per_move,
                'spillage_ratios': spillage_ratios,
                'efficiency_trend': efficiency_trend,
                'objects_trend': objects_trend,
                'spillage_trend': spillage_trend
            }
        )
        
        # Generate recommendations
        if analysis.metrics['trend_slope_efficiency'] > 0.01:
            analysis.recommendations.append("Efficiency is improving over time - current strategy is effective")
        elif analysis.metrics['trend_slope_efficiency'] < -0.01:
            analysis.recommendations.append("Efficiency is declining - consider strategy adjustment")
        
        if analysis.metrics['avg_spillage_ratio'] > 0.1:
            analysis.recommendations.append("High spillage ratio detected - focus on spillage minimization")
        
        return analysis
    
    def analyze_strategy_comparison(self, strategies: List[str]) -> Dict[str, Any]:
        """Compare performance across multiple strategies."""
        comparison = {
            'strategies': {},
            'ranking': {},
            'summary': {
                'total_strategies': len(strategies),
                'analysis_timestamp': time.time()
            }
        }
        
        strategy_metrics = []
        
        for strategy in strategies:
            performance = self.get_strategy_performance(strategy)
            if 'error' not in performance:
                comparison['strategies'][strategy] = performance
                strategy_metrics.append((strategy, performance))
        
        if not strategy_metrics:
            comparison['summary']['error'] = 'No valid strategy data found'
            return comparison
        
        # Rank strategies by different criteria
        comparison['ranking']['by_efficiency'] = sorted(
            strategy_metrics, 
            key=lambda x: x[1]['avg_efficiency_score'], 
            reverse=True
        )
        
        comparison['ranking']['by_success_rate'] = sorted(
            strategy_metrics,
            key=lambda x: x[1]['success_rate'],
            reverse=True
        )
        
        comparison['ranking']['by_objects_transported'] = sorted(
            strategy_metrics,
            key=lambda x: x[1]['total_objects_transported'],
            reverse=True
        )
        
        comparison['ranking']['by_spillage'] = sorted(
            strategy_metrics,
            key=lambda x: x[1]['avg_spillage_ratio']
        )  # Lower is better for spillage
        
        # Calculate overall best strategy
        best_strategy = max(strategy_metrics, key=lambda x: (
            x[1]['success_rate'] * 0.3 + 
            x[1]['avg_efficiency_score'] * 0.3 +
            (1 - x[1]['avg_spillage_ratio']) * 0.2 +
            x[1]['avg_prediction_accuracy'] * 0.2
        ))
        
        comparison['summary']['best_overall_strategy'] = best_strategy[0]
        
        return comparison
    
    def export_history(self, filepath: str, include_state_snapshots: bool = False) -> bool:
        """Export move history to file."""
        try:
            export_data = {
                'metadata': {
                    'export_timestamp': time.time(),
                    'total_moves': len(self.move_history),
                    'total_scenarios': len(self.scenario_history),
                    'tracking_duration': time.time() - self.tracking_start_time
                },
                'move_history': {mid: asdict(move) for mid, move in self.move_history.items()},
                'scenario_history': self.scenario_history,
                'strategy_history': self.strategy_history,
                'agent_history': self.agent_history
            }
            
            if include_state_snapshots:
                export_data['state_snapshots'] = [asdict(snapshot) for snapshot in self.state_snapshots]
            
            with open(filepath, 'w') as f:
                json.dump(export_data, f, indent=2, default=str)
            
            print(f"Exported move history to {filepath}")
            return True
            
        except Exception as e:
            print(f"Error exporting history: {e}")
            return False
    
    def import_history(self, filepath: str) -> bool:
        """Import move history from file."""
        try:
            with open(filepath, 'r') as f:
                import_data = json.load(f)
            
            # Import move history
            for move_id, move_data in import_data['move_history'].items():
                # Reconstruct move record with proper dataclass handling
                move_record = DetailedMoveRecord(**move_data)
                # Handle enum conversion
                move_record.status = MoveStatus(move_record.status)
                
                self.move_history[move_id] = move_record
            
            # Import tracking structures
            self.scenario_history = import_data.get('scenario_history', {})
            self.strategy_history = import_data.get('strategy_history', {})
            self.agent_history = {int(k): v for k, v in import_data.get('agent_history', {}).items()}
            
            # Import state snapshots if available
            if 'state_snapshots' in import_data:
                self.state_snapshots = [StateSnapshot(**snapshot) for snapshot in import_data['state_snapshots']]
            
            # Update counters
            self.total_moves_recorded = len(self.move_history)
            self.total_scenarios_tracked = len(self.scenario_history)
            
            print(f"Imported {self.total_moves_recorded} moves from {filepath}")
            return True
            
        except Exception as e:
            print(f"Error importing history: {e}")
            return False
    
    # Private helper methods
    
    def _create_state_snapshot(self, env: SimulationEnv, move_id: Optional[str] = None) -> StateSnapshot:
        """Create a state snapshot from environment."""
        total_objects = sum(cell.num_objects for cell in env.cells_with_objects)
        objects_in_target = sum(cell.num_objects for cell in env.target_zone_cells)
        
        # Calculate heat map statistics
        heat_values = [getattr(cell, 'heat_map', 0) for cell in env.all_cells]
        heat_max = max(heat_values) if heat_values else 0
        heat_avg = statistics.mean(heat_values) if heat_values else 0
        
        # Create state hash
        state_data = {
            'total_objects': total_objects,
            'cells_with_objects': [(c.x, c.y, c.num_objects) for c in env.cells_with_objects],
            'agent_positions': [agent["position"] for agent in env.agents]
        }
        state_hash = hashlib.md5(str(sorted(state_data.items())).encode()).hexdigest()
        
        snapshot = StateSnapshot(
            timestamp=time.time(),
            move_id=move_id,
            total_objects=total_objects,
            objects_in_target=objects_in_target,
            objects_transported=0,  # Would need historical tracking
            agent_positions=[agent["position"] for agent in env.agents],
            heat_map_max=heat_max,
            heat_map_avg=heat_avg,
            highway_threshold=getattr(env, 'highway_threshold', 0),
            cells_with_objects_count=len(env.cells_with_objects),
            state_hash=state_hash
        )
        
        return snapshot
    
    def _filter_moves(self, 
                     scenario_id: Optional[str] = None,
                     strategy: Optional[str] = None,
                     agent_id: Optional[int] = None,
                     time_window: Optional[Tuple[float, float]] = None) -> List[DetailedMoveRecord]:
        """Filter moves based on various criteria."""
        moves = list(self.move_history.values())
        
        if scenario_id:
            moves = [m for m in moves if m.scenario_id == scenario_id]
        
        if strategy:
            moves = [m for m in moves if m.strategy_used == strategy]
        
        if agent_id is not None:
            moves = [m for m in moves if m.agent_id == agent_id]
        
        if time_window:
            start_time, end_time = time_window
            moves = [m for m in moves if start_time <= m.planned_timestamp <= end_time]
        
        return moves
    
    def _calculate_moving_average(self, values: List[float], window_size: int) -> List[float]:
        """Calculate moving average for a list of values."""
        if len(values) < window_size:
            return values
        
        moving_avg = []
        for i in range(len(values) - window_size + 1):
            window = values[i:i + window_size]
            moving_avg.append(statistics.mean(window))
        
        return moving_avg
    
    def _calculate_trend_slope(self, values: List[float]) -> float:
        """Calculate trend slope using simple linear regression."""
        if len(values) < 2:
            return 0.0
        
        n = len(values)
        x_values = list(range(n))
        
        # Calculate slope using least squares
        x_mean = statistics.mean(x_values)
        y_mean = statistics.mean(values)
        
        numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_values, values))
        denominator = sum((x - x_mean) ** 2 for x in x_values)
        
        if denominator == 0:
            return 0.0
        
        slope = numerator / denominator
        return slope
    
    def _auto_save(self):
        """Automatically save data to storage path."""
        if not self.storage_path:
            return
        
        try:
            # Save to timestamped file
            timestamp = int(time.time())
            filename = f"move_history_{timestamp}.json"
            filepath = self.storage_path / filename
            
            self.export_history(str(filepath))
            
        except Exception as e:
            print(f"Auto-save failed: {e}")


if __name__ == "__main__":
    # Example usage and testing
    print("Testing MoveHistoryTracker...")
    
    # Create tracker
    tracker = MoveHistoryTracker()
    
    # Create mock environment for testing
    from core_env import SimulationEnv
    
    env = SimulationEnv(grid_size=10, target_zone_radius=2, agent_positions=[(2, 2, 0)], num_random_objects=5)
    
    # Record some sample moves
    move_id_1 = "test_move_1"
    tracker.record_move_plan(
        move_id=move_id_1,
        scenario_id="test_scenario",
        strategy="greedy_nearest",
        agent_id=0,
        source_cell=(2, 2),
        target_cell=(5, 5),
        path_coords=[(2, 2), (3, 3), (4, 4), (5, 5)],
        path_type="target",
        expected_outcomes={'objects': 3, 'spillage': 0.1, 'distance': 4.2, 'time': 2.5},
        pre_move_env=env
    )
    
    # Record execution
    tracker.record_move_execution_start(move_id_1)
    time.sleep(0.1)  # Simulate execution time
    
    tracker.record_move_execution_complete(
        move_id_1,
        actual_outcomes={'objects': 2, 'spillage': 0.15, 'distance': 4.1},
        post_move_env=env
    )
    
    # Analyze performance
    strategy_performance = tracker.get_strategy_performance("greedy_nearest")
    print(f"Strategy performance: {strategy_performance['success_rate']:.2f} success rate")
    
    # Analyze trends
    efficiency_analysis = tracker.analyze_efficiency_trends()
    print(f"Efficiency analysis completed: {efficiency_analysis.total_moves_analyzed} moves analyzed")
    
    print("MoveHistoryTracker testing completed successfully!")