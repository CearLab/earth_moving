"""
Automated Benchmark Test Runner
Orchestrates execution of comprehensive performance benchmarks
"""

import time
import sys
import os
from pathlib import Path
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
import traceback
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from datetime import datetime

# Import our benchmark infrastructure
from benchmark_config import BenchmarkConfig, BenchmarkScenario, BenchmarkType, StrategyType
from benchmark_performance_tracker import PerformanceTracker, BenchmarkResult

# Import strategic demo infrastructure
from demo_interactive_comprehensive import ComprehensiveDemo
from strategic_orchestrator import StrategicOrchestrator

@dataclass
class BenchmarkRunConfig:
    """Configuration for benchmark execution"""
    output_directory: str = "benchmark_results"
    parallel_execution: bool = False
    max_workers: int = 4
    save_individual_results: bool = True
    continue_on_error: bool = True
    verbose: bool = True
    create_summary_report: bool = True

class BenchmarkRunner:
    """Automated benchmark execution system"""
    
    def __init__(self, run_config: BenchmarkRunConfig = None):
        self.run_config = run_config or BenchmarkRunConfig()
        self.config = BenchmarkConfig()
        self.tracker = PerformanceTracker(enable_memory_profiling=True)
        
        # Create output directory
        self.output_dir = Path(self.run_config.output_directory)
        self.output_dir.mkdir(exist_ok=True)
        
        # Results storage
        self.benchmark_results: Dict[str, List[BenchmarkResult]] = {}
        self.failed_scenarios: List[tuple] = []  # (scenario, error)
        self.execution_log: List[str] = []
        
        # Progress tracking
        self.total_scenarios = 0
        self.completed_scenarios = 0
        self.start_time = None
    
    def log(self, message: str, level: str = "INFO"):
        """Log message with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {level}: {message}"
        self.execution_log.append(log_entry)
        
        if self.run_config.verbose:
            print(log_entry)
    
    def create_demo_environment(self, scenario: BenchmarkScenario) -> ComprehensiveDemo:
        """Create configured demo environment for scenario"""
        demo = ComprehensiveDemo()
        
        # Configure the environment according to scenario
        success = demo.create_simulation_quick(
            use_spillage=scenario.use_spillage,
            grid_size=scenario.grid_size,
            object_count=scenario.object_count,
            seed=scenario.seed
        )
        
        if not success:
            raise RuntimeError(f"Failed to create simulation for scenario {scenario.scenario_id}")
        
        return demo
    
    def run_full_calculation_benchmark(self, scenario: BenchmarkScenario) -> BenchmarkResult:
        """Benchmark full calculation from scratch"""
        demo = self.create_demo_environment(scenario)
        
        def full_calculation():
            """Perform complete strategic calculation from scratch"""
            # Clear any cached data first
            if hasattr(demo, 'orchestrator') and demo.orchestrator:
                # Clear cached calculations
                if hasattr(demo.orchestrator, 'clear_all_cached_data'):
                    demo.orchestrator.clear_all_cached_data()
            
            # Force complete recalculation
            demo.calculate_strategic_fields_now()
            
            return {
                'grid_size': scenario.grid_size,
                'object_count': scenario.object_count,
                'total_cells': scenario.grid_size ** 2,
                'calculation_type': 'full_from_scratch'
            }
        
        return self.tracker.run_benchmark(
            operation_name=f"full_calculation_{scenario.grid_size}x{scenario.grid_size}_{scenario.object_count}obj",
            operation_func=full_calculation,
            iterations=scenario.iterations,
            scenario_id=scenario.scenario_id,
            grid_size=scenario.grid_size,
            object_count=scenario.object_count,
            use_spillage=scenario.use_spillage,
            strategy=scenario.strategy.value
        )
    
    def run_environment_update_benchmark(self, scenario: BenchmarkScenario) -> BenchmarkResult:
        """Benchmark realistic environment update performance using actual strategic paths"""
        demo = self.create_demo_environment(scenario)
        
        # First do a full calculation to have baseline
        demo.calculate_strategic_fields_now()
        
        # Generate several different strategic moves for varied complexity testing
        strategic_moves = self._generate_strategic_moves_for_benchmark(demo, scenario)
        if not strategic_moves:
            raise RuntimeError("No strategic moves available for environment update benchmark")
        
        move_index = 0
        
        # Pre-execute strategic moves OUTSIDE the timing measurement
        executed_states = []
        
        print(f"Pre-executing {scenario.iterations} strategic moves for pure update measurement...")
        
        for i in range(scenario.iterations):
            move = strategic_moves[i % len(strategic_moves)]
            
            # Execute the strategic path to set up affected_cells
            try:
                path_start = time.time()
                demo.env.execute_path(
                    start_cell=move['source_cell'],
                    path_type=move['path_type'], 
                    use_spillage=scenario.use_spillage,
                    precomputed_path=move.get('path_info')
                )
                path_time = (time.time() - path_start) * 1000
                
                # Save complete environment state after path execution (before update_environment)
                affected_cells_backup = getattr(demo.env, 'affected_cells', []).copy() if hasattr(demo.env, 'affected_cells') else []
                spillage_cells_backup = getattr(demo.env, 'spillage_affected_cells', []).copy() if hasattr(demo.env, 'spillage_affected_cells') else []
                
                # Save full environment state snapshot for restoration
                import copy
                full_env_backup = {
                    'cells_grid': copy.deepcopy([[cell for cell in row] for row in demo.env.grid]),
                    'cells_with_objects': demo.env.cells_with_objects.copy(),
                    'target_zones': demo.env.target_zones.copy(),
                    'affected_cells': affected_cells_backup,
                    'spillage_affected_cells': spillage_cells_backup
                }
                
                # Capture the state for pure update measurement
                state_info = {
                    'move': move,
                    'path_execution_time_ms': path_time,
                    'affected_cells_backup': affected_cells_backup,
                    'spillage_cells_backup': spillage_cells_backup,
                    'full_env_backup': full_env_backup,
                    'affected_cells_count': len(affected_cells_backup),
                    'spillage_cells_count': len(spillage_cells_backup)
                }
                executed_states.append(state_info)
                
            except Exception as e:
                executed_states.append({
                    'move': move,
                    'error': str(e),
                    'path_execution_time_ms': 0
                })
        
        print(f"Pre-execution complete. {len(executed_states)} states prepared.")
        state_index = 0
        
        def pure_environment_update():
            """Measure ONLY the environment update time, with pre-executed strategic states"""
            nonlocal state_index
            
            if state_index >= len(executed_states):
                # Reset to beginning to allow continued iterations
                state_index = 0
            
            state = executed_states[state_index]
            state_index += 1
            
            if 'error' in state:
                return {
                    'update_type': 'failed', 
                    'reason': state['error'],
                    'move_type': state['move']['path_type']
                }
            
            # Restore complete environment state from pre-execution snapshot
            backup = state['full_env_backup']
            demo.env.grid = backup['cells_grid']
            demo.env.cells_with_objects = backup['cells_with_objects']
            demo.env.target_zones = backup['target_zones']
            demo.env.affected_cells = backup['affected_cells']
            if hasattr(demo.env, 'spillage_affected_cells'):
                demo.env.spillage_affected_cells = backup['spillage_affected_cells']
            
            # NOW we only measure the update environment call - NO path execution
            try:
                demo.env.update_environment()  # This is what we're measuring
                
                # Collect metrics about the update complexity  
                metrics = {
                    'move_type': state['move']['path_type'],
                    'source_position': (state['move']['source_cell'].x, state['move']['source_cell'].y),
                    'path_length': state['move'].get('path_length', 0),
                    'objects_moved': state['move'].get('objects_moved', 0),
                    'path_execution_time_ms': state['path_execution_time_ms'],  # Separate timing
                    'update_type': 'pure_incremental',
                    'pre_affected_cells': state['affected_cells_count'],
                    'pre_spillage_cells': state['spillage_cells_count']
                }
                
                # Add post-update complexity metrics
                if hasattr(demo.env, 'direct_affected_cells'):
                    metrics['direct_affected_count'] = len(demo.env.direct_affected_cells)
                if hasattr(demo.env, 'spillage_cells'):
                    metrics['spillage_cells_count'] = len(demo.env.spillage_cells)
                if hasattr(demo.env, 'recalculation_cells'):
                    metrics['recalculation_count'] = len(demo.env.recalculation_cells)
                
                # Calculate total cells that needed processing
                total_affected = 0
                if hasattr(demo.env, 'direct_affected_cells'):
                    total_affected += len(demo.env.direct_affected_cells)
                if hasattr(demo.env, 'recalculation_cells'):
                    total_affected += len(demo.env.recalculation_cells)
                metrics['total_cells_processed'] = total_affected
                
                return metrics
                
            except Exception as e:
                return {
                    'update_type': 'failed', 
                    'reason': str(e),
                    'move_type': state['move']['path_type']
                }
        
        return self.tracker.run_benchmark(
            operation_name=f"pure_env_update_{scenario.grid_size}x{scenario.grid_size}",
            operation_func=pure_environment_update,
            iterations=scenario.iterations,
            scenario_id=scenario.scenario_id,
            grid_size=scenario.grid_size,
            object_count=scenario.object_count
        )
    
    def _generate_strategic_moves_for_benchmark(self, demo, scenario) -> List[dict]:
        """Generate diverse strategic moves for realistic environment update benchmarking"""
        import random
        
        moves = []
        if not hasattr(demo, 'env') or not demo.env or not demo.env.cells_with_objects:
            return moves
            
        # Get cells with objects for move generation
        available_cells = [cell for cell in demo.env.cells_with_objects if not cell.is_target_zone]
        if not available_cells:
            return moves
            
        # Generate different types of moves for varied complexity
        move_types = ['target', 'highway'] if hasattr(demo.env.cells_with_objects[0], 'best_path_highway') else ['target']
        
        # Sample different cells and path types to create diverse moves
        random.seed(scenario.random_seed if hasattr(scenario, 'random_seed') else 42)
        sample_size = min(10, len(available_cells))  # Sample up to 10 different moves
        sampled_cells = random.sample(available_cells, sample_size)
        
        for cell in sampled_cells:
            for path_type in move_types:
                try:
                    # Get path information for this cell/type combination
                    if path_type == 'target':
                        if hasattr(cell, 'best_path_target') and cell.best_path_target:
                            path_info = {
                                'path': cell.best_path_target,
                                'objects': cell.num_objects,
                                'distance': len(cell.best_path_target) if cell.best_path_target else 0
                            }
                        else:
                            # Generate path on demand
                            path_info = demo.env.get_path_for_preview(cell, 'target')
                    else:  # highway
                        if hasattr(cell, 'best_path_highway') and cell.best_path_highway:
                            path_info = {
                                'path': cell.best_path_highway,
                                'objects': cell.num_objects,
                                'distance': len(cell.best_path_highway) if cell.best_path_highway else 0
                            }
                        else:
                            continue  # Skip if no highway path available
                    
                    if path_info and path_info.get('path'):
                        move = {
                            'source_cell': cell,
                            'path_type': path_type,
                            'path_info': path_info,
                            'path_length': len(path_info['path']),
                            'objects_moved': path_info.get('objects', cell.num_objects),
                            'complexity_estimate': len(path_info['path']) * cell.num_objects
                        }
                        moves.append(move)
                        
                except Exception as e:
                    print(f"Warning: Could not generate {path_type} move for cell ({cell.x}, {cell.y}): {e}")
                    continue
        
        if not moves:
            # Fallback: create at least one simple move
            if available_cells:
                cell = available_cells[0]
                moves.append({
                    'source_cell': cell,
                    'path_type': 'target',
                    'path_info': None,  # Will use cached path
                    'path_length': 1,
                    'objects_moved': cell.num_objects,
                    'complexity_estimate': cell.num_objects
                })
        
        # Sort by complexity for varied testing (simple to complex)
        moves.sort(key=lambda x: x.get('complexity_estimate', 0))
        print(f"Generated {len(moves)} strategic moves for environment update benchmark")
        return moves
    
    def run_spillage_comparison_benchmark(self, scenario: BenchmarkScenario) -> BenchmarkResult:
        """Compare performance with and without spillage"""
        results = {}
        
        for use_spillage in [True, False]:
            # Create demo with specific spillage setting
            scenario.use_spillage = use_spillage
            demo = self.create_demo_environment(scenario)
            
            def spillage_calculation():
                demo.calculate_strategic_fields_now()
                return {
                    'spillage_enabled': use_spillage,
                    'grid_size': scenario.grid_size,
                    'object_count': scenario.object_count
                }
            
            # Run benchmark for this spillage setting
            result = self.tracker.run_benchmark(
                operation_name=f"spillage_{'enabled' if use_spillage else 'disabled'}_{scenario.grid_size}x{scenario.grid_size}",
                operation_func=spillage_calculation,
                iterations=scenario.iterations // 2,  # Split iterations between both modes
                scenario_id=f"{scenario.scenario_id}_{'spill' if use_spillage else 'nospill'}",
                spillage_mode=use_spillage
            )
            
            results[f"spillage_{'enabled' if use_spillage else 'disabled'}"] = result
        
        # Return the spillage-enabled result as primary, but store both
        return results["spillage_enabled"]
    
    def run_strategic_analysis_benchmark(self, scenario: BenchmarkScenario) -> BenchmarkResult:
        """Benchmark strategic analysis performance"""
        demo = self.create_demo_environment(scenario)
        
        # Ensure strategic fields are calculated first
        demo.calculate_strategic_fields_now()
        
        def strategic_analysis():
            """Perform strategic analysis"""
            try:
                if hasattr(demo, 'orchestrator') and demo.orchestrator:
                    analysis_result = demo.orchestrator.analyze_current_situation()
                    return {
                        'analysis_successful': True,
                        'analysis_type': 'comprehensive',
                        'grid_size': scenario.grid_size,
                        'object_count': scenario.object_count
                    }
                else:
                    return {
                        'analysis_successful': False,
                        'reason': 'no_orchestrator',
                        'grid_size': scenario.grid_size,
                        'object_count': scenario.object_count
                    }
            except Exception as e:
                return {
                    'analysis_successful': False,
                    'error': str(e),
                    'grid_size': scenario.grid_size,
                    'object_count': scenario.object_count
                }
        
        return self.tracker.run_benchmark(
            operation_name=f"strategic_analysis_{scenario.grid_size}x{scenario.grid_size}",
            operation_func=strategic_analysis,
            iterations=scenario.iterations,
            scenario_id=scenario.scenario_id,
            grid_size=scenario.grid_size,
            object_count=scenario.object_count
        )
    
    def run_save_load_benchmark(self, scenario: BenchmarkScenario) -> BenchmarkResult:
        """Benchmark save/load performance"""
        demo = self.create_demo_environment(scenario)
        demo.calculate_strategic_fields_now()
        
        save_state_name = f"benchmark_save_{scenario.scenario_id}"
        
        def save_load_cycle():
            """Perform complete save and load cycle"""
            # Save current state
            save_start = time.perf_counter()
            save_success = demo.save_current_state(save_state_name)
            save_time = time.perf_counter() - save_start
            
            if not save_success:
                return {
                    'operation_successful': False,
                    'error': 'save_failed',
                    'save_time': save_time
                }
            
            # Load saved state
            load_start = time.perf_counter()
            load_success = demo.load_saved_state(save_state_name)
            load_time = time.perf_counter() - load_start
            
            return {
                'operation_successful': save_success and load_success,
                'save_time': save_time,
                'load_time': load_time,
                'total_cycle_time': save_time + load_time,
                'grid_size': scenario.grid_size,
                'object_count': scenario.object_count
            }
        
        return self.tracker.run_benchmark(
            operation_name=f"save_load_{scenario.grid_size}x{scenario.grid_size}",
            operation_func=save_load_cycle,
            iterations=scenario.iterations,
            scenario_id=scenario.scenario_id,
            grid_size=scenario.grid_size,
            object_count=scenario.object_count
        )
    
    def execute_scenario(self, scenario: BenchmarkScenario) -> Optional[BenchmarkResult]:
        """Execute single benchmark scenario"""
        try:
            self.log(f"Executing scenario: {scenario.scenario_id} ({scenario.benchmark_type.value})")
            
            # Select appropriate benchmark function
            benchmark_functions = {
                BenchmarkType.FULL_CALCULATION: self.run_full_calculation_benchmark,
                BenchmarkType.ENVIRONMENT_UPDATE: self.run_environment_update_benchmark,
                BenchmarkType.SPILLAGE_COMPARISON: self.run_spillage_comparison_benchmark,
                BenchmarkType.STRATEGIC_ANALYSIS: self.run_strategic_analysis_benchmark,
                BenchmarkType.SAVE_LOAD_PERFORMANCE: self.run_save_load_benchmark
            }
            
            benchmark_func = benchmark_functions.get(scenario.benchmark_type)
            if not benchmark_func:
                raise ValueError(f"Unknown benchmark type: {scenario.benchmark_type}")
            
            # Execute benchmark
            result = benchmark_func(scenario)
            
            # Store result
            if scenario.benchmark_type.value not in self.benchmark_results:
                self.benchmark_results[scenario.benchmark_type.value] = []
            
            self.benchmark_results[scenario.benchmark_type.value].append(result)
            
            self.completed_scenarios += 1
            progress = (self.completed_scenarios / self.total_scenarios) * 100
            self.log(f"Completed scenario {scenario.scenario_id} ({progress:.1f}% total progress)")
            
            return result
            
        except Exception as e:
            error_msg = f"Failed to execute scenario {scenario.scenario_id}: {str(e)}"
            self.log(error_msg, "ERROR")
            self.log(traceback.format_exc(), "DEBUG")
            
            self.failed_scenarios.append((scenario, str(e)))
            
            if not self.run_config.continue_on_error:
                raise
            
            return None
    
    def run_benchmark_suite(self, config_type: str = 'comprehensive', 
                          specific_types: List[BenchmarkType] = None) -> Dict[str, Any]:
        """Run complete benchmark suite"""
        self.log(f"Starting benchmark suite: {config_type}")
        self.start_time = time.perf_counter()
        
        # Generate scenarios
        all_scenarios = self.config.generate_all_scenarios(config_type)
        
        # Filter to specific types if requested
        if specific_types:
            filtered_scenarios = {}
            for bench_type in specific_types:
                if bench_type in all_scenarios:
                    filtered_scenarios[bench_type] = all_scenarios[bench_type]
            all_scenarios = filtered_scenarios
        
        # Calculate total scenarios
        self.total_scenarios = sum(len(scenarios) for scenarios in all_scenarios.values())
        self.completed_scenarios = 0
        
        self.log(f"Total scenarios to execute: {self.total_scenarios}")
        
        # Execute benchmarks
        if self.run_config.parallel_execution:
            self.log(f"Executing benchmarks in parallel (max_workers={self.run_config.max_workers})")
            self._run_parallel_benchmarks(all_scenarios)
        else:
            self.log("Executing benchmarks sequentially")
            self._run_sequential_benchmarks(all_scenarios)
        
        # Calculate total execution time
        total_time = time.perf_counter() - self.start_time
        
        self.log(f"Benchmark suite completed in {total_time:.2f} seconds")
        self.log(f"Successfully completed: {self.completed_scenarios}/{self.total_scenarios}")
        self.log(f"Failed scenarios: {len(self.failed_scenarios)}")
        
        # Generate results summary
        results_summary = self._generate_results_summary(total_time)
        
        # Save results
        if self.run_config.save_individual_results:
            self._save_detailed_results(config_type, results_summary)
        
        if self.run_config.create_summary_report:
            self._create_summary_report(config_type, results_summary)
        
        return results_summary
    
    def _run_sequential_benchmarks(self, all_scenarios: Dict[BenchmarkType, List[BenchmarkScenario]]):
        """Execute benchmarks sequentially"""
        for benchmark_type, scenarios in all_scenarios.items():
            self.log(f"Starting {benchmark_type.value} benchmarks ({len(scenarios)} scenarios)")
            
            for scenario in scenarios:
                self.execute_scenario(scenario)
    
    def _run_parallel_benchmarks(self, all_scenarios: Dict[BenchmarkType, List[BenchmarkScenario]]):
        """Execute benchmarks in parallel"""
        all_scenarios_flat = []
        for scenarios in all_scenarios.values():
            all_scenarios_flat.extend(scenarios)
        
        with ThreadPoolExecutor(max_workers=self.run_config.max_workers) as executor:
            future_to_scenario = {
                executor.submit(self.execute_scenario, scenario): scenario 
                for scenario in all_scenarios_flat
            }
            
            for future in as_completed(future_to_scenario):
                scenario = future_to_scenario[future]
                try:
                    result = future.result()
                except Exception as e:
                    self.log(f"Parallel execution error for {scenario.scenario_id}: {str(e)}", "ERROR")
    
    def _generate_results_summary(self, total_time: float) -> Dict[str, Any]:
        """Generate comprehensive results summary"""
        summary = {
            'execution_info': {
                'start_time': self.start_time,
                'total_execution_time': total_time,
                'total_scenarios': self.total_scenarios,
                'completed_scenarios': self.completed_scenarios,
                'failed_scenarios': len(self.failed_scenarios),
                'success_rate': (self.completed_scenarios / self.total_scenarios) * 100 if self.total_scenarios > 0 else 0
            },
            'system_info': self.tracker.get_system_info(),
            'benchmark_results': {},
            'performance_statistics': {},
            'failed_scenarios': [
                {'scenario_id': scenario.scenario_id, 'error': error} 
                for scenario, error in self.failed_scenarios
            ]
        }
        
        # Aggregate results by benchmark type
        for benchmark_type, results in self.benchmark_results.items():
            if not results:
                continue
            
            type_summary = {
                'total_runs': len(results),
                'total_iterations': sum(len(result.runs) for result in results),
                'average_time_ms': [],
                'memory_usage_mb': [],
                'scenarios': []
            }
            
            for result in results:
                type_summary['average_time_ms'].append(result.average_time_ms)
                type_summary['memory_usage_mb'].append(result.average_memory_delta)
                type_summary['scenarios'].append({
                    'scenario_id': result.scenario_id,
                    'average_time_ms': result.average_time_ms,
                    'std_deviation_ms': result.std_deviation * 1000,
                    'memory_delta_mb': result.average_memory_delta,
                    'parameters': result.parameters
                })
            
            # Calculate overall statistics
            if type_summary['average_time_ms']:
                times = type_summary['average_time_ms']
                type_summary['overall_avg_time_ms'] = sum(times) / len(times)
                type_summary['min_time_ms'] = min(times)
                type_summary['max_time_ms'] = max(times)
            
            if type_summary['memory_usage_mb']:
                memory = type_summary['memory_usage_mb']
                type_summary['overall_avg_memory_mb'] = sum(memory) / len(memory)
            
            summary['benchmark_results'][benchmark_type] = type_summary
        
        return summary
    
    def _save_detailed_results(self, config_type: str, summary: Dict[str, Any]):
        """Save detailed benchmark results"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"benchmark_detailed_{config_type}_{timestamp}.json"
        
        # Export all tracker results
        detailed_results = self.tracker.export_results()
        detailed_results['summary'] = summary
        
        with open(filename, 'w') as f:
            json.dump(detailed_results, f, indent=2)
        
        self.log(f"Detailed results saved to: {filename}")
    
    def _create_summary_report(self, config_type: str, summary: Dict[str, Any]):
        """Create human-readable summary report"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"benchmark_summary_{config_type}_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write("=" * 80 + "\n")
            f.write(f"EARTH MOVING ALGORITHM BENCHMARK REPORT\n")
            f.write(f"Configuration: {config_type.upper()}\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 80 + "\n\n")
            
            # Execution Summary
            exec_info = summary['execution_info']
            f.write("EXECUTION SUMMARY:\n")
            f.write(f"  Total Execution Time: {exec_info['total_execution_time']:.2f} seconds\n")
            f.write(f"  Scenarios Executed: {exec_info['completed_scenarios']}/{exec_info['total_scenarios']}\n")
            f.write(f"  Success Rate: {exec_info['success_rate']:.1f}%\n")
            f.write(f"  Failed Scenarios: {exec_info['failed_scenarios']}\n\n")
            
            # Performance Results
            f.write("PERFORMANCE RESULTS BY BENCHMARK TYPE:\n")
            for benchmark_type, results in summary['benchmark_results'].items():
                f.write(f"\n{benchmark_type.replace('_', ' ').title()}:\n")
                f.write(f"  Total Runs: {results['total_runs']}\n")
                f.write(f"  Total Iterations: {results['total_iterations']}\n")
                
                if 'overall_avg_time_ms' in results:
                    f.write(f"  Average Time: {results['overall_avg_time_ms']:.2f}ms\n")
                    f.write(f"  Time Range: {results['min_time_ms']:.2f}ms - {results['max_time_ms']:.2f}ms\n")
                
                if 'overall_avg_memory_mb' in results:
                    f.write(f"  Average Memory Usage: {results['overall_avg_memory_mb']:.2f}MB\n")
            
            # System Information
            f.write(f"\nSYSTEM INFORMATION:\n")
            sys_info = summary['system_info']
            f.write(f"  Python Version: {sys_info.get('python_version', 'Unknown')}\n")
            f.write(f"  CPU Count: {sys_info.get('cpu_count', 'Unknown')}\n")
            f.write(f"  Total Memory: {sys_info.get('memory_total_gb', 0):.1f}GB\n")
            f.write(f"  Platform: {sys_info.get('platform', 'Unknown')}\n")
        
        self.log(f"Summary report saved to: {filename}")

# Example usage and testing
if __name__ == "__main__":
    # Create benchmark runner with custom configuration
    run_config = BenchmarkRunConfig(
        output_directory="benchmark_results",
        parallel_execution=False,  # Start with sequential for testing
        verbose=True,
        create_summary_report=True
    )
    
    runner = BenchmarkRunner(run_config)
    
    # Run quick test first
    print("Running quick benchmark test...")
    results = runner.run_benchmark_suite('quick', specific_types=[BenchmarkType.FULL_CALCULATION])
    
    print(f"\nBenchmark completed!")
    print(f"Results available in: {runner.output_dir}")
    
    # Print basic statistics
    runner.tracker.print_benchmark_summary()