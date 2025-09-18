"""
Benchmark Configuration System
Defines comprehensive test matrices and parameter combinations for performance evaluation
"""

import itertools
from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
from enum import Enum
import os

class BenchmarkType(Enum):
    FULL_CALCULATION = "full_calculation"
    ENVIRONMENT_UPDATE = "environment_update"
    SPILLAGE_COMPARISON = "spillage_comparison"
    STRATEGIC_ANALYSIS = "strategic_analysis"
    SAVE_LOAD_PERFORMANCE = "save_load_performance"
    A_STAR_OPTIMIZATION = "a_star_optimization"
    OPTIMIZATION_MATRIX = "optimization_matrix"
    SUFFIX_STITCHING_COMPARISON = "suffix_stitching_comparison"

class StrategyType(Enum):
    GREEDY_NEAREST = "greedy_nearest"
    EFFICIENCY_FIRST = "efficiency_first"
    HIGHWAY_FORMATION = "highway_formation"
    SPILLAGE_MINIMIZATION = "spillage_minimization"

@dataclass
class BenchmarkScenario:
    """Single benchmark scenario configuration"""
    scenario_id: str
    grid_size: int
    object_count: int
    seed: int
    use_spillage: bool
    strategy: StrategyType
    benchmark_type: BenchmarkType
    iterations: int = 10  # Number of repetitions for averaging
    use_suffix_stitching: bool = True  # A* suffix stitching optimization
    use_affected_only_updates: bool = True  # Environment update optimization
    
    def __str__(self):
        return f"{self.scenario_id}_{self.grid_size}x{self.grid_size}_{self.object_count}obj_s{self.seed}_{'spill' if self.use_spillage else 'nospill'}_{self.strategy.value}"

class BenchmarkConfig:
    """Comprehensive benchmark configuration generator"""
    
    def __init__(self):
        # Base parameter ranges
        self.grid_sizes = [15, 20, 25, 30, 35]  # Environment sizes
        self.object_counts = [20, 35, 55, 75, 100]  # Different object densities
        self.seeds = [31, 42, 123, 456, 789]  # Different random scenarios
        self.spillage_modes = [True, False]  # With/without spillage
        self.strategies = list(StrategyType)
        self.benchmark_types = list(BenchmarkType)
        
        # Performance test configurations
        self.quick_test_config = {
            'grid_sizes': [20, 25],
            'object_counts': [35, 55],
            'seeds': [31, 42],
            'iterations': 5
        }
        
        self.comprehensive_test_config = {
            'grid_sizes': [25, 35,45,65,100],
            'object_counts': [50,75,100,150],
            'seeds': [31, 42, 123, 456, 789],
            'iterations': 5
        }
        
        # Try to load custom config if available (after defaults are set)
        self._load_custom_config_if_exists()
        
        self.stress_test_config = {
            'grid_sizes': [40, 50, 60],
            'object_counts': [150, 200, 300],
            'seeds': self.seeds,
            'iterations': 3
        }
    
    def generate_full_calculation_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios for full calculation from scratch benchmarks"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        for grid_size, object_count, seed, use_spillage, strategy in itertools.product(
            config['grid_sizes'],
            config['object_counts'],
            self.seeds[:2] if config_type == 'quick' else config.get('seeds', self.seeds),
            self.spillage_modes,
            [StrategyType.GREEDY_NEAREST, StrategyType.EFFICIENCY_FIRST]  # Focus on main strategies
        ):
            scenario_id = f"full_calc_{len(scenarios):03d}"
            scenarios.append(BenchmarkScenario(
                scenario_id=scenario_id,
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=use_spillage,
                strategy=strategy,
                benchmark_type=BenchmarkType.FULL_CALCULATION,
                iterations=config['iterations']
            ))
        
        return scenarios
    
    def generate_update_performance_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios for environment update performance benchmarks"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        # Focus on larger environments where update performance matters most
        update_grid_sizes = [size for size in config['grid_sizes'] if size >= 25]
        update_object_counts = [count for count in config['object_counts'] if count >= 35]
        
        for grid_size, object_count, seed, use_spillage in itertools.product(
            update_grid_sizes,
            update_object_counts,
            config.get('seeds', self.seeds)[:3],  # Use fewer seeds for update tests
            [True, False]  # Test both spillage modes as requested
        ):
            scenario_id = f"update_{len(scenarios):03d}"
            scenarios.append(BenchmarkScenario(
                scenario_id=scenario_id,
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=use_spillage,
                strategy=StrategyType.GREEDY_NEAREST,
                benchmark_type=BenchmarkType.ENVIRONMENT_UPDATE,
                iterations=config['iterations'] * 2  # More iterations for update timing
            ))
        
        return scenarios
    
    def generate_spillage_comparison_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios comparing spillage vs no-spillage performance"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        for grid_size, object_count, seed, strategy in itertools.product(
            config['grid_sizes'],
            config['object_counts'],
            config.get('seeds', self.seeds),
            [StrategyType.GREEDY_NEAREST, StrategyType.EFFICIENCY_FIRST, StrategyType.SPILLAGE_MINIMIZATION]
        ):
            # Create paired scenarios: one with spillage, one without
            for use_spillage in [True, False]:
                scenario_id = f"spillage_comp_{len(scenarios):03d}"
                scenarios.append(BenchmarkScenario(
                    scenario_id=scenario_id,
                    grid_size=grid_size,
                    object_count=object_count,
                    seed=seed,
                    use_spillage=use_spillage,
                    strategy=strategy,
                    benchmark_type=BenchmarkType.SPILLAGE_COMPARISON,
                    iterations=config['iterations']
                ))
        
        return scenarios
    
    def generate_strategic_analysis_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios for strategic analysis performance benchmarks"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        for grid_size, object_count, seed in itertools.product(
            config['grid_sizes'],
            config['object_counts'],
            config.get('seeds', self.seeds)[:3]  # Analysis doesn't vary much by seed
        ):
            scenario_id = f"analysis_{len(scenarios):03d}"
            scenarios.append(BenchmarkScenario(
                scenario_id=scenario_id,
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=True,  # Analysis with full complexity
                strategy=StrategyType.GREEDY_NEAREST,  # Strategy doesn't affect analysis performance
                benchmark_type=BenchmarkType.STRATEGIC_ANALYSIS,
                iterations=config['iterations'] * 3  # More iterations for statistical significance
            ))
        
        return scenarios
    
    def generate_save_load_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios for save/load performance benchmarks"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        # Save/load performance scales with state complexity
        for grid_size, object_count, seed in itertools.product(
            config['grid_sizes'],
            config['object_counts'],
            config.get('seeds', self.seeds)[:2]  # Fewer seeds needed
        ):
            scenario_id = f"saveload_{len(scenarios):03d}"
            scenarios.append(BenchmarkScenario(
                scenario_id=scenario_id,
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=True,  # Full state complexity
                strategy=StrategyType.GREEDY_NEAREST,
                benchmark_type=BenchmarkType.SAVE_LOAD_PERFORMANCE,
                iterations=config['iterations'] * 2
            ))
        
        return scenarios
    
    def generate_suffix_stitching_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios comparing A* with and without suffix stitching"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        for grid_size, object_count, seed, use_spillage in itertools.product(
            config['grid_sizes'],
            config['object_counts'],
            config.get('seeds', self.seeds)[:3],  # Fewer seeds needed for optimization comparison
            [True, False]  # Test both spillage modes
        ):
            # Create paired scenarios: one with suffix stitching, one without
            for use_suffix_stitching in [True, False]:
                scenario_id = f"suffix_stitch_{len(scenarios):03d}"
                scenarios.append(BenchmarkScenario(
                    scenario_id=scenario_id,
                    grid_size=grid_size,
                    object_count=object_count,
                    seed=seed,
                    use_spillage=use_spillage,
                    strategy=StrategyType.GREEDY_NEAREST,  # Focus on one strategy for clear comparison
                    benchmark_type=BenchmarkType.SUFFIX_STITCHING_COMPARISON,
                    iterations=config['iterations'] * 2,  # More iterations for statistical significance
                    use_suffix_stitching=use_suffix_stitching,
                    use_affected_only_updates=True  # Keep other optimizations constant
                ))
        
        return scenarios
    
    def generate_optimization_matrix_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate complete optimization matrix: all combinations of optimizations"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        # Use moderate complexity for matrix comparison
        matrix_grid_sizes = [25, 30] if config_type == 'comprehensive' else [25]
        matrix_object_counts = [55, 75] if config_type == 'comprehensive' else [55]
        
        for grid_size, object_count, seed, use_spillage, use_suffix_stitching, use_affected_only in itertools.product(
            matrix_grid_sizes,
            matrix_object_counts,
            config.get('seeds', self.seeds)[:2],  # Fewer seeds for matrix
            [True, False],  # Spillage on/off
            [True, False],  # Suffix stitching on/off
            [True, False]   # Affected-only updates on/off
        ):
            scenario_id = f"opt_matrix_{len(scenarios):03d}"
            
            # Create descriptive name for optimization combination
            opt_name = []
            if use_suffix_stitching:
                opt_name.append("suffix")
            if use_affected_only:
                opt_name.append("affected_only")
            if use_spillage:
                opt_name.append("spillage")
            opt_combination = "_".join(opt_name) if opt_name else "baseline"
            
            scenarios.append(BenchmarkScenario(
                scenario_id=f"{scenario_id}_{opt_combination}",
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=use_spillage,
                strategy=StrategyType.GREEDY_NEAREST,
                benchmark_type=BenchmarkType.OPTIMIZATION_MATRIX,
                iterations=config['iterations'],
                use_suffix_stitching=use_suffix_stitching,
                use_affected_only_updates=use_affected_only
            ))
        
        return scenarios
    
    def generate_a_star_optimization_scenarios(self, config_type='comprehensive') -> List[BenchmarkScenario]:
        """Generate scenarios focused on A* algorithm optimization analysis"""
        config = getattr(self, f'{config_type}_test_config')
        scenarios = []
        
        # Focus on scenarios where A* optimization makes the most difference
        for grid_size, object_count, seed in itertools.product(
            [size for size in config['grid_sizes'] if size >= 25],  # Larger grids benefit more
            [count for count in config['object_counts'] if count >= 35],  # More objects = more paths
            config.get('seeds', self.seeds)[:3]
        ):
            for use_suffix_stitching in [True, False]:
                scenario_id = f"astar_opt_{len(scenarios):03d}"
                scenarios.append(BenchmarkScenario(
                    scenario_id=scenario_id,
                    grid_size=grid_size,
                    object_count=object_count,
                    seed=seed,
                    use_spillage=False,  # Test A* optimization without spillage complexity
                    strategy=StrategyType.GREEDY_NEAREST,
                    benchmark_type=BenchmarkType.A_STAR_OPTIMIZATION,
                    iterations=config['iterations'] * 3,  # More iterations for timing precision
                    use_suffix_stitching=use_suffix_stitching,
                    use_affected_only_updates=True  # Keep environment updates optimized
                ))
        
        return scenarios
    
    def generate_all_scenarios(self, config_type='comprehensive') -> Dict[BenchmarkType, List[BenchmarkScenario]]:
        """Generate complete benchmark suite"""
        all_scenarios = {
            BenchmarkType.FULL_CALCULATION: self.generate_full_calculation_scenarios(config_type),
            BenchmarkType.ENVIRONMENT_UPDATE: self.generate_update_performance_scenarios(config_type),
            BenchmarkType.SPILLAGE_COMPARISON: self.generate_spillage_comparison_scenarios(config_type),
            BenchmarkType.STRATEGIC_ANALYSIS: self.generate_strategic_analysis_scenarios(config_type),
            BenchmarkType.SAVE_LOAD_PERFORMANCE: self.generate_save_load_scenarios(config_type),
            BenchmarkType.A_STAR_OPTIMIZATION: self.generate_a_star_optimization_scenarios(config_type),
            BenchmarkType.SUFFIX_STITCHING_COMPARISON: self.generate_suffix_stitching_scenarios(config_type),
            BenchmarkType.OPTIMIZATION_MATRIX: self.generate_optimization_matrix_scenarios(config_type)
        }
        
        return all_scenarios
    
    def get_scenario_summary(self, scenarios: Dict[BenchmarkType, List[BenchmarkScenario]]) -> Dict[str, Any]:
        """Get summary statistics of benchmark scenarios"""
        total_scenarios = sum(len(scenario_list) for scenario_list in scenarios.values())
        total_iterations = sum(
            sum(scenario.iterations for scenario in scenario_list) 
            for scenario_list in scenarios.values()
        )
        
        return {
            'total_benchmark_types': len(scenarios),
            'total_scenarios': total_scenarios,
            'total_iterations': total_iterations,
            'scenarios_by_type': {
                bench_type.value: len(scenario_list) 
                for bench_type, scenario_list in scenarios.items()
            },
            'grid_size_range': f"{min(self.grid_sizes)}-{max(self.grid_sizes)}",
            'object_count_range': f"{min(self.object_counts)}-{max(self.object_counts)}",
            'seed_count': len(self.seeds)
        }
    
    def print_configuration_summary(self, config_type='comprehensive'):
        """Print detailed configuration summary"""
        scenarios = self.generate_all_scenarios(config_type)
        summary = self.get_scenario_summary(scenarios)
        
        print(f"=== BENCHMARK CONFIGURATION SUMMARY ({config_type.upper()}) ===")
        print(f"Total Benchmark Types: {summary['total_benchmark_types']}")
        print(f"Total Scenarios: {summary['total_scenarios']}")
        print(f"Total Test Iterations: {summary['total_iterations']:,}")
        print(f"Grid Size Range: {summary['grid_size_range']}")
        print(f"Object Count Range: {summary['object_count_range']}")
        print(f"Seeds: {len(self.seeds)} different random scenarios")
        print()
        
        print("Scenarios by Type:")
        for bench_type, count in summary['scenarios_by_type'].items():
            print(f"  {bench_type.replace('_', ' ').title()}: {count} scenarios")
        
        print()
        estimated_time = self._estimate_total_time(summary['total_iterations'])
        print(f"Estimated Total Runtime: {estimated_time}")
        
        return scenarios
    
    def _estimate_total_time(self, total_iterations: int) -> str:
        """Estimate total benchmark runtime"""
        # Rough estimates based on complexity
        avg_time_per_iteration = {
            BenchmarkType.FULL_CALCULATION: 2.0,  # 2 seconds per full calculation
            BenchmarkType.ENVIRONMENT_UPDATE: 0.1,  # 100ms per update
            BenchmarkType.SPILLAGE_COMPARISON: 1.5,  # 1.5s per comparison
            BenchmarkType.STRATEGIC_ANALYSIS: 0.3,  # 300ms per analysis
            BenchmarkType.SAVE_LOAD_PERFORMANCE: 0.5,  # 500ms per save/load
            BenchmarkType.A_STAR_OPTIMIZATION: 1.8,  # 1.8s per A* optimization test
            BenchmarkType.SUFFIX_STITCHING_COMPARISON: 1.2,  # 1.2s per suffix stitching test
            BenchmarkType.OPTIMIZATION_MATRIX: 2.5  # 2.5s per matrix combination test
        }
        
        # Simplified estimation using average
        avg_time = sum(avg_time_per_iteration.values()) / len(avg_time_per_iteration)
        total_seconds = total_iterations * avg_time
        
        if total_seconds < 60:
            return f"{total_seconds:.0f} seconds"
        elif total_seconds < 3600:
            return f"{total_seconds/60:.1f} minutes"
        else:
            return f"{total_seconds/3600:.1f} hours"
    
    def _load_custom_config_if_exists(self):
        """Load custom configuration from file if it exists"""
        config_file = "custom_benchmark_config.txt"
        if os.path.exists(config_file):
            try:
                print(f"Loading custom benchmark config from {config_file}")
                with open(config_file, 'r') as f:
                    lines = f.readlines()
                
                for line in lines:
                    line = line.strip()
                    if line.startswith('#') or not line:
                        continue
                    if '=' in line:
                        key, value = line.split('=', 1)
                        key = key.strip()
                        value = value.strip()
                        
                        # Parse list values
                        if value.startswith('[') and value.endswith(']'):
                            # Parse list of numbers
                            value = value[1:-1]  # Remove brackets
                            if value:
                                parsed_list = [int(x.strip()) for x in value.split(',')]
                                if key == 'grid_sizes':
                                    self.comprehensive_test_config['grid_sizes'] = parsed_list
                                elif key == 'object_counts':
                                    self.comprehensive_test_config['object_counts'] = parsed_list
                                elif key == 'seeds':
                                    self.comprehensive_test_config['seeds'] = parsed_list
                        elif key == 'iterations':
                            self.comprehensive_test_config['iterations'] = int(value)
                
                print("Custom config loaded successfully!")
            except Exception as e:
                print(f"Error loading custom config: {e}")
                print("Using default comprehensive config")

# Example usage
if __name__ == "__main__":
    config = BenchmarkConfig()
    
    print("QUICK TEST CONFIGURATION:")
    quick_scenarios = config.print_configuration_summary('quick')
    
    print("\n" + "="*60 + "\n")
    
    print("COMPREHENSIVE TEST CONFIGURATION:")
    comp_scenarios = config.print_configuration_summary('comprehensive')