"""
Benchmark Configuration System
Defines comprehensive test matrices and parameter combinations for performance evaluation
"""

import itertools
from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
from enum import Enum

class BenchmarkType(Enum):
    FULL_CALCULATION = "full_calculation"
    ENVIRONMENT_UPDATE = "environment_update"
    SPILLAGE_COMPARISON = "spillage_comparison"
    STRATEGIC_ANALYSIS = "strategic_analysis"
    SAVE_LOAD_PERFORMANCE = "save_load_performance"

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
            'grid_sizes': self.grid_sizes,
            'object_counts': self.object_counts,
            'seeds': self.seeds,
            'iterations': 10
        }
        
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
        
        for grid_size, object_count, seed in itertools.product(
            update_grid_sizes,
            update_object_counts,
            config.get('seeds', self.seeds)[:3]  # Use fewer seeds for update tests
        ):
            scenario_id = f"update_{len(scenarios):03d}"
            scenarios.append(BenchmarkScenario(
                scenario_id=scenario_id,
                grid_size=grid_size,
                object_count=object_count,
                seed=seed,
                use_spillage=True,  # Updates are more complex with spillage
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
    
    def generate_all_scenarios(self, config_type='comprehensive') -> Dict[BenchmarkType, List[BenchmarkScenario]]:
        """Generate complete benchmark suite"""
        all_scenarios = {
            BenchmarkType.FULL_CALCULATION: self.generate_full_calculation_scenarios(config_type),
            BenchmarkType.ENVIRONMENT_UPDATE: self.generate_update_performance_scenarios(config_type),
            BenchmarkType.SPILLAGE_COMPARISON: self.generate_spillage_comparison_scenarios(config_type),
            BenchmarkType.STRATEGIC_ANALYSIS: self.generate_strategic_analysis_scenarios(config_type),
            BenchmarkType.SAVE_LOAD_PERFORMANCE: self.generate_save_load_scenarios(config_type)
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
            BenchmarkType.SAVE_LOAD_PERFORMANCE: 0.5  # 500ms per save/load
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

# Example usage
if __name__ == "__main__":
    config = BenchmarkConfig()
    
    print("QUICK TEST CONFIGURATION:")
    quick_scenarios = config.print_configuration_summary('quick')
    
    print("\n" + "="*60 + "\n")
    
    print("COMPREHENSIVE TEST CONFIGURATION:")
    comp_scenarios = config.print_configuration_summary('comprehensive')