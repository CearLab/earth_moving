"""
Memory Analysis Tool for Benchmark Scenarios
Analyzes actual memory usage patterns and provides recommendations
"""

import psutil
import tracemalloc
import gc
from typing import Dict, Any
from benchmark_config import BenchmarkConfig, BenchmarkScenario, BenchmarkType

def analyze_memory_usage():
    """Analyze memory usage patterns for different scenario complexities"""
    
    # Start memory tracing
    tracemalloc.start()
    process = psutil.Process()
    
    config = BenchmarkConfig()
    
    print("=== MEMORY ANALYSIS FOR BENCHMARK SCENARIOS ===\n")
    
    # Test different grid sizes with fixed object count
    test_scenarios = [
        {"grid_size": 25, "object_count": 50, "description": "Small (25x25, 50 objects)"},
        {"grid_size": 35, "object_count": 75, "description": "Medium (35x35, 75 objects)"},
        {"grid_size": 45, "object_count": 100, "description": "Large (45x45, 100 objects)"},
        {"grid_size": 65, "object_count": 150, "description": "Very Large (65x65, 150 objects)"},
        {"grid_size": 100, "object_count": 200, "description": "Extreme (100x100, 200 objects)"},
    ]
    
    results = []
    
    for scenario_config in test_scenarios:
        print(f"Testing {scenario_config['description']}...")
        
        # Create scenario
        scenario = BenchmarkScenario(
            scenario_id=f"memory_test_{scenario_config['grid_size']}",
            grid_size=scenario_config['grid_size'],
            object_count=scenario_config['object_count'],
            seed=42,
            use_spillage=False,  # Test without spillage first
            strategy=config.strategies[0],
            benchmark_type=BenchmarkType.FULL_CALCULATION,
            iterations=1  # Single iteration for memory test
        )
        
        # Measure memory before
        gc.collect()
        memory_before = process.memory_info().rss / 1024 / 1024  # MB
        snapshot_before = tracemalloc.take_snapshot()
        
        try:
            # Import and create demo environment
            from benchmark_runner import BenchmarkRunner
            runner = BenchmarkRunner()
            demo = runner.create_demo_environment(scenario)
            
            # Measure memory after environment creation
            memory_after_env = process.memory_info().rss / 1024 / 1024  # MB
            
            # Run calculation
            demo.calculate_strategic_fields_now()
            
            # Measure memory after calculation
            memory_after_calc = process.memory_info().rss / 1024 / 1024  # MB
            snapshot_after = tracemalloc.take_snapshot()
            
            # Calculate memory usage
            env_memory = memory_after_env - memory_before
            calc_memory = memory_after_calc - memory_after_env
            total_memory = memory_after_calc - memory_before
            
            # Get top memory consumers
            top_stats = snapshot_after.compare_to(snapshot_before, 'lineno')[:3]
            
            result = {
                'scenario': scenario_config['description'],
                'grid_size': scenario_config['grid_size'],
                'object_count': scenario_config['object_count'],
                'complexity': scenario_config['grid_size'] * scenario_config['object_count'],
                'env_memory_mb': env_memory,
                'calc_memory_mb': calc_memory,
                'total_memory_mb': total_memory,
                'memory_per_cell': total_memory / (scenario_config['grid_size'] ** 2),
                'memory_per_object': total_memory / scenario_config['object_count'],
                'success': True,
                'top_memory_consumers': [str(stat) for stat in top_stats[:2]]
            }
            
            # Cleanup
            del demo
            gc.collect()
            
        except MemoryError as e:
            result = {
                'scenario': scenario_config['description'],
                'grid_size': scenario_config['grid_size'],
                'object_count': scenario_config['object_count'],
                'complexity': scenario_config['grid_size'] * scenario_config['object_count'],
                'success': False,
                'error': f"MemoryError: {str(e)}",
                'memory_before_mb': memory_before
            }
        except Exception as e:
            result = {
                'scenario': scenario_config['description'],
                'grid_size': scenario_config['grid_size'],
                'object_count': scenario_config['object_count'],
                'complexity': scenario_config['grid_size'] * scenario_config['object_count'],
                'success': False,
                'error': f"Error: {str(e)}"
            }
        
        results.append(result)
        
        # Print immediate results
        if result['success']:
            print(f"  [OK] Total Memory: {result['total_memory_mb']:.1f} MB")
            print(f"    - Environment: {result['env_memory_mb']:.1f} MB")
            print(f"    - Calculation: {result['calc_memory_mb']:.1f} MB")
            print(f"    - Per Cell: {result['memory_per_cell']:.3f} MB")
            print(f"    - Per Object: {result['memory_per_object']:.3f} MB")
        else:
            print(f"  [FAIL] FAILED: {result.get('error', 'Unknown error')}")
        
        print()
    
    # Analysis and recommendations
    print("=== MEMORY ANALYSIS RESULTS ===\n")
    
    successful_results = [r for r in results if r['success']]
    failed_results = [r for r in results if not r['success']]
    
    if successful_results:
        print("Successful Scenarios:")
        for result in successful_results:
            print(f"  {result['scenario']}: {result['total_memory_mb']:.1f} MB")
        
        print(f"\nMemory Scaling Analysis:")
        if len(successful_results) >= 2:
            small = successful_results[0]
            large = successful_results[-1]
            
            size_ratio = large['grid_size'] / small['grid_size']
            memory_ratio = large['total_memory_mb'] / small['total_memory_mb']
            
            print(f"  Grid size ratio: {size_ratio:.1f}x")
            print(f"  Memory ratio: {memory_ratio:.1f}x")
            
            if memory_ratio > size_ratio ** 2:
                print(f"  [WARNING] Memory scaling is WORSE than quadratic! (Expected: {size_ratio**2:.1f}x)")
                print(f"     This suggests algorithmic inefficiency, not just grid size scaling.")
            elif memory_ratio > size_ratio:
                print(f"  [OK] Memory scaling is between linear and quadratic (reasonable)")
            else:
                print(f"  [EXCELLENT] Memory scaling is better than linear")
    
    if failed_results:
        print(f"\nFailed Scenarios (Memory Limits):")
        for result in failed_results:
            complexity = result['complexity']
            print(f"  {result['scenario']}: Complexity {complexity}")
        
        if successful_results:
            max_successful_complexity = max(r['complexity'] for r in successful_results)
            print(f"\nRecommended Maximum Complexity: {max_successful_complexity}")
            print(f"Safe grid sizes with 100 objects: {int((max_successful_complexity / 100) ** 0.5)}")
    
    # Generate safe configuration
    print(f"\n=== RECOMMENDED SAFE CONFIGURATION ===")
    
    if successful_results:
        safe_results = [r for r in successful_results if r['total_memory_mb'] < 500]  # Under 500MB
        
        if safe_results:
            max_safe = max(safe_results, key=lambda x: x['complexity'])
            safe_grid_size = max_safe['grid_size']
            safe_objects = max_safe['object_count']
            
            print(f"Maximum safe grid size: {safe_grid_size}")
            print(f"Maximum safe object count: {safe_objects}")
            print(f"Recommended config for comprehensive benchmark:")
            print(f"  grid_sizes = [25, {safe_grid_size}]" + (f", {safe_grid_size + 10}" if safe_grid_size < 40 else ""))
            print(f"  object_counts = [50, {safe_objects}]")
            print(f"  seeds = [31, 42, 123]  # Multiple seeds are fine - they don't affect memory")
            print(f"  iterations = 5  # Safe iteration count")
        else:
            print("All tested scenarios use significant memory. Use smaller configurations.")
    
    tracemalloc.stop()
    return results

if __name__ == "__main__":
    try:
        results = analyze_memory_usage()
    except Exception as e:
        print(f"Analysis failed: {e}")
        import traceback
        traceback.print_exc()