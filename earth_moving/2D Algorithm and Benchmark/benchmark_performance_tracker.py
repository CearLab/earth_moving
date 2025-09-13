"""
Performance Measurement Framework
High-precision timing utilities and memory usage monitoring for benchmark evaluation
"""

import time
import psutil
import gc
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional, Callable
from contextlib import contextmanager
from collections import defaultdict
import tracemalloc
import threading
from functools import wraps

@dataclass
class PerformanceMetrics:
    """Container for performance measurement results"""
    operation_name: str
    start_time: float
    end_time: float
    execution_time: float
    cpu_percent_start: float
    cpu_percent_end: float
    memory_start_mb: float
    memory_end_mb: float
    memory_peak_mb: float
    memory_allocated_mb: float
    gc_collections: Dict[int, int] = field(default_factory=dict)
    custom_metrics: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def memory_delta_mb(self) -> float:
        """Memory usage change during operation"""
        return self.memory_end_mb - self.memory_start_mb
    
    @property
    def execution_time_ms(self) -> float:
        """Execution time in milliseconds"""
        return self.execution_time * 1000

@dataclass
class BenchmarkResult:
    """Complete benchmark result with multiple runs"""
    scenario_id: str
    operation_name: str
    runs: List[PerformanceMetrics]
    parameters: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def average_time(self) -> float:
        """Average execution time across all runs"""
        return sum(run.execution_time for run in self.runs) / len(self.runs)
    
    @property
    def average_time_ms(self) -> float:
        """Average execution time in milliseconds"""
        return self.average_time * 1000
    
    @property
    def min_time(self) -> float:
        """Minimum execution time"""
        return min(run.execution_time for run in self.runs)
    
    @property
    def max_time(self) -> float:
        """Maximum execution time"""
        return max(run.execution_time for run in self.runs)
    
    @property
    def std_deviation(self) -> float:
        """Standard deviation of execution times"""
        avg = self.average_time
        variance = sum((run.execution_time - avg) ** 2 for run in self.runs) / len(self.runs)
        return variance ** 0.5
    
    @property
    def average_memory_delta(self) -> float:
        """Average memory usage change"""
        return sum(run.memory_delta_mb for run in self.runs) / len(self.runs)
    
    @property
    def peak_memory_usage(self) -> float:
        """Peak memory usage across all runs"""
        return max(run.memory_peak_mb for run in self.runs)

class PerformanceTracker:
    """High-precision performance tracking system"""
    
    def __init__(self, enable_memory_profiling=True):
        self.enable_memory_profiling = enable_memory_profiling
        self.results: Dict[str, List[BenchmarkResult]] = defaultdict(list)
        self.process = psutil.Process()
        
        # Memory profiling setup
        if self.enable_memory_profiling:
            tracemalloc.start()
        
        # Performance monitoring thread
        self._monitoring = False
        self._monitor_thread = None
        self._monitor_data = []
    
    @contextmanager
    def measure_operation(self, operation_name: str, **custom_metrics):
        """Context manager for measuring operation performance"""
        # Pre-measurement cleanup
        gc.collect()
        
        # Start measurements
        start_time = time.perf_counter()
        start_cpu = self.process.cpu_percent()
        start_memory = self.process.memory_info().rss / 1024 / 1024  # MB
        start_gc = {i: gc.get_count()[i] for i in range(3)}
        
        # Memory profiling snapshot
        if self.enable_memory_profiling:
            tracemalloc.take_snapshot()
        
        # Start continuous monitoring for peak memory
        peak_memory = start_memory
        memory_allocated = 0
        
        if self.enable_memory_profiling:
            snapshot_start = tracemalloc.take_snapshot()
        
        try:
            # Yield control to measured code
            yield
            
        finally:
            # End measurements
            end_time = time.perf_counter()
            end_cpu = self.process.cpu_percent()
            end_memory = self.process.memory_info().rss / 1024 / 1024  # MB
            end_gc = {i: gc.get_count()[i] for i in range(3)}
            
            # Calculate peak memory and allocations
            current_memory = self.process.memory_info().rss / 1024 / 1024
            peak_memory = max(peak_memory, current_memory)
            
            if self.enable_memory_profiling:
                snapshot_end = tracemalloc.take_snapshot()
                top_stats = snapshot_end.compare_to(snapshot_start, 'lineno')
                memory_allocated = sum(stat.size_diff for stat in top_stats if stat.size_diff > 0) / 1024 / 1024
            
            # Create performance metrics
            metrics = PerformanceMetrics(
                operation_name=operation_name,
                start_time=start_time,
                end_time=end_time,
                execution_time=end_time - start_time,
                cpu_percent_start=start_cpu,
                cpu_percent_end=end_cpu,
                memory_start_mb=start_memory,
                memory_end_mb=end_memory,
                memory_peak_mb=peak_memory,
                memory_allocated_mb=memory_allocated,
                gc_collections={i: end_gc[i] - start_gc[i] for i in range(3)},
                custom_metrics=custom_metrics
            )
            
            # Store the metrics for later retrieval
            if not hasattr(self, '_current_metrics'):
                self._current_metrics = []
            self._current_metrics.append(metrics)
            
            return metrics
    
    def measure_function(self, operation_name: str = None):
        """Decorator for measuring function performance"""
        def decorator(func: Callable):
            @wraps(func)
            def wrapper(*args, **kwargs):
                name = operation_name or f"{func.__module__}.{func.__name__}"
                with self.measure_operation(name):
                    return func(*args, **kwargs)
            return wrapper
        return decorator
    
    def run_benchmark(self, operation_name: str, operation_func: Callable, 
                     iterations: int, scenario_id: str = None, **parameters) -> BenchmarkResult:
        """Run a complete benchmark with multiple iterations"""
        runs = []
        
        for i in range(iterations):
            # Run the operation
            with self.measure_operation(f"{operation_name}_iter_{i}"):
                try:
                    result = operation_func()
                    # Store custom result metrics if returned
                    if isinstance(result, dict) and hasattr(self, '_current_metrics') and self._current_metrics:
                        self._current_metrics[-1].custom_metrics.update(result)
                except Exception as e:
                    if hasattr(self, '_current_metrics') and self._current_metrics:
                        self._current_metrics[-1].custom_metrics['error'] = str(e)
                    raise
            
            # Add the metrics from this run
            if hasattr(self, '_current_metrics') and self._current_metrics:
                runs.extend(self._current_metrics)
                self._current_metrics = []  # Reset for next iteration
        
        # Create benchmark result
        benchmark_result = BenchmarkResult(
            scenario_id=scenario_id or f"benchmark_{len(self.results[operation_name])}",
            operation_name=operation_name,
            runs=runs,
            parameters=parameters
        )
        
        # Store result
        self.results[operation_name].append(benchmark_result)
        
        return benchmark_result
    
    def start_continuous_monitoring(self, interval: float = 0.1):
        """Start continuous system monitoring"""
        if self._monitoring:
            return
        
        self._monitoring = True
        self._monitor_data = []
        
        def monitor():
            while self._monitoring:
                try:
                    cpu_percent = self.process.cpu_percent()
                    memory_mb = self.process.memory_info().rss / 1024 / 1024
                    timestamp = time.perf_counter()
                    
                    self._monitor_data.append({
                        'timestamp': timestamp,
                        'cpu_percent': cpu_percent,
                        'memory_mb': memory_mb
                    })
                    
                    time.sleep(interval)
                except Exception:
                    break
        
        self._monitor_thread = threading.Thread(target=monitor, daemon=True)
        self._monitor_thread.start()
    
    def stop_continuous_monitoring(self) -> List[Dict]:
        """Stop continuous monitoring and return collected data"""
        self._monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1.0)
        
        data = self._monitor_data.copy()
        self._monitor_data.clear()
        return data
    
    def get_system_info(self) -> Dict[str, Any]:
        """Get comprehensive system information"""
        return {
            'python_version': sys.version,
            'cpu_count': psutil.cpu_count(),
            'cpu_freq': psutil.cpu_freq()._asdict() if psutil.cpu_freq() else None,
            'memory_total_gb': psutil.virtual_memory().total / 1024 / 1024 / 1024,
            'memory_available_gb': psutil.virtual_memory().available / 1024 / 1024 / 1024,
            'platform': sys.platform,
            'process_id': self.process.pid,
            'memory_profiling_enabled': self.enable_memory_profiling
        }
    
    def print_benchmark_summary(self, operation_name: str = None):
        """Print summary of benchmark results"""
        if operation_name:
            results = self.results.get(operation_name, [])
            operations = [operation_name]
        else:
            results = []
            operations = list(self.results.keys())
            for op in operations:
                results.extend(self.results[op])
        
        print("=== BENCHMARK PERFORMANCE SUMMARY ===")
        print(f"Total Operations: {len(operations)}")
        print(f"Total Benchmark Results: {len(results)}")
        print()
        
        for operation in operations:
            op_results = self.results[operation]
            if not op_results:
                continue
            
            print(f"Operation: {operation}")
            print(f"  Benchmark Runs: {len(op_results)}")
            
            # Calculate overall statistics
            all_times = []
            all_memory_deltas = []
            
            for result in op_results:
                all_times.extend([run.execution_time for run in result.runs])
                all_memory_deltas.extend([run.memory_delta_mb for run in result.runs])
            
            if all_times:
                avg_time = sum(all_times) / len(all_times)
                min_time = min(all_times)
                max_time = max(all_times)
                
                print(f"  Average Time: {avg_time*1000:.2f}ms")
                print(f"  Time Range: {min_time*1000:.2f}ms - {max_time*1000:.2f}ms")
            
            if all_memory_deltas:
                avg_memory = sum(all_memory_deltas) / len(all_memory_deltas)
                print(f"  Average Memory Delta: {avg_memory:.2f}MB")
            
            print()
    
    def export_results(self, filename: str = None) -> Dict:
        """Export all results to dictionary format"""
        export_data = {
            'system_info': self.get_system_info(),
            'timestamp': time.time(),
            'results': {}
        }
        
        for operation_name, results in self.results.items():
            export_data['results'][operation_name] = []
            
            for result in results:
                result_data = {
                    'scenario_id': result.scenario_id,
                    'operation_name': result.operation_name,
                    'parameters': result.parameters,
                    'statistics': {
                        'average_time': result.average_time,
                        'average_time_ms': result.average_time_ms,
                        'min_time': result.min_time,
                        'max_time': result.max_time,
                        'std_deviation': result.std_deviation,
                        'average_memory_delta': result.average_memory_delta,
                        'peak_memory_usage': result.peak_memory_usage
                    },
                    'runs': []
                }
                
                for run in result.runs:
                    run_data = {
                        'operation_name': run.operation_name,
                        'execution_time': run.execution_time,
                        'execution_time_ms': run.execution_time_ms,
                        'memory_delta_mb': run.memory_delta_mb,
                        'memory_peak_mb': run.memory_peak_mb,
                        'cpu_percent_start': run.cpu_percent_start,
                        'cpu_percent_end': run.cpu_percent_end,
                        'gc_collections': run.gc_collections,
                        'custom_metrics': run.custom_metrics
                    }
                    result_data['runs'].append(run_data)
                
                export_data['results'][operation_name].append(result_data)
        
        if filename:
            import json
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            print(f"Results exported to: {filename}")
        
        return export_data
    
    def clear_results(self):
        """Clear all stored results"""
        self.results.clear()
    
    def memory_profile_operation(self, operation_func: Callable, top_k: int = 10):
        """Detailed memory profiling of an operation"""
        if not self.enable_memory_profiling:
            print("Memory profiling not enabled")
            return None
        
        # Take initial snapshot
        snapshot_start = tracemalloc.take_snapshot()
        
        # Run operation
        result = operation_func()
        
        # Take final snapshot
        snapshot_end = tracemalloc.take_snapshot()
        
        # Analyze differences
        top_stats = snapshot_end.compare_to(snapshot_start, 'lineno')
        
        print(f"=== MEMORY PROFILE (Top {top_k}) ===")
        for index, stat in enumerate(top_stats[:top_k], 1):
            print(f"{index:2d}. {stat}")
        
        return {
            'result': result,
            'memory_stats': top_stats,
            'total_allocated': sum(stat.size_diff for stat in top_stats if stat.size_diff > 0),
            'total_freed': sum(stat.size_diff for stat in top_stats if stat.size_diff < 0)
        }

# Example usage and testing
if __name__ == "__main__":
    tracker = PerformanceTracker()
    
    # Example: Measure a simple operation
    def example_operation():
        """Example operation to measure"""
        import random
        data = [random.random() for _ in range(100000)]
        return sum(data)
    
    # Run benchmark
    result = tracker.run_benchmark(
        operation_name="example_sum",
        operation_func=example_operation,
        iterations=5,
        scenario_id="test_scenario"
    )
    
    # Print results
    tracker.print_benchmark_summary()
    
    # Export results
    export_data = tracker.export_results()
    print(f"Export contains {len(export_data['results'])} operation types")