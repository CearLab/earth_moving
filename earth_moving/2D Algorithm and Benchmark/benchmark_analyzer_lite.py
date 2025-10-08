"""
Lightweight Statistical Analysis Engine for Benchmark Results
Uses only standard library and numpy for core analysis
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional, Union
from dataclasses import dataclass, field
from collections import defaultdict
from datetime import datetime
import warnings

# Only use matplotlib if available
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Suppress warnings for cleaner output
warnings.filterwarnings('ignore')

@dataclass
class StatisticalSummary:
    """Statistical summary of benchmark results"""
    mean: float
    median: float
    std: float
    min: float
    max: float
    percentile_25: float
    percentile_75: float
    count: int
    confidence_interval_95: Tuple[float, float] = None
    
    @property
    def coefficient_of_variation(self) -> float:
        """Coefficient of variation (std/mean)"""
        return self.std / self.mean if self.mean > 0 else float('inf')
    
    @property
    def range_value(self) -> float:
        """Range (max - min)"""
        return self.max - self.min

@dataclass
class ComparisonResult:
    """Result of statistical comparison between two datasets"""
    group1_name: str
    group2_name: str
    group1_stats: StatisticalSummary
    group2_stats: StatisticalSummary
    t_statistic: float
    p_value: float
    is_significant: bool
    effect_size: float  # Cohen's d
    performance_improvement: float  # Percentage improvement
    
    @property
    def interpretation(self) -> str:
        """Human-readable interpretation of the comparison"""
        if not self.is_significant:
            return "No significant difference"
        
        better_group = self.group1_name if self.group1_stats.mean < self.group2_stats.mean else self.group2_name
        improvement = abs(self.performance_improvement)
        
        if improvement < 5:
            magnitude = "small"
        elif improvement < 20:
            magnitude = "moderate"
        else:
            magnitude = "large"
        
        return f"{better_group} shows {magnitude} improvement ({improvement:.1f}%)"

class BenchmarkAnalyzerLite:
    """Lightweight statistical analysis engine for benchmark results"""
    
    def __init__(self, output_directory: str = "analysis_results"):
        self.output_dir = Path(output_directory)
        self.output_dir.mkdir(exist_ok=True)
        
        self.loaded_results: Dict[str, Any] = {}
        self.analysis_cache: Dict[str, Any] = {}
    
    def load_benchmark_results(self, results_file: Union[str, Path]) -> Dict[str, Any]:
        """Load benchmark results from JSON file"""
        results_path = Path(results_file)
        
        if not results_path.exists():
            raise FileNotFoundError(f"Results file not found: {results_path}")
        
        with open(results_path, 'r') as f:
            results = json.load(f)
        
        # Cache results with filename as key
        cache_key = results_path.stem
        self.loaded_results[cache_key] = results
        
        print(f"Loaded benchmark results: {cache_key}")
        print(f"  System: {results.get('system_info', {}).get('platform', 'Unknown')}")
        print(f"  Timestamp: {datetime.fromtimestamp(results.get('timestamp', 0))}")
        print(f"  Operations: {len(results.get('results', {}))}")
        
        return results
    
    def extract_performance_data(self, results: Dict[str, Any], 
                                operation_filter: str = None) -> List[Dict[str, Any]]:
        """Extract performance data into list of dictionaries for analysis"""
        data_rows = []
        
        results_data = results.get('results', {})
        
        for operation_name, operation_results in results_data.items():
            # Apply operation filter if specified
            if operation_filter and operation_filter not in operation_name:
                continue
            
            for benchmark_result in operation_results:
                scenario_id = benchmark_result.get('scenario_id', 'unknown')
                parameters = benchmark_result.get('parameters', {})
                
                # Extract run-level data
                for run in benchmark_result.get('runs', []):
                    row = {
                        'operation_name': operation_name,
                        'scenario_id': scenario_id,
                        'execution_time_ms': run.get('execution_time_ms', 0),
                        'memory_delta_mb': run.get('memory_delta_mb', 0),
                        'memory_peak_mb': run.get('memory_peak_mb', 0),
                        'cpu_percent_start': run.get('cpu_percent_start', 0),
                        'cpu_percent_end': run.get('cpu_percent_end', 0),
                        # Parameters
                        'grid_size': parameters.get('grid_size', 0),
                        'object_count': parameters.get('object_count', 0),
                        'use_spillage': parameters.get('use_spillage', False),
                        'strategy': parameters.get('strategy', 'unknown'),
                        'spillage_mode': parameters.get('spillage_mode', None),
                        # Derived metrics
                        'objects_per_cell': parameters.get('object_count', 0) / (parameters.get('grid_size', 1) ** 2),
                        'cpu_delta': run.get('cpu_percent_end', 0) - run.get('cpu_percent_start', 0)
                    }
                    
                    # Add custom metrics if available
                    custom_metrics = run.get('custom_metrics', {})
                    for key, value in custom_metrics.items():
                        if isinstance(value, (int, float)):
                            row[f'custom_{key}'] = value
                    
                    data_rows.append(row)
        
        # Add derived columns
        for row in data_rows:
            row['total_cells'] = row['grid_size'] ** 2
            row['object_density'] = row['object_count'] / row['total_cells'] if row['total_cells'] > 0 else 0
            row['time_per_object'] = row['execution_time_ms'] / row['object_count'] if row['object_count'] > 0 else 0
            row['time_per_cell'] = row['execution_time_ms'] / row['total_cells'] if row['total_cells'] > 0 else 0
        
        return data_rows
    
    def calculate_statistical_summary(self, data: np.ndarray) -> StatisticalSummary:
        """Calculate comprehensive statistical summary"""
        if len(data) == 0:
            return StatisticalSummary(0, 0, 0, 0, 0, 0, 0, 0)
        
        mean_val = np.mean(data)
        std_val = np.std(data, ddof=1)
        
        # Calculate confidence interval using normal approximation
        confidence_interval = None
        if len(data) > 1:
            se = std_val / np.sqrt(len(data))
            # Use 1.96 for 95% confidence (normal approximation)
            margin = 1.96 * se
            confidence_interval = (mean_val - margin, mean_val + margin)
        
        return StatisticalSummary(
            mean=mean_val,
            median=np.median(data),
            std=std_val,
            min=np.min(data),
            max=np.max(data),
            percentile_25=np.percentile(data, 25),
            percentile_75=np.percentile(data, 75),
            count=len(data),
            confidence_interval_95=confidence_interval
        )
    
    def simple_t_test(self, group1: np.ndarray, group2: np.ndarray) -> Tuple[float, float]:
        """Simple two-sample t-test implementation"""
        n1, n2 = len(group1), len(group2)
        mean1, mean2 = np.mean(group1), np.mean(group2)
        var1, var2 = np.var(group1, ddof=1), np.var(group2, ddof=1)
        
        # Welch's t-test (unequal variances)
        pooled_se = np.sqrt(var1/n1 + var2/n2)
        t_stat = (mean1 - mean2) / pooled_se
        
        # Degrees of freedom (Welch-Satterthwaite equation)
        df = (var1/n1 + var2/n2)**2 / ((var1/n1)**2/(n1-1) + (var2/n2)**2/(n2-1))
        
        # Simple p-value approximation (this is very rough)
        # For proper p-values, you'd need scipy.stats
        abs_t = abs(t_stat)
        if abs_t > 2.58:  # 99% confidence
            p_value = 0.01
        elif abs_t > 1.96:  # 95% confidence
            p_value = 0.05
        elif abs_t > 1.28:  # 80% confidence
            p_value = 0.20
        else:
            p_value = 0.50
        
        return t_stat, p_value
    
    def compare_groups(self, group1_data: np.ndarray, group2_data: np.ndarray,
                      group1_name: str, group2_name: str,
                      alpha: float = 0.05) -> ComparisonResult:
        """Perform statistical comparison between two groups"""
        stats1 = self.calculate_statistical_summary(group1_data)
        stats2 = self.calculate_statistical_summary(group2_data)
        
        # Perform t-test
        t_stat, p_value = self.simple_t_test(group1_data, group2_data)
        is_significant = p_value < alpha
        
        # Calculate effect size (Cohen's d)
        pooled_std = np.sqrt(((len(group1_data) - 1) * stats1.std ** 2 + 
                             (len(group2_data) - 1) * stats2.std ** 2) / 
                            (len(group1_data) + len(group2_data) - 2))
        
        effect_size = abs(stats1.mean - stats2.mean) / pooled_std if pooled_std > 0 else 0
        
        # Calculate performance improvement percentage
        if stats1.mean > 0 and stats2.mean > 0:
            performance_improvement = ((stats1.mean - stats2.mean) / stats1.mean) * 100
        else:
            performance_improvement = 0
        
        return ComparisonResult(
            group1_name=group1_name,
            group2_name=group2_name,
            group1_stats=stats1,
            group2_stats=stats2,
            t_statistic=t_stat,
            p_value=p_value,
            is_significant=is_significant,
            effect_size=effect_size,
            performance_improvement=performance_improvement
        )
    
    def analyze_scalability(self, data_rows: List[Dict[str, Any]], 
                          performance_metric: str = 'execution_time_ms',
                          scale_factor: str = 'object_count') -> Dict[str, Any]:
        """Analyze performance scalability with respect to problem size"""
        if not data_rows:
            return {}
        
        # Group by scale factor and calculate statistics
        scale_groups = defaultdict(list)
        
        for row in data_rows:
            if scale_factor in row and performance_metric in row:
                scale_groups[row[scale_factor]].append(row[performance_metric])
        
        scalability_stats = []
        for scale_value in sorted(scale_groups.keys()):
            data_array = np.array(scale_groups[scale_value])
            stats = self.calculate_statistical_summary(data_array)
            
            scalability_stats.append({
                scale_factor: scale_value,
                'mean_performance': stats.mean,
                'std_performance': stats.std,
                'count': stats.count,
                'min_performance': stats.min,
                'max_performance': stats.max
            })
        
        # Calculate scaling relationship (linear fit)
        scaling_analysis = {}
        if len(scalability_stats) > 1:
            x_vals = [stat[scale_factor] for stat in scalability_stats]
            y_vals = [stat['mean_performance'] for stat in scalability_stats]
            
            # Simple linear regression
            x = np.array(x_vals)
            y = np.array(y_vals)
            
            # Calculate slope and intercept
            n = len(x)
            sum_x = np.sum(x)
            sum_y = np.sum(y)
            sum_xy = np.sum(x * y)
            sum_x2 = np.sum(x * x)
            
            slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)
            intercept = (sum_y - slope * sum_x) / n
            
            # Calculate R-squared
            y_pred = slope * x + intercept
            ss_res = np.sum((y - y_pred) ** 2)
            ss_tot = np.sum((y - np.mean(y)) ** 2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
            
            scaling_analysis = {
                'linear_slope': slope,
                'linear_intercept': intercept,
                'linear_r_squared': r_squared,
                'complexity_estimate': self._estimate_complexity(slope, x_vals, y_vals)
            }
        
        return {
            'scalability_data': scalability_stats,
            'scaling_analysis': scaling_analysis,
            'scale_factor': scale_factor,
            'performance_metric': performance_metric
        }
    
    def _estimate_complexity(self, slope: float, x_vals: List[float], y_vals: List[float]) -> str:
        """Estimate algorithmic complexity based on scaling behavior"""
        if len(x_vals) < 2:
            return "insufficient_data"
        
        # Simple complexity estimation based on slope and data pattern
        if abs(slope) < 0.1:
            return "O(1) - Constant"
        
        # Check if quadratic fit is much better than linear
        x = np.array(x_vals)
        y = np.array(y_vals)
        
        # Linear fit error
        y_linear = slope * x + (np.mean(y) - slope * np.mean(x))
        linear_error = np.mean((y - y_linear) ** 2)
        
        # Try quadratic pattern detection
        # If performance grows much faster than linear, suggest quadratic
        growth_ratio = y[-1] / y[0] if y[0] > 0 else 1
        size_ratio = x[-1] / x[0] if x[0] > 0 else 1
        
        if growth_ratio > size_ratio ** 1.5:
            return "O(n²) - Quadratic (approximate)"
        elif growth_ratio > size_ratio:
            return "O(n) - Linear"
        else:
            return "O(log n) - Logarithmic (approximate)"
    
    def analyze_spillage_impact(self, data_rows: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Analyze performance impact of spillage mode"""
        spillage_true = []
        spillage_false = []
        
        for row in data_rows:
            if 'use_spillage' in row and 'execution_time_ms' in row:
                if row['use_spillage']:
                    spillage_true.append(row['execution_time_ms'])
                else:
                    spillage_false.append(row['execution_time_ms'])
        
        if len(spillage_true) == 0 or len(spillage_false) == 0:
            return {'error': 'Insufficient data for spillage comparison'}
        
        comparison = self.compare_groups(
            np.array(spillage_true), np.array(spillage_false),
            'With Spillage', 'Without Spillage'
        )
        
        # Calculate overhead
        spillage_overhead = ((comparison.group1_stats.mean - comparison.group2_stats.mean) / 
                           comparison.group2_stats.mean) * 100
        
        return {
            'comparison': comparison,
            'spillage_overhead_percent': spillage_overhead,
            'spillage_samples': len(spillage_true),
            'no_spillage_samples': len(spillage_false),
            'recommendation': self._get_spillage_recommendation(spillage_overhead, comparison.is_significant)
        }
    
    def _get_spillage_recommendation(self, overhead: float, is_significant: bool) -> str:
        """Generate spillage usage recommendation"""
        if not is_significant:
            return "No significant performance difference - use spillage for accuracy"
        
        if overhead < 10:
            return "Low overhead - spillage recommended for better accuracy"
        elif overhead < 25:
            return "Moderate overhead - consider spillage for complex scenarios"
        else:
            return "High overhead - consider disabling spillage for performance"
    
    def print_analysis_summary(self, results_key: str):
        """Print comprehensive analysis summary"""
        if results_key not in self.loaded_results:
            print(f"Results not found: {results_key}")
            return
        
        results = self.loaded_results[results_key]
        data_rows = self.extract_performance_data(results)
        
        if not data_rows:
            print("No performance data available for analysis")
            return
        
        print("=" * 80)
        print(f"PERFORMANCE ANALYSIS SUMMARY - {results_key}")
        print("=" * 80)
        
        # Basic statistics
        print(f"Total Samples: {len(data_rows)}")
        unique_scenarios = set(row['scenario_id'] for row in data_rows)
        unique_operations = set(row['operation_name'] for row in data_rows)
        print(f"Unique Scenarios: {len(unique_scenarios)}")
        print(f"Operations Tested: {', '.join(unique_operations)}")
        print()
        
        # Overall performance
        exec_times = np.array([row['execution_time_ms'] for row in data_rows])
        mem_deltas = np.array([row['memory_delta_mb'] for row in data_rows])
        
        exec_stats = self.calculate_statistical_summary(exec_times)
        mem_stats = self.calculate_statistical_summary(mem_deltas)
        
        print("OVERALL PERFORMANCE:")
        print(f"  Execution Time: {exec_stats.mean:.2f}ms ± {exec_stats.std:.2f}ms")
        print(f"  Range: {exec_stats.min:.2f}ms - {exec_stats.max:.2f}ms")
        print(f"  Memory Usage: {mem_stats.mean:.2f}MB ± {mem_stats.std:.2f}MB")
        print()
        
        # Scalability insights
        scalability = self.analyze_scalability(data_rows, scale_factor='object_count')
        if 'scaling_analysis' in scalability:
            scaling = scalability['scaling_analysis']
            print(f"SCALABILITY (Object Count):")
            print(f"  Linear R²: {scaling.get('linear_r_squared', 0):.3f}")
            print(f"  Complexity: {scaling.get('complexity_estimate', 'Unknown')}")
            print()
        
        # Spillage analysis
        spillage_analysis = self.analyze_spillage_impact(data_rows)
        if 'spillage_overhead_percent' in spillage_analysis:
            print(f"SPILLAGE IMPACT:")
            print(f"  Overhead: {spillage_analysis['spillage_overhead_percent']:.1f}%")
            print(f"  Recommendation: {spillage_analysis['recommendation']}")
            print()
        
        # Operation comparison
        operation_groups = defaultdict(list)
        for row in data_rows:
            operation_groups[row['operation_name']].append(row['execution_time_ms'])
        
        if len(operation_groups) > 1:
            print("OPERATION COMPARISON:")
            for operation, times in operation_groups.items():
                stats = self.calculate_statistical_summary(np.array(times))
                print(f"  {operation}: {stats.mean:.2f}ms ± {stats.std:.2f}ms")
            print()

# Example usage
if __name__ == "__main__":
    analyzer = BenchmarkAnalyzerLite()
    
    # Example: Load and analyze results (if available)
    try:
        results_file = "benchmark_results/benchmark_detailed_quick_20250107_120000.json"
        
        if Path(results_file).exists():
            analyzer.load_benchmark_results(results_file)
            analyzer.print_analysis_summary("benchmark_detailed_quick_20250107_120000")
        else:
            print("No benchmark results found. Run benchmark_runner.py first.")
            print("This lite analyzer provides:")
            print("- Statistical summaries without external dependencies")
            print("- Scalability analysis with complexity estimation") 
            print("- Spillage impact analysis")
            print("- Performance comparisons")
            
    except Exception as e:
        print(f"Analysis example failed: {e}")
        print("This is expected if no benchmark results are available yet.")