"""
Statistical Analysis Engine for Benchmark Results
Comprehensive analysis, comparison, and reporting of performance benchmarks
"""

import json
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional, Union
from dataclasses import dataclass, field
from collections import defaultdict
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from datetime import datetime
import warnings

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

class BenchmarkAnalyzer:
    """Comprehensive statistical analysis engine for benchmark results"""
    
    def __init__(self, output_directory: str = "analysis_results"):
        self.output_dir = Path(output_directory)
        self.output_dir.mkdir(exist_ok=True)
        
        self.loaded_results: Dict[str, Any] = {}
        self.analysis_cache: Dict[str, Any] = {}
        
        # Configure plotting style
        plt.style.use('seaborn-v0_8-whitegrid')
        sns.set_palette("husl")
    
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
                                operation_filter: str = None) -> pd.DataFrame:
        """Extract performance data into pandas DataFrame for analysis"""
        data_rows = []
        
        results_data = results.get('results', {})
        
        for operation_name, operation_results in results_data.items():
            # Apply operation filter if specified
            if operation_filter and operation_filter not in operation_name:
                continue
            
            for benchmark_result in operation_results:
                scenario_id = benchmark_result.get('scenario_id', 'unknown')
                parameters = benchmark_result.get('parameters', {})
                statistics = benchmark_result.get('statistics', {})
                
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
        
        df = pd.DataFrame(data_rows)
        
        if not df.empty:
            # Add derived columns
            df['total_cells'] = df['grid_size'] ** 2
            df['object_density'] = df['object_count'] / df['total_cells']
            df['time_per_object'] = df['execution_time_ms'] / df['object_count']
            df['time_per_cell'] = df['execution_time_ms'] / df['total_cells']
        
        return df
    
    def calculate_statistical_summary(self, data: np.ndarray) -> StatisticalSummary:
        """Calculate comprehensive statistical summary"""
        if len(data) == 0:
            return StatisticalSummary(0, 0, 0, 0, 0, 0, 0, 0)
        
        mean_val = np.mean(data)
        std_val = np.std(data, ddof=1)
        
        # Calculate confidence interval
        confidence_interval = None
        if len(data) > 1:
            se = std_val / np.sqrt(len(data))
            t_val = stats.t.ppf(0.975, len(data) - 1)  # 95% confidence
            margin = t_val * se
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
    
    def compare_groups(self, group1_data: np.ndarray, group2_data: np.ndarray,
                      group1_name: str, group2_name: str,
                      alpha: float = 0.05) -> ComparisonResult:
        """Perform statistical comparison between two groups"""
        stats1 = self.calculate_statistical_summary(group1_data)
        stats2 = self.calculate_statistical_summary(group2_data)
        
        # Perform t-test
        t_stat, p_value = stats.ttest_ind(group1_data, group2_data, equal_var=False)
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
    
    def analyze_scalability(self, df: pd.DataFrame, 
                          performance_metric: str = 'execution_time_ms',
                          scale_factor: str = 'object_count') -> Dict[str, Any]:
        """Analyze performance scalability with respect to problem size"""
        if df.empty:
            return {}
        
        # Group by scale factor and calculate statistics
        scalability_stats = []
        
        for scale_value in sorted(df[scale_factor].unique()):
            subset = df[df[scale_factor] == scale_value]
            if len(subset) == 0:
                continue
            
            summary = self.calculate_statistical_summary(subset[performance_metric].values)
            
            scalability_stats.append({
                scale_factor: scale_value,
                'mean_performance': summary.mean,
                'std_performance': summary.std,
                'count': summary.count,
                'min_performance': summary.min,
                'max_performance': summary.max
            })
        
        scalability_df = pd.DataFrame(scalability_stats)
        
        # Calculate scaling relationship (linear regression)
        scaling_analysis = {}
        if len(scalability_df) > 1:
            x = scalability_df[scale_factor].values
            y = scalability_df['mean_performance'].values
            
            # Linear fit
            linear_slope, linear_intercept, linear_r, linear_p, linear_se = stats.linregress(x, y)
            
            # Polynomial fit (degree 2)
            poly_coeffs = np.polyfit(x, y, 2)
            poly_r2 = 1 - (np.sum((y - np.polyval(poly_coeffs, x)) ** 2) / 
                          np.sum((y - np.mean(y)) ** 2))
            
            scaling_analysis = {
                'linear_slope': linear_slope,
                'linear_intercept': linear_intercept,
                'linear_r_squared': linear_r ** 2,
                'linear_p_value': linear_p,
                'polynomial_coefficients': poly_coeffs.tolist(),
                'polynomial_r_squared': poly_r2,
                'complexity_estimate': self._estimate_complexity(linear_slope, poly_coeffs)
            }
        
        return {
            'scalability_data': scalability_df.to_dict('records'),
            'scaling_analysis': scaling_analysis,
            'scale_factor': scale_factor,
            'performance_metric': performance_metric
        }
    
    def _estimate_complexity(self, linear_slope: float, poly_coeffs: np.ndarray) -> str:
        """Estimate algorithmic complexity based on scaling behavior"""
        if len(poly_coeffs) < 3:
            return "insufficient_data"
        
        a, b, c = poly_coeffs
        
        # Analyze the polynomial fit
        if abs(a) < 1e-6:  # Essentially linear
            if abs(linear_slope) < 0.1:
                return "O(1) - Constant"
            else:
                return "O(n) - Linear"
        elif a > 0:  # Positive quadratic term
            if abs(b) < 1e-3:
                return "O(n²) - Quadratic"
            else:
                return "O(n²) - Quadratic with linear component"
        else:
            return "Complex/Decreasing - Unusual behavior"
    
    def analyze_spillage_impact(self, df: pd.DataFrame) -> Dict[str, Any]:
        """Analyze performance impact of spillage mode"""
        if df.empty or 'use_spillage' not in df.columns:
            return {'error': 'No spillage data available'}
        
        spillage_data = df[df['use_spillage'] == True]['execution_time_ms'].values
        no_spillage_data = df[df['use_spillage'] == False]['execution_time_ms'].values
        
        if len(spillage_data) == 0 or len(no_spillage_data) == 0:
            return {'error': 'Insufficient data for spillage comparison'}
        
        comparison = self.compare_groups(
            spillage_data, no_spillage_data,
            'With Spillage', 'Without Spillage'
        )
        
        # Calculate overhead
        spillage_overhead = ((comparison.group1_stats.mean - comparison.group2_stats.mean) / 
                           comparison.group2_stats.mean) * 100
        
        return {
            'comparison': comparison,
            'spillage_overhead_percent': spillage_overhead,
            'spillage_samples': len(spillage_data),
            'no_spillage_samples': len(no_spillage_data),
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
    
    def generate_performance_report(self, results_key: str, 
                                  output_filename: str = None) -> Dict[str, Any]:
        """Generate comprehensive performance analysis report"""
        if results_key not in self.loaded_results:
            raise ValueError(f"Results not found: {results_key}")
        
        results = self.loaded_results[results_key]
        df = self.extract_performance_data(results)
        
        if df.empty:
            return {'error': 'No performance data extracted'}
        
        report = {
            'metadata': {
                'results_key': results_key,
                'generated_at': datetime.now().isoformat(),
                'total_samples': len(df),
                'unique_scenarios': df['scenario_id'].nunique(),
                'operations': df['operation_name'].unique().tolist()
            }
        }
        
        # Overall performance statistics
        report['overall_performance'] = {
            'execution_time': self.calculate_statistical_summary(df['execution_time_ms'].values).__dict__,
            'memory_usage': self.calculate_statistical_summary(df['memory_delta_mb'].values).__dict__,
            'peak_memory': self.calculate_statistical_summary(df['memory_peak_mb'].values).__dict__
        }
        
        # Scalability analysis
        report['scalability_analysis'] = {}
        for scale_factor in ['grid_size', 'object_count', 'total_cells']:
            if scale_factor in df.columns:
                report['scalability_analysis'][scale_factor] = self.analyze_scalability(
                    df, scale_factor=scale_factor
                )
        
        # Spillage impact analysis
        report['spillage_analysis'] = self.analyze_spillage_impact(df)
        
        # Operation comparison
        report['operation_comparison'] = self._analyze_operation_performance(df)
        
        # Performance outliers
        report['outlier_analysis'] = self._identify_performance_outliers(df)
        
        # Save report
        if output_filename:
            report_path = self.output_dir / output_filename
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            report_path = self.output_dir / f"performance_report_{results_key}_{timestamp}.json"
        
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        print(f"Performance report saved to: {report_path}")
        return report
    
    def _analyze_operation_performance(self, df: pd.DataFrame) -> Dict[str, Any]:
        """Analyze and compare performance across different operations"""
        if 'operation_name' not in df.columns:
            return {}
        
        operations = df['operation_name'].unique()
        operation_stats = {}
        
        for operation in operations:
            op_data = df[df['operation_name'] == operation]['execution_time_ms'].values
            operation_stats[operation] = self.calculate_statistical_summary(op_data).__dict__
        
        # Find best and worst performing operations
        if len(operation_stats) > 1:
            best_operation = min(operation_stats.keys(), 
                               key=lambda x: operation_stats[x]['mean'])
            worst_operation = max(operation_stats.keys(), 
                                key=lambda x: operation_stats[x]['mean'])
            
            return {
                'operation_statistics': operation_stats,
                'best_performing': best_operation,
                'worst_performing': worst_operation,
                'performance_ratio': (operation_stats[worst_operation]['mean'] / 
                                    operation_stats[best_operation]['mean'])
            }
        
        return {'operation_statistics': operation_stats}
    
    def _identify_performance_outliers(self, df: pd.DataFrame, 
                                     z_threshold: float = 3.0) -> Dict[str, Any]:
        """Identify performance outliers using statistical methods"""
        outliers_info = {}
        
        for metric in ['execution_time_ms', 'memory_delta_mb']:
            if metric not in df.columns:
                continue
            
            values = df[metric].values
            z_scores = np.abs(stats.zscore(values))
            outlier_indices = np.where(z_scores > z_threshold)[0]
            
            if len(outlier_indices) > 0:
                outliers_info[metric] = {
                    'count': len(outlier_indices),
                    'percentage': (len(outlier_indices) / len(values)) * 100,
                    'outlier_values': values[outlier_indices].tolist(),
                    'outlier_scenarios': df.iloc[outlier_indices]['scenario_id'].tolist()
                }
        
        return outliers_info
    
    def create_performance_visualizations(self, results_key: str, 
                                        save_plots: bool = True) -> Dict[str, str]:
        """Create comprehensive performance visualizations"""
        if results_key not in self.loaded_results:
            raise ValueError(f"Results not found: {results_key}")
        
        results = self.loaded_results[results_key]
        df = self.extract_performance_data(results)
        
        if df.empty:
            return {'error': 'No data for visualization'}
        
        plot_files = {}
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 1. Scalability plots
        if 'object_count' in df.columns and 'grid_size' in df.columns:
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            # Execution time vs object count
            sns.scatterplot(data=df, x='object_count', y='execution_time_ms', 
                           hue='grid_size', ax=axes[0,0])
            axes[0,0].set_title('Execution Time vs Object Count')
            
            # Memory usage vs grid size
            sns.boxplot(data=df, x='grid_size', y='memory_delta_mb', ax=axes[0,1])
            axes[0,1].set_title('Memory Usage by Grid Size')
            
            # Object density impact
            sns.scatterplot(data=df, x='object_density', y='execution_time_ms', 
                           hue='use_spillage', ax=axes[1,0])
            axes[1,0].set_title('Performance vs Object Density')
            
            # Time per object scaling
            sns.lineplot(data=df, x='object_count', y='time_per_object', 
                        hue='grid_size', ax=axes[1,1])
            axes[1,1].set_title('Time per Object Scaling')
            
            plt.tight_layout()
            
            if save_plots:
                scalability_file = self.output_dir / f"scalability_analysis_{results_key}_{timestamp}.png"
                plt.savefig(scalability_file, dpi=300, bbox_inches='tight')
                plot_files['scalability'] = str(scalability_file)
            
            plt.show()
        
        # 2. Performance distribution plots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Execution time distribution
        df['execution_time_ms'].hist(bins=50, ax=axes[0,0])
        axes[0,0].set_title('Execution Time Distribution')
        axes[0,0].set_xlabel('Time (ms)')
        
        # Memory usage distribution
        df['memory_delta_mb'].hist(bins=50, ax=axes[0,1])
        axes[0,1].set_title('Memory Usage Distribution')
        axes[0,1].set_xlabel('Memory (MB)')
        
        # Performance by operation type
        if 'operation_name' in df.columns:
            sns.boxplot(data=df, y='operation_name', x='execution_time_ms', ax=axes[1,0])
            axes[1,0].set_title('Performance by Operation Type')
        
        # Spillage comparison
        if 'use_spillage' in df.columns:
            sns.boxplot(data=df, x='use_spillage', y='execution_time_ms', ax=axes[1,1])
            axes[1,1].set_title('Spillage Impact on Performance')
        
        plt.tight_layout()
        
        if save_plots:
            distribution_file = self.output_dir / f"performance_distributions_{results_key}_{timestamp}.png"
            plt.savefig(distribution_file, dpi=300, bbox_inches='tight')
            plot_files['distributions'] = str(distribution_file)
        
        plt.show()
        
        return plot_files
    
    def print_analysis_summary(self, results_key: str):
        """Print comprehensive analysis summary"""
        if results_key not in self.loaded_results:
            print(f"Results not found: {results_key}")
            return
        
        results = self.loaded_results[results_key]
        df = self.extract_performance_data(results)
        
        if df.empty:
            print("No performance data available for analysis")
            return
        
        print("=" * 80)
        print(f"PERFORMANCE ANALYSIS SUMMARY - {results_key}")
        print("=" * 80)
        
        # Basic statistics
        print(f"Total Samples: {len(df)}")
        print(f"Unique Scenarios: {df['scenario_id'].nunique()}")
        print(f"Operations Tested: {', '.join(df['operation_name'].unique())}")
        print()
        
        # Overall performance
        exec_stats = self.calculate_statistical_summary(df['execution_time_ms'].values)
        mem_stats = self.calculate_statistical_summary(df['memory_delta_mb'].values)
        
        print("OVERALL PERFORMANCE:")
        print(f"  Execution Time: {exec_stats.mean:.2f}ms ± {exec_stats.std:.2f}ms")
        print(f"  Range: {exec_stats.min:.2f}ms - {exec_stats.max:.2f}ms")
        print(f"  Memory Usage: {mem_stats.mean:.2f}MB ± {mem_stats.std:.2f}MB")
        print()
        
        # Scalability insights
        if 'object_count' in df.columns:
            scalability = self.analyze_scalability(df, scale_factor='object_count')
            if 'scaling_analysis' in scalability:
                scaling = scalability['scaling_analysis']
                print(f"SCALABILITY (Object Count):")
                print(f"  Linear R²: {scaling.get('linear_r_squared', 0):.3f}")
                print(f"  Complexity: {scaling.get('complexity_estimate', 'Unknown')}")
                print()
        
        # Spillage analysis
        spillage_analysis = self.analyze_spillage_impact(df)
        if 'spillage_overhead_percent' in spillage_analysis:
            print(f"SPILLAGE IMPACT:")
            print(f"  Overhead: {spillage_analysis['spillage_overhead_percent']:.1f}%")
            print(f"  Recommendation: {spillage_analysis['recommendation']}")
            print()

# Example usage
if __name__ == "__main__":
    analyzer = BenchmarkAnalyzer()
    
    # Example: Load and analyze results (if available)
    try:
        # This would load actual benchmark results
        results_file = "benchmark_results/benchmark_detailed_quick_20250107_120000.json"
        
        if Path(results_file).exists():
            analyzer.load_benchmark_results(results_file)
            analyzer.print_analysis_summary("benchmark_detailed_quick_20250107_120000")
            report = analyzer.generate_performance_report("benchmark_detailed_quick_20250107_120000")
            plots = analyzer.create_performance_visualizations("benchmark_detailed_quick_20250107_120000")
        else:
            print("No benchmark results found. Run benchmark_runner.py first.")
            
    except Exception as e:
        print(f"Analysis example failed: {e}")
        print("This is expected if no benchmark results are available yet.")