"""
Comprehensive Benchmark Report Generator
Creates professional HTML reports with embedded visualizations and statistical analysis
"""

import json
import base64
import io
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np

# Import our analysis components
from benchmark_analyzer import BenchmarkAnalyzer, StatisticalSummary

@dataclass
class ReportSection:
    """Individual report section configuration"""
    title: str
    content: str
    charts: List[str] = None
    importance: str = "normal"  # critical, high, normal, low

class BenchmarkReportGenerator:
    """Professional HTML report generator for benchmark results"""
    
    def __init__(self, output_directory: str = "benchmark_reports"):
        self.output_dir = Path(output_directory)
        self.output_dir.mkdir(exist_ok=True)
        
        self.analyzer = BenchmarkAnalyzer()
        self.report_sections: List[ReportSection] = []
        
        # Chart storage for embedding
        self.embedded_charts: Dict[str, str] = {}
        
        # Configure matplotlib for report generation
        plt.style.use('seaborn-v0_8-whitegrid')
        plt.rcParams['figure.dpi'] = 150
        plt.rcParams['savefig.dpi'] = 150
        plt.rcParams['font.size'] = 10
    
    def load_and_analyze_results(self, results_file: str) -> Dict[str, Any]:
        """Load results and perform comprehensive analysis"""
        results = self.analyzer.load_benchmark_results(results_file)
        results_key = Path(results_file).stem
        
        # Generate comprehensive analysis
        report_data = self.analyzer.generate_performance_report(results_key)
        
        # Extract DataFrame for detailed analysis
        df = self.analyzer.extract_performance_data(results)
        
        return {
            'results': results,
            'analysis': report_data,
            'dataframe': df,
            'results_key': results_key
        }
    
    def create_embedded_chart(self, fig, chart_id: str) -> str:
        """Convert matplotlib figure to embedded base64 image"""
        buffer = io.BytesIO()
        fig.savefig(buffer, format='png', bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        buffer.seek(0)
        
        # Convert to base64
        image_base64 = base64.b64encode(buffer.getvalue()).decode()
        buffer.close()
        
        # Store for embedding
        self.embedded_charts[chart_id] = image_base64
        
        return f'<img src="data:image/png;base64,{image_base64}" class="chart-image" id="{chart_id}">'
    
    def generate_performance_charts(self, df: pd.DataFrame) -> Dict[str, str]:
        """Generate comprehensive performance visualization charts"""
        charts = {}
        
        if df.empty:
            return charts
        
        # 1. Executive Summary Chart - Key Metrics Overview
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Performance Overview', fontsize=16, fontweight='bold')
        
        # Execution time distribution
        df['execution_time_ms'].hist(bins=30, ax=axes[0,0], alpha=0.7, color='skyblue')
        axes[0,0].set_title('Execution Time Distribution')
        axes[0,0].set_xlabel('Time (ms)')
        axes[0,0].set_ylabel('Frequency')
        
        # Memory usage scatter
        if 'object_count' in df.columns:
            axes[0,1].scatter(df['object_count'], df['memory_delta_mb'], alpha=0.6, color='lightcoral')
            axes[0,1].set_title('Memory Usage vs Object Count')
            axes[0,1].set_xlabel('Object Count')
            axes[0,1].set_ylabel('Memory Delta (MB)')
        
        # Performance by operation type
        if 'operation_name' in df.columns and df['operation_name'].nunique() > 1:
            operation_means = df.groupby('operation_name')['execution_time_ms'].mean().sort_values()
            operation_means.plot(kind='barh', ax=axes[1,0], color='lightgreen')
            axes[1,0].set_title('Average Performance by Operation')
            axes[1,0].set_xlabel('Average Time (ms)')
        
        # Scalability preview
        if 'grid_size' in df.columns:
            size_means = df.groupby('grid_size')['execution_time_ms'].mean()
            size_means.plot(kind='line', ax=axes[1,1], marker='o', color='orange')
            axes[1,1].set_title('Performance Scaling by Grid Size')
            axes[1,1].set_xlabel('Grid Size')
            axes[1,1].set_ylabel('Average Time (ms)')
        
        plt.tight_layout()
        charts['performance_overview'] = self.create_embedded_chart(fig, 'performance_overview')
        plt.close(fig)
        
        # 2. Detailed Scalability Analysis
        if 'object_count' in df.columns and 'grid_size' in df.columns:
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            fig.suptitle('Scalability Analysis', fontsize=16, fontweight='bold')
            
            # Object count scaling
            sns.regplot(data=df, x='object_count', y='execution_time_ms', 
                       ax=axes[0,0], scatter_kws={'alpha': 0.5})
            axes[0,0].set_title('Performance vs Object Count')
            axes[0,0].set_xlabel('Object Count')
            axes[0,0].set_ylabel('Execution Time (ms)')
            
            # Grid size scaling
            sns.boxplot(data=df, x='grid_size', y='execution_time_ms', ax=axes[0,1])
            axes[0,1].set_title('Performance Distribution by Grid Size')
            axes[0,1].set_xlabel('Grid Size')
            axes[0,1].set_ylabel('Execution Time (ms)')
            
            # Object density impact
            if 'object_density' in df.columns:
                sns.scatterplot(data=df, x='object_density', y='execution_time_ms', 
                              hue='grid_size', ax=axes[1,0])
                axes[1,0].set_title('Performance vs Object Density')
                axes[1,0].set_xlabel('Objects per Cell')
                axes[1,0].set_ylabel('Execution Time (ms)')
            
            # Time per object efficiency
            if 'time_per_object' in df.columns:
                sns.lineplot(data=df, x='object_count', y='time_per_object', 
                           hue='grid_size', ax=axes[1,1])
                axes[1,1].set_title('Efficiency: Time per Object')
                axes[1,1].set_xlabel('Object Count')
                axes[1,1].set_ylabel('Time per Object (ms)')
            
            plt.tight_layout()
            charts['scalability_analysis'] = self.create_embedded_chart(fig, 'scalability_analysis')
            plt.close(fig)
        
        # 3. Spillage Impact Analysis
        if 'use_spillage' in df.columns:
            fig, axes = plt.subplots(1, 3, figsize=(15, 5))
            fig.suptitle('Spillage Mode Impact Analysis', fontsize=16, fontweight='bold')
            
            # Performance comparison
            sns.boxplot(data=df, x='use_spillage', y='execution_time_ms', ax=axes[0])
            axes[0].set_title('Execution Time by Spillage Mode')
            axes[0].set_xlabel('Spillage Enabled')
            axes[0].set_ylabel('Execution Time (ms)')
            
            # Memory impact
            sns.boxplot(data=df, x='use_spillage', y='memory_delta_mb', ax=axes[1])
            axes[1].set_title('Memory Usage by Spillage Mode')
            axes[1].set_xlabel('Spillage Enabled')
            axes[1].set_ylabel('Memory Delta (MB)')
            
            # Distribution comparison
            spillage_true = df[df['use_spillage'] == True]['execution_time_ms']
            spillage_false = df[df['use_spillage'] == False]['execution_time_ms']
            
            axes[2].hist([spillage_false, spillage_true], bins=20, alpha=0.7, 
                        label=['Without Spillage', 'With Spillage'], color=['lightblue', 'lightcoral'])
            axes[2].set_title('Performance Distribution Comparison')
            axes[2].set_xlabel('Execution Time (ms)')
            axes[2].set_ylabel('Frequency')
            axes[2].legend()
            
            plt.tight_layout()
            charts['spillage_impact'] = self.create_embedded_chart(fig, 'spillage_impact')
            plt.close(fig)
        
        return charts
    
    def generate_executive_summary(self, analysis: Dict[str, Any], df: pd.DataFrame) -> str:
        """Generate executive summary section"""
        metadata = analysis.get('metadata', {})
        overall_perf = analysis.get('overall_performance', {})
        scalability = analysis.get('scalability_analysis', {})
        spillage = analysis.get('spillage_analysis', {})
        
        # Key statistics
        exec_stats = overall_perf.get('execution_time', {})
        avg_time = exec_stats.get('mean', 0)
        time_std = exec_stats.get('std', 0)
        
        # Performance rating
        if avg_time < 100:
            performance_rating = "Excellent"
            rating_color = "green"
        elif avg_time < 500:
            performance_rating = "Good"
            rating_color = "orange"
        else:
            performance_rating = "Needs Optimization"
            rating_color = "red"
        
        summary = f"""
        <div class="executive-summary">
            <h2>Executive Summary</h2>
            
            <div class="key-metrics">
                <div class="metric-card">
                    <h3>Overall Performance</h3>
                    <div class="metric-value" style="color: {rating_color};">{performance_rating}</div>
                    <div class="metric-details">
                        Average: {avg_time:.1f}ms ± {time_std:.1f}ms
                    </div>
                </div>
                
                <div class="metric-card">
                    <h3>Test Coverage</h3>
                    <div class="metric-value">{metadata.get('total_samples', 0):,} samples</div>
                    <div class="metric-details">
                        {metadata.get('unique_scenarios', 0)} scenarios tested
                    </div>
                </div>
                
                <div class="metric-card">
                    <h3>Scalability</h3>
                    <div class="metric-value">
        """
        
        # Add scalability assessment
        if 'object_count' in scalability:
            obj_analysis = scalability['object_count'].get('scaling_analysis', {})
            complexity = obj_analysis.get('complexity_estimate', 'Unknown')
            r_squared = obj_analysis.get('linear_r_squared', 0)
            
            if r_squared > 0.9:
                scalability_rating = "Predictable"
            elif r_squared > 0.7:
                scalability_rating = "Good"
            else:
                scalability_rating = "Variable"
            
            summary += f"""
                        {scalability_rating}
                    </div>
                    <div class="metric-details">
                        {complexity}
                    </div>
                </div>
            </div>
            
            <div class="key-findings">
                <h3>Key Findings</h3>
                <ul>
            """
            
            # Add key findings
            if spillage.get('spillage_overhead_percent'):
                overhead = spillage['spillage_overhead_percent']
                summary += f"<li>Spillage mode adds {overhead:.1f}% performance overhead</li>"
            
            if obj_analysis.get('complexity_estimate'):
                summary += f"<li>Algorithm exhibits {complexity.lower()} scaling behavior</li>"
            
            if avg_time > 1000:
                summary += f"<li>Performance optimization recommended for large scenarios</li>"
            else:
                summary += f"<li>Performance is within acceptable ranges for tested scenarios</li>"
        
        summary += """
                </ul>
            </div>
        </div>
        """
        
        return summary
    
    def generate_detailed_analysis(self, analysis: Dict[str, Any]) -> str:
        """Generate detailed analysis section"""
        scalability = analysis.get('scalability_analysis', {})
        spillage = analysis.get('spillage_analysis', {})
        operation_comp = analysis.get('operation_comparison', {})
        
        detailed = """
        <div class="detailed-analysis">
            <h2>Detailed Performance Analysis</h2>
        """
        
        # Scalability Analysis
        if scalability:
            detailed += """
            <div class="analysis-section">
                <h3>Scalability Analysis</h3>
                <div class="scalability-results">
            """
            
            for factor, analysis_data in scalability.items():
                if 'scaling_analysis' in analysis_data:
                    scaling = analysis_data['scaling_analysis']
                    complexity = scaling.get('complexity_estimate', 'Unknown')
                    r_squared = scaling.get('linear_r_squared', 0)
                    
                    detailed += f"""
                    <div class="scale-factor">
                        <h4>{factor.replace('_', ' ').title()}</h4>
                        <p><strong>Complexity:</strong> {complexity}</p>
                        <p><strong>Linear Fit R²:</strong> {r_squared:.3f}</p>
                        <p><strong>Predictability:</strong> {'High' if r_squared > 0.9 else 'Moderate' if r_squared > 0.7 else 'Low'}</p>
                    </div>
                    """
            
            detailed += "</div></div>"
        
        # Spillage Analysis
        if spillage and 'spillage_overhead_percent' in spillage:
            overhead = spillage['spillage_overhead_percent']
            recommendation = spillage.get('recommendation', 'No recommendation available')
            
            detailed += f"""
            <div class="analysis-section">
                <h3>Spillage Mode Impact</h3>
                <div class="spillage-analysis">
                    <p><strong>Performance Overhead:</strong> {overhead:.1f}%</p>
                    <p><strong>Recommendation:</strong> {recommendation}</p>
                    <p><strong>Statistical Significance:</strong> {'Yes' if hasattr(spillage, 'is_significant') and spillage.is_significant else 'No'}</p>
                </div>
            </div>
            """
        
        # Operation Comparison
        if operation_comp and 'operation_statistics' in operation_comp:
            detailed += """
            <div class="analysis-section">
                <h3>Operation Performance Comparison</h3>
                <table class="performance-table">
                    <thead>
                        <tr>
                            <th>Operation</th>
                            <th>Average Time (ms)</th>
                            <th>Std Dev (ms)</th>
                            <th>Min (ms)</th>
                            <th>Max (ms)</th>
                            <th>Samples</th>
                        </tr>
                    </thead>
                    <tbody>
            """
            
            op_stats = operation_comp['operation_statistics']
            for operation, stats in op_stats.items():
                detailed += f"""
                        <tr>
                            <td>{operation.replace('_', ' ').title()}</td>
                            <td>{stats['mean']:.2f}</td>
                            <td>{stats['std']:.2f}</td>
                            <td>{stats['min']:.2f}</td>
                            <td>{stats['max']:.2f}</td>
                            <td>{stats['count']}</td>
                        </tr>
                """
            
            detailed += """
                    </tbody>
                </table>
            </div>
            """
        
        detailed += "</div>"
        return detailed
    
    def generate_html_report(self, results_file: str, 
                           report_title: str = None,
                           include_raw_data: bool = False) -> str:
        """Generate comprehensive HTML report"""
        
        # Load and analyze data
        data = self.load_and_analyze_results(results_file)
        analysis = data['analysis']
        df = data['dataframe']
        results_key = data['results_key']
        
        if not report_title:
            report_title = f"Earth Moving Algorithm Performance Report - {results_key}"
        
        # Generate charts
        charts = self.generate_performance_charts(df)
        
        # Generate report sections
        executive_summary = self.generate_executive_summary(analysis, df)
        detailed_analysis = self.generate_detailed_analysis(analysis)
        
        # Create HTML report
        html_content = f"""
        <!DOCTYPE html>
        <html lang="en">
        <head>
            <meta charset="UTF-8">
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <title>{report_title}</title>
            <style>
                {self.get_report_css()}
            </style>
        </head>
        <body>
            <div class="container">
                <header>
                    <h1>{report_title}</h1>
                    <div class="report-info">
                        <span>Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</span>
                        <span>Results File: {Path(results_file).name}</span>
                    </div>
                </header>
                
                {executive_summary}
                
                <div class="charts-section">
                    <h2>Performance Visualizations</h2>
                    
                    <div class="chart-container">
                        <h3>Performance Overview</h3>
                        {charts.get('performance_overview', '<p>Chart not available</p>')}
                    </div>
                    
                    <div class="chart-container">
                        <h3>Scalability Analysis</h3>
                        {charts.get('scalability_analysis', '<p>Chart not available</p>')}
                    </div>
                    
                    <div class="chart-container">
                        <h3>Spillage Impact Analysis</h3>
                        {charts.get('spillage_impact', '<p>Chart not available</p>')}
                    </div>
                </div>
                
                {detailed_analysis}
                
                <div class="recommendations">
                    <h2>Recommendations</h2>
                    {self.generate_recommendations(analysis, df)}
                </div>
                
                <footer>
                    <p>Report generated by Earth Moving Algorithm Benchmark System</p>
                    <p>For questions or issues, contact the development team</p>
                </footer>
            </div>
        </body>
        </html>
        """
        
        # Save HTML report
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = self.output_dir / f"performance_report_{results_key}_{timestamp}.html"
        
        with open(report_filename, 'w', encoding='utf-8') as f:
            f.write(html_content)
        
        print(f"Comprehensive HTML report generated: {report_filename}")
        return str(report_filename)
    
    def generate_recommendations(self, analysis: Dict[str, Any], df: pd.DataFrame) -> str:
        """Generate performance recommendations"""
        recommendations = []
        
        # Overall performance recommendations
        overall_perf = analysis.get('overall_performance', {})
        avg_time = overall_perf.get('execution_time', {}).get('mean', 0)
        
        if avg_time > 1000:
            recommendations.append("Consider performance optimization for scenarios exceeding 1 second execution time")
        
        # Scalability recommendations
        scalability = analysis.get('scalability_analysis', {})
        if 'object_count' in scalability:
            complexity = scalability['object_count'].get('scaling_analysis', {}).get('complexity_estimate', '')
            if 'Quadratic' in complexity:
                recommendations.append("Quadratic scaling detected - investigate algorithmic optimization for large object counts")
        
        # Spillage recommendations
        spillage = analysis.get('spillage_analysis', {})
        if spillage.get('recommendation'):
            recommendations.append(f"Spillage Mode: {spillage['recommendation']}")
        
        # Memory recommendations
        mem_stats = overall_perf.get('memory_usage', {})
        avg_memory = mem_stats.get('mean', 0)
        if avg_memory > 100:
            recommendations.append("Monitor memory usage - consider memory optimization for large scenarios")
        
        if not recommendations:
            recommendations.append("Performance is within acceptable ranges - no immediate optimization required")
        
        rec_html = "<ul>"
        for rec in recommendations:
            rec_html += f"<li>{rec}</li>"
        rec_html += "</ul>"
        
        return rec_html
    
    def get_report_css(self) -> str:
        """Return CSS styles for HTML report"""
        return """
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
            color: #333;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 30px;
            box-shadow: 0 0 20px rgba(0,0,0,0.1);
            border-radius: 8px;
        }
        
        header {
            text-align: center;
            margin-bottom: 40px;
            padding-bottom: 20px;
            border-bottom: 2px solid #e0e0e0;
        }
        
        header h1 {
            color: #2c3e50;
            margin-bottom: 10px;
            font-size: 2.5em;
        }
        
        .report-info {
            color: #7f8c8d;
            font-size: 0.9em;
        }
        
        .report-info span {
            margin: 0 15px;
        }
        
        .executive-summary {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            border-radius: 8px;
            margin-bottom: 40px;
        }
        
        .key-metrics {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin: 20px 0;
        }
        
        .metric-card {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 8px;
            text-align: center;
            backdrop-filter: blur(10px);
        }
        
        .metric-card h3 {
            margin-top: 0;
            font-size: 1.1em;
            opacity: 0.9;
        }
        
        .metric-value {
            font-size: 2em;
            font-weight: bold;
            margin: 10px 0;
        }
        
        .metric-details {
            font-size: 0.9em;
            opacity: 0.8;
        }
        
        .key-findings ul {
            list-style-type: none;
            padding-left: 0;
        }
        
        .key-findings li {
            padding: 5px 0;
            padding-left: 20px;
            position: relative;
        }
        
        .key-findings li:before {
            content: "→";
            position: absolute;
            left: 0;
            color: #f39c12;
            font-weight: bold;
        }
        
        .charts-section {
            margin: 40px 0;
        }
        
        .chart-container {
            margin-bottom: 40px;
            text-align: center;
        }
        
        .chart-image {
            max-width: 100%;
            height: auto;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
        }
        
        .detailed-analysis {
            margin: 40px 0;
        }
        
        .analysis-section {
            margin-bottom: 30px;
            padding: 20px;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            background: #fafafa;
        }
        
        .analysis-section h3 {
            color: #2c3e50;
            margin-top: 0;
        }
        
        .performance-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        
        .performance-table th,
        .performance-table td {
            padding: 10px;
            text-align: left;
            border-bottom: 1px solid #e0e0e0;
        }
        
        .performance-table th {
            background-color: #34495e;
            color: white;
            font-weight: bold;
        }
        
        .performance-table tr:nth-child(even) {
            background-color: #f8f9fa;
        }
        
        .scalability-results {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 20px;
            margin-top: 15px;
        }
        
        .scale-factor {
            padding: 15px;
            background: white;
            border-radius: 8px;
            border-left: 4px solid #3498db;
        }
        
        .scale-factor h4 {
            margin-top: 0;
            color: #2c3e50;
        }
        
        .recommendations {
            background: #d5f4e6;
            padding: 25px;
            border-radius: 8px;
            border-left: 5px solid #27ae60;
            margin: 40px 0;
        }
        
        .recommendations h2 {
            color: #27ae60;
            margin-top: 0;
        }
        
        .recommendations ul {
            margin-bottom: 0;
        }
        
        .recommendations li {
            margin-bottom: 8px;
            line-height: 1.5;
        }
        
        footer {
            text-align: center;
            margin-top: 40px;
            padding-top: 20px;
            border-top: 1px solid #e0e0e0;
            color: #7f8c8d;
            font-size: 0.9em;
        }
        
        h2 {
            color: #2c3e50;
            border-bottom: 2px solid #3498db;
            padding-bottom: 10px;
        }
        
        h3 {
            color: #34495e;
        }
        """

# Example usage
if __name__ == "__main__":
    generator = BenchmarkReportGenerator()
    
    # Example: Generate report (if results available)
    try:
        results_file = "benchmark_results/benchmark_detailed_quick_20250107_120000.json"
        
        if Path(results_file).exists():
            report_path = generator.generate_html_report(
                results_file,
                report_title="Earth Moving Algorithm Performance Analysis",
                include_raw_data=False
            )
            print(f"Report generated successfully: {report_path}")
        else:
            print("No benchmark results found. Run benchmark_runner.py first to generate data.")
            print("This report generator will create comprehensive HTML reports with:")
            print("- Executive summary with key metrics")
            print("- Interactive performance visualizations")  
            print("- Detailed statistical analysis")
            print("- Performance recommendations")
            print("- Professional styling and layout")
            
    except Exception as e:
        print(f"Report generation example failed: {e}")
        print("This is expected if no benchmark results are available yet.")