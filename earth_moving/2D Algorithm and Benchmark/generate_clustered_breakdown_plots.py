"""
Break Down Clustered Plots into Individual Components
Separates the 3 remaining multi-panel figures into individual plots
"""

import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import numpy as np
from benchmark_analyzer import BenchmarkAnalyzer

# Configure matplotlib for thesis-quality plots
plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 12
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['legend.fontsize'] = 12
plt.rcParams['xtick.labelsize'] = 11
plt.rcParams['ytick.labelsize'] = 11

def save_plot(fig, name, plots_dir):
    """Save plot in multiple formats"""
    fig.tight_layout()
    
    # Save in all formats
    fig.savefig(plots_dir / f"{name}.png", dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(plots_dir / f"{name}.svg", bbox_inches='tight', facecolor='white') 
    fig.savefig(plots_dir / f"{name}.pdf", bbox_inches='tight', facecolor='white')
    
    plt.close(fig)
    print(f"[OK] Generated: {name}")

def break_down_clustered_plots():
    """Break down the 3 clustered plots into individual components"""
    
    # Load data
    results_file = "benchmark_results/benchmark_detailed_comprehensive_20250916_075417.json"
    analyzer = BenchmarkAnalyzer()
    
    results = analyzer.load_benchmark_results(results_file)
    df = analyzer.extract_performance_data(results)
    
    # Create plots directory
    plots_dir = Path("benchmark_reports/individual_plots")
    plots_dir.mkdir(parents=True, exist_ok=True)
    
    print("Breaking down clustered plots into individual components...")
    
    # =================================================================
    # 1. PERFORMANCE OVERVIEW BREAKDOWN (4 individual plots)
    # =================================================================
    
    # Performance Overview 1: Execution Time Distribution
    fig, ax = plt.subplots(figsize=(10, 6))
    df['execution_time_ms'].hist(bins=30, ax=ax, alpha=0.7, color='skyblue', edgecolor='black')
    ax.set_title('Execution Time Distribution Across All Operations')
    ax.set_xlabel('Execution Time (ms)')
    ax.set_ylabel('Frequency')
    ax.grid(True, alpha=0.3, axis='y')
    
    # Add statistics
    mean_time = df['execution_time_ms'].mean()
    median_time = df['execution_time_ms'].median()
    ax.axvline(mean_time, color='red', linestyle='--', linewidth=2, 
              label=f'Mean: {mean_time:.0f}ms')
    ax.axvline(median_time, color='blue', linestyle='--', linewidth=2,
              label=f'Median: {median_time:.0f}ms')
    ax.legend()
    
    save_plot(fig, "execution_time_distribution", plots_dir)
    
    # Performance Overview 2: Memory Usage vs Object Count  
    if 'object_count' in df.columns and 'memory_delta_mb' in df.columns:
        fig, ax = plt.subplots(figsize=(10, 6))
        scatter = ax.scatter(df['object_count'], df['memory_delta_mb'], 
                           alpha=0.6, color='lightcoral', s=50, edgecolors='black', linewidth=0.5)
        ax.set_title('Memory Usage vs Object Count')
        ax.set_xlabel('Object Count')
        ax.set_ylabel('Memory Delta (MB)')
        ax.grid(True, alpha=0.3)
        
        # Add trend line
        if len(df) > 1:
            z = np.polyfit(df['object_count'], df['memory_delta_mb'], 1)
            p = np.poly1d(z)
            ax.plot(df['object_count'].sort_values(), p(df['object_count'].sort_values()), 
                   "r--", alpha=0.8, linewidth=2)
        
        save_plot(fig, "memory_vs_object_count", plots_dir)
    
    # Performance Overview 3: Performance by Operation Type
    if 'operation_name' in df.columns and df['operation_name'].nunique() > 1:
        fig, ax = plt.subplots(figsize=(12, 8))
        operation_means = df.groupby('operation_name')['execution_time_ms'].mean().sort_values()
        
        bars = operation_means.plot(kind='barh', ax=ax, color='lightgreen', alpha=0.8, edgecolor='black')
        ax.set_title('Average Performance by Operation Type')
        ax.set_xlabel('Average Execution Time (ms)')
        ax.set_ylabel('Operation')
        ax.grid(True, alpha=0.3, axis='x')
        
        # Add value labels
        for i, (op, value) in enumerate(operation_means.items()):
            ax.text(value + max(operation_means) * 0.01, i, f'{value:.0f}ms', 
                   va='center', fontweight='bold')
        
        save_plot(fig, "performance_by_operation_type", plots_dir)
    
    # Performance Overview 4: Scalability Preview  
    if 'grid_size' in df.columns:
        fig, ax = plt.subplots(figsize=(10, 6))
        size_means = df.groupby('grid_size')['execution_time_ms'].mean().sort_index()
        
        ax.plot(size_means.index, size_means.values, 'o-', linewidth=3, markersize=10, 
               color='orange', markeredgecolor='black', markeredgewidth=1)
        ax.set_title('Performance Scaling by Grid Size')
        ax.set_xlabel('Grid Size')
        ax.set_ylabel('Average Execution Time (ms)')
        ax.grid(True, alpha=0.3)
        
        # Add value labels
        for x, y in zip(size_means.index, size_means.values):
            ax.annotate(f'{y:.0f}ms', (x, y), textcoords="offset points", 
                       xytext=(0,10), ha='center', fontweight='bold')
        
        save_plot(fig, "scalability_preview", plots_dir)
    
    # =================================================================
    # 2. SCALABILITY ANALYSIS BREAKDOWN (4 individual plots)  
    # =================================================================
    
    if 'object_count' in df.columns and 'grid_size' in df.columns:
        
        # Scalability 1: Performance vs Object Count with Regression
        fig, ax = plt.subplots(figsize=(10, 6))
        sns.regplot(data=df, x='object_count', y='execution_time_ms', ax=ax, 
                   scatter_kws={'alpha': 0.5, 's': 50, 'edgecolors': 'black', 'linewidth': 0.5},
                   line_kws={'color': 'red', 'linewidth': 2})
        ax.set_title('Performance vs Object Count (with Trend Line)')
        ax.set_xlabel('Object Count')
        ax.set_ylabel('Execution Time (ms)')
        ax.grid(True, alpha=0.3)
        
        save_plot(fig, "performance_vs_object_count_regression", plots_dir)
        
        # Scalability 2: Performance Distribution by Grid Size (Box Plot)
        fig, ax = plt.subplots(figsize=(10, 6))
        sns.boxplot(data=df, x='grid_size', y='execution_time_ms', ax=ax)
        ax.set_title('Performance Distribution by Grid Size')
        ax.set_xlabel('Grid Size')
        ax.set_ylabel('Execution Time (ms)')
        ax.grid(True, alpha=0.3, axis='y')
        
        save_plot(fig, "performance_distribution_by_grid_size", plots_dir)
        
        # Scalability 3: Performance vs Object Density
        if 'object_density' in df.columns:
            fig, ax = plt.subplots(figsize=(10, 6))
            scatter = sns.scatterplot(data=df, x='object_density', y='execution_time_ms', 
                                    hue='grid_size', ax=ax, s=80, alpha=0.7)
            ax.set_title('Performance vs Object Density (by Grid Size)')
            ax.set_xlabel('Objects per Cell')
            ax.set_ylabel('Execution Time (ms)')
            ax.grid(True, alpha=0.3)
            plt.legend(title='Grid Size', bbox_to_anchor=(1.05, 1), loc='upper left')
            
            save_plot(fig, "performance_vs_object_density", plots_dir)
        
        # Scalability 4: Time per Object Efficiency
        if 'time_per_object' in df.columns:
            fig, ax = plt.subplots(figsize=(10, 6))
            sns.lineplot(data=df, x='object_count', y='time_per_object', 
                        hue='grid_size', ax=ax, linewidth=3, marker='o', markersize=8)
            ax.set_title('Efficiency: Time per Object by Grid Size')
            ax.set_xlabel('Object Count')
            ax.set_ylabel('Time per Object (ms)')
            ax.grid(True, alpha=0.3)
            plt.legend(title='Grid Size', bbox_to_anchor=(1.05, 1), loc='upper left')
            
            save_plot(fig, "time_per_object_efficiency", plots_dir)
    
    # =================================================================
    # 3. SPILLAGE IMPACT BREAKDOWN (3 individual plots)
    # =================================================================
    
    spillage_data = df[df['use_spillage'].notna()] if 'use_spillage' in df.columns else pd.DataFrame()
    
    if not spillage_data.empty:
        
        # Spillage 1: Direct Comparison Bar Chart
        fig, ax = plt.subplots(figsize=(8, 6))
        
        spillage_means = spillage_data.groupby('use_spillage')['execution_time_ms'].mean()
        categories = ['Without Spillage', 'With Spillage']
        values = [spillage_means[False], spillage_means[True]]
        colors = ['#4CAF50', '#FF5722']
        
        bars = ax.bar(categories, values, color=colors, alpha=0.8, edgecolor='black', linewidth=2)
        
        # Add value labels and improvement
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2., value + max(values) * 0.01,
                   f'{value:.0f}ms', ha='center', va='bottom', fontweight='bold', fontsize=12)
        
        overhead = ((values[1] - values[0]) / values[0]) * 100
        ax.text(0.5, max(values) * 0.8, f'Spillage Overhead: {overhead:.1f}%', 
               ha='center', va='center', fontsize=14, fontweight='bold',
               bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        ax.set_title('Spillage Physics Model Performance Impact')
        ax.set_ylabel('Average Execution Time (ms)')
        ax.grid(True, alpha=0.3, axis='y')
        
        save_plot(fig, "spillage_direct_comparison", plots_dir)
        
        # Spillage 2: Distribution Comparison
        fig, ax = plt.subplots(figsize=(10, 6))
        
        spillage_true = spillage_data[spillage_data['use_spillage'] == True]['execution_time_ms']
        spillage_false = spillage_data[spillage_data['use_spillage'] == False]['execution_time_ms']
        
        ax.hist(spillage_false, bins=20, alpha=0.7, label='Without Spillage', 
               color='#4CAF50', edgecolor='black')
        ax.hist(spillage_true, bins=20, alpha=0.7, label='With Spillage', 
               color='#FF5722', edgecolor='black')
        
        ax.set_title('Execution Time Distribution: Spillage Impact')
        ax.set_xlabel('Execution Time (ms)')
        ax.set_ylabel('Frequency')
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        
        save_plot(fig, "spillage_distribution_comparison", plots_dir)
        
        # Spillage 3: Performance by Grid Size
        if 'grid_size' in spillage_data.columns:
            fig, ax = plt.subplots(figsize=(10, 6))
            
            # Calculate means by grid size and spillage
            spillage_by_grid = spillage_data.groupby(['grid_size', 'use_spillage'])['execution_time_ms'].mean().unstack()
            
            spillage_by_grid.plot(kind='bar', ax=ax, color=['#4CAF50', '#FF5722'], 
                                alpha=0.8, edgecolor='black', width=0.7)
            
            ax.set_title('Spillage Impact by Grid Size')
            ax.set_xlabel('Grid Size')
            ax.set_ylabel('Average Execution Time (ms)')
            ax.legend(['Without Spillage', 'With Spillage'])
            ax.grid(True, alpha=0.3, axis='y')
            plt.xticks(rotation=0)
            
            save_plot(fig, "spillage_impact_by_grid_size", plots_dir)
    
    print(f"\n[SUCCESS] Generated 11 individual components from clustered plots!")
    print(f"Total individual plots now available: {len(list(plots_dir.glob('*.png')))}")

if __name__ == "__main__":
    break_down_clustered_plots()