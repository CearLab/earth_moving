"""
Generate Individual Thesis-Quality Plots
Creates separate plot files for each performance metric instead of clustered multi-panel figures
"""

import json
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import numpy as np

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

def load_benchmark_data():
    """Load the latest benchmark results"""
    results_file = "analysis_results/performance_report_benchmark_detailed_comprehensive_20250916_075417_20250916_075431.json"
    
    with open(results_file, 'r') as f:
        data = json.load(f)
    
    # Extract operation statistics
    ops = data['operation_comparison']['operation_statistics']
    
    # Create DataFrame for analysis
    plot_data = []
    
    for op_name, stats in ops.items():
        row = {
            'operation': op_name,
            'mean_time': stats['mean'],
            'median_time': stats['median'],
            'std_dev': stats['std'],
            'min_time': stats['min'],
            'max_time': stats['max'],
            'count': stats['count']
        }
        
        # Extract grid size and operation type
        if 'full_calculation_' in op_name:
            parts = op_name.split('_')
            row['operation_type'] = 'Full Calculation'
            row['grid_size'] = int(parts[2].split('x')[0])
            row['object_count'] = int(parts[3].replace('obj', ''))
        elif 'pure_env_update_' in op_name:
            grid_size = int(op_name.split('_')[-1].split('x')[0])
            row['operation_type'] = 'Environment Update'
            row['grid_size'] = grid_size
            row['object_count'] = 0  # Not applicable
        elif 'strategic_analysis_' in op_name:
            grid_size = int(op_name.split('_')[-1].split('x')[0])
            row['operation_type'] = 'Strategic Analysis'
            row['grid_size'] = grid_size
            row['object_count'] = 0
        elif 'save_load_' in op_name:
            grid_size = int(op_name.split('_')[-1].split('x')[0])
            row['operation_type'] = 'Save/Load'
            row['grid_size'] = grid_size
            row['object_count'] = 0
        else:
            continue
            
        plot_data.append(row)
    
    return pd.DataFrame(plot_data), data

def save_plot(fig, name, plots_dir):
    """Save plot in multiple formats"""
    # Tight layout and clean appearance
    fig.tight_layout()
    
    # Save in all formats
    fig.savefig(plots_dir / f"{name}.png", dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(plots_dir / f"{name}.svg", bbox_inches='tight', facecolor='white') 
    fig.savefig(plots_dir / f"{name}.pdf", bbox_inches='tight', facecolor='white')
    
    plt.close(fig)
    print(f"[OK] Generated: {name}")

def create_individual_plots():
    """Create individual thesis-quality plots"""
    
    df, data = load_benchmark_data()
    
    # Create plots directory
    plots_dir = Path("benchmark_reports/individual_plots")
    plots_dir.mkdir(parents=True, exist_ok=True)
    
    print("Generating individual thesis-quality plots...")
    
    # 1. Grid Size vs Performance (Environment Update vs Full Calculation)
    fig, ax = plt.subplots(figsize=(10, 6))
    
    env_updates = df[df['operation_type'] == 'Environment Update']
    full_calcs = df[df['operation_type'] == 'Full Calculation'].groupby('grid_size')['mean_time'].mean().reset_index()
    
    ax.plot(env_updates['grid_size'], env_updates['mean_time'], 'o-', linewidth=3, markersize=8, 
           label='Environment Update', color='#2E8B57')
    ax.plot(full_calcs['grid_size'], full_calcs['mean_time'], 's-', linewidth=3, markersize=8,
           label='Full Calculation', color='#DC143C')
    
    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Execution Time (ms)')
    ax.set_title('Performance Comparison: Environment Update vs Full Calculation')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    save_plot(fig, "grid_size_performance_comparison", plots_dir)
    
    # 2. Environment Update Scaling
    fig, ax = plt.subplots(figsize=(8, 6))
    
    ax.bar(env_updates['grid_size'].astype(str), env_updates['mean_time'], 
           color='#2E8B57', alpha=0.7, edgecolor='black')
    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Environment Update Time (ms)')
    ax.set_title('Environment Update Performance Scaling')
    ax.grid(True, alpha=0.3, axis='y')
    
    save_plot(fig, "environment_update_scaling", plots_dir)
    
    # 3. Full Calculation by Object Count
    fig, ax = plt.subplots(figsize=(10, 6))
    
    full_calc_data = df[df['operation_type'] == 'Full Calculation']
    
    for grid_size in sorted(full_calc_data['grid_size'].unique()):
        subset = full_calc_data[full_calc_data['grid_size'] == grid_size]
        ax.plot(subset['object_count'], subset['mean_time'], 'o-', linewidth=2, markersize=6,
               label=f'{grid_size}×{grid_size}')
    
    ax.set_xlabel('Object Count')
    ax.set_ylabel('Execution Time (ms)')
    ax.set_title('Full Calculation Performance by Object Count')
    ax.legend(title='Grid Size')
    ax.grid(True, alpha=0.3)
    
    save_plot(fig, "full_calculation_by_object_count", plots_dir)
    
    # 4. Performance Improvement Bar Chart
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Calculate improvements
    improvements = []
    grid_sizes = []
    
    for grid_size in sorted(env_updates['grid_size'].unique()):
        env_time = env_updates[env_updates['grid_size'] == grid_size]['mean_time'].iloc[0]
        full_time = full_calcs[full_calcs['grid_size'] == grid_size]['mean_time'].iloc[0]
        improvement = ((full_time - env_time) / full_time) * 100
        
        improvements.append(improvement)
        grid_sizes.append(f'{grid_size}×{grid_size}')
    
    bars = ax.bar(grid_sizes, improvements, color='#4CAF50', alpha=0.8, edgecolor='black')
    
    # Add value labels on bars
    for bar, improvement in zip(bars, improvements):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
               f'{improvement:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Performance Improvement (%)')
    ax.set_title('Environment Update Optimization Benefits')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, max(improvements) + 5)
    
    save_plot(fig, "environment_update_improvement_percentage", plots_dir)
    
    # 5. Speedup Factor Chart
    fig, ax = plt.subplots(figsize=(8, 6))
    
    speedups = []
    for grid_size in sorted(env_updates['grid_size'].unique()):
        env_time = env_updates[env_updates['grid_size'] == grid_size]['mean_time'].iloc[0]
        full_time = full_calcs[full_calcs['grid_size'] == grid_size]['mean_time'].iloc[0]
        speedup = full_time / env_time
        speedups.append(speedup)
    
    bars = ax.bar(grid_sizes, speedups, color='#FF9800', alpha=0.8, edgecolor='black')
    
    # Add value labels
    for bar, speedup in zip(bars, speedups):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 0.1,
               f'{speedup:.1f}x', ha='center', va='bottom', fontweight='bold')
    
    ax.set_xlabel('Grid Size')
    ax.set_ylabel('Speedup Factor')
    ax.set_title('Environment Update Speedup Over Full Calculation')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, max(speedups) + 1)
    
    save_plot(fig, "environment_update_speedup_factor", plots_dir)
    
    # 6. Operation Performance Overview
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Get one representative data point per operation type per grid size
    overview_data = []
    for op_type in ['Environment Update', 'Strategic Analysis', 'Save/Load']:
        subset = df[df['operation_type'] == op_type]
        for _, row in subset.iterrows():
            overview_data.append({
                'operation': f"{op_type}\n({row['grid_size']}×{row['grid_size']})",
                'mean_time': row['mean_time'],
                'operation_type': op_type
            })
    
    # Add average full calculation for comparison
    for grid_size in sorted(full_calcs['grid_size'].unique()):
        time = full_calcs[full_calcs['grid_size'] == grid_size]['mean_time'].iloc[0]
        overview_data.append({
            'operation': f"Full Calculation\n({grid_size}×{grid_size})",
            'mean_time': time,
            'operation_type': 'Full Calculation'
        })
    
    overview_df = pd.DataFrame(overview_data)
    
    # Create color map
    color_map = {
        'Environment Update': '#2E8B57',
        'Full Calculation': '#DC143C', 
        'Strategic Analysis': '#4169E1',
        'Save/Load': '#9932CC'
    }
    
    colors = [color_map[op_type] for op_type in overview_df['operation_type']]
    
    bars = ax.bar(range(len(overview_df)), overview_df['mean_time'], 
                  color=colors, alpha=0.7, edgecolor='black')
    
    ax.set_xlabel('Operation Type')
    ax.set_ylabel('Execution Time (ms)')
    ax.set_title('Performance Overview by Operation Type')
    ax.set_xticks(range(len(overview_df)))
    ax.set_xticklabels(overview_df['operation'], rotation=45, ha='right')
    
    # Create legend
    legend_elements = [plt.Rectangle((0,0),1,1, facecolor=color_map[op], alpha=0.7, edgecolor='black')
                      for op in color_map.keys()]
    ax.legend(legend_elements, color_map.keys(), loc='upper left')
    
    ax.grid(True, alpha=0.3, axis='y')
    
    save_plot(fig, "operation_performance_overview", plots_dir)
    
    # 7. Memory Usage Analysis (if available)
    if 'overall_performance' in data and 'memory_usage' in data['overall_performance']:
        fig, ax = plt.subplots(figsize=(8, 6))
        
        memory_stats = data['overall_performance']['memory_usage']
        
        categories = ['Mean', 'Median', 'Min', 'Max']
        values = [memory_stats['mean'], memory_stats['median'], 
                 memory_stats['min'], memory_stats['max']]
        colors = ['#4CAF50', '#2196F3', '#FF9800', '#F44336']
        
        bars = ax.bar(categories, values, color=colors, alpha=0.7, edgecolor='black')
        
        # Add value labels
        for bar, value in zip(bars, values):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + (0.1 if height > 0 else -0.3),
                   f'{value:.2f} MB', ha='center', va='bottom' if height > 0 else 'top', 
                   fontweight='bold')
        
        ax.set_ylabel('Memory Usage (MB)')
        ax.set_title('Memory Usage Statistics')
        ax.grid(True, alpha=0.3, axis='y')
        
        save_plot(fig, "memory_usage_analysis", plots_dir)
    
    # 8. Performance Distribution Histogram
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Combine all execution times
    all_times = df['mean_time'].values
    
    ax.hist(all_times, bins=20, color='#607D8B', alpha=0.7, edgecolor='black')
    ax.axvline(np.mean(all_times), color='red', linestyle='--', linewidth=2, 
              label=f'Mean: {np.mean(all_times):.1f} ms')
    ax.axvline(np.median(all_times), color='blue', linestyle='--', linewidth=2,
              label=f'Median: {np.median(all_times):.1f} ms')
    
    ax.set_xlabel('Execution Time (ms)')
    ax.set_ylabel('Frequency')
    ax.set_title('Performance Distribution Across All Operations')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    save_plot(fig, "performance_distribution_histogram", plots_dir)
    
    print(f"\n[SUCCESS] Generated 8 individual plots in: {plots_dir}")
    print("Available formats: PNG (300 DPI), SVG (vector), PDF (publication-ready)")

if __name__ == "__main__":
    create_individual_plots()