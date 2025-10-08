# Plot Files Index - benchmark_detailed_comprehensive_20250916_075417
Generated on: 2025-09-16 07:55:05

This directory contains individual plot files saved from the benchmark report, 
formatted for thesis use with high resolution (300 DPI) and clean styling.

## Available Formats:
- **PNG**: High-resolution raster images (300 DPI) - good for most thesis applications
- **SVG**: Vector graphics - scalable without quality loss, ideal for academic publications  
- **PDF**: Vector format - publication-ready, perfect for LaTeX documents

## Plot Categories:

### Performance Overview Plots:
- `performance_overview.*` - Overall performance metrics distribution
- `execution_time_distribution.*` - Timing analysis across scenarios
- `memory_usage_analysis.*` - Memory consumption patterns

### Optimization Analysis Plots:
- `suffix_stitching_optimization.*` - A* suffix stitching performance comparison
- `optimization_matrix_heatmap.*` - Complete optimization combinations analysis
- `performance_improvements.*` - Summary of all optimization benefits

### Spillage Model Analysis:
- `spillage_impact.*` - Performance impact of spillage model enabled/disabled

### Scalability Analysis:
- `scalability_analysis.*` - Performance scaling with problem size
- `grid_size_performance.*` - Performance vs. grid size relationship

## Usage in Thesis:

### LaTeX Example:
```latex
\begin{figure}[htbp]
    \centering
    \includegraphics[width=0.8\textwidth]{plots/suffix_stitching_optimization.pdf}
    \caption{A* Suffix Stitching Optimization Performance Analysis}
    \label{fig:suffix_stitching_opt}
\end{figure}
```

### Word/LibreOffice:
Use the PNG files (300 DPI) for high-quality images that maintain clarity when scaled.

## File Summary:

### PNG Files (3 files):
- `performance_overview.png` (748.4 KB)
- `scalability_analysis.png` (902.5 KB)
- `spillage_impact.png` (331.2 KB)

### SVG Files (3 files):
- `performance_overview.svg` (1.2 MB)
- `scalability_analysis.svg` (2.6 MB)
- `spillage_impact.svg` (278.4 KB)

### PDF Files (3 files):
- `performance_overview.pdf` (49.2 KB)
- `scalability_analysis.pdf` (149.2 KB)
- `spillage_impact.pdf` (44.3 KB)

## Plot Descriptions for Thesis Context:

### Suffix Stitching Optimization Analysis
Shows the performance improvement achieved by the A* suffix stitching optimization.
Key metrics: execution time reduction, memory efficiency, path quality maintenance.

### Optimization Matrix Heatmap  
Demonstrates how different optimization combinations affect performance.
Useful for showing incremental benefits of each optimization technique.

### Performance Improvements Summary
Bar chart showing percentage improvements across different metrics.
Perfect for highlighting the quantitative benefits of your algorithmic contributions.

### Spillage Model Impact Analysis
Compares performance with and without the spillage physics simulation.
Shows the computational cost of adding realistic spillage behavior.

## Thesis Integration Tips:

1. **Figure Captions**: Include quantitative results (e.g., "X% improvement in execution time")
2. **References**: Cite the specific benchmark configuration and test parameters
3. **Consistency**: Use the same format (PNG/SVG/PDF) throughout your thesis
4. **Resolution**: All images are saved at 300 DPI for print-quality output
5. **Color Scheme**: Plots use a consistent, professional color scheme suitable for academic publication

## Technical Details:
- Font sizes optimized for thesis readability (12pt base, 14pt labels, 16pt titles)
- High contrast colors for both color and grayscale printing
- Clean, minimal styling following academic publication standards
- Tight bounding boxes for efficient space usage in documents
