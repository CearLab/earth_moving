# Complete Benchmark Report Documentation
*Understanding Every Graph, Plot, and Metric in the HTML Performance Report*

## Overview

The HTML benchmark report provides comprehensive performance analysis of the earth moving algorithm through multiple visualization types. This document explains every chart, graph, metric, and comparison in detail.

## Report Structure

The report is organized into several main sections, each with specific visualizations and purposes:

### 1. Executive Summary Section

**Purpose**: High-level overview of overall system performance  
**Key Metrics**:
- **Overall Performance Rating**: Categorizes performance as "Excellent" (<100ms), "Good" (<500ms), or "Needs Optimization" (>500ms)
- **Test Coverage**: Shows total number of samples tested and unique scenarios
- **Scalability Assessment**: Evaluates how predictably the algorithm scales

## Detailed Chart Analysis

### 1. Performance Overview Chart (4-panel executive summary)

**Location**: Top of report  
**Format**: 2x2 grid of subplots  
**Purpose**: Quick visual assessment of key performance characteristics

#### Panel 1: Execution Time Distribution (Top-Left)
- **X-axis**: Time in milliseconds
- **Y-axis**: Frequency (number of test runs)  
- **Chart Type**: Histogram with 30 bins
- **Interpretation**: 
  - Shows performance consistency - narrow distribution = consistent performance
  - Multiple peaks may indicate different performance modes (e.g., with/without spillage)
  - Long tail suggests some operations are significantly slower

#### Panel 2: Memory Usage vs Object Count (Top-Right)  
- **X-axis**: Number of objects in the simulation
- **Y-axis**: Memory delta in megabytes (memory change during operation)
- **Chart Type**: Scatter plot
- **Interpretation**:
  - Should show positive correlation - more objects require more memory
  - Scattered points suggest memory allocation variability
  - Outliers may indicate memory leaks or inefficient scenarios

#### Panel 3: Average Performance by Operation (Bottom-Left)
- **X-axis**: Average execution time in milliseconds
- **Y-axis**: Different operation types (sorted by performance)
- **Chart Type**: Horizontal bar chart
- **Operations Compared**:
  - **Pure Env Update**: Environment update after path execution
  - **Full Calculation**: Complete field calculations from scratch
  - **Spillage Enabled/Disabled**: Same operations with different spillage settings
  - **Strategic Analysis**: Planning and strategy computation
- **Interpretation**:
  - Shorter bars = faster operations
  - Pure Env Update should be significantly faster than Full Calculation
  - Shows relative cost of different algorithm phases

#### Panel 4: Performance Scaling by Grid Size (Bottom-Right)
- **X-axis**: Grid size (e.g., 15, 20, 25, 30, 35)
- **Y-axis**: Average execution time in milliseconds
- **Chart Type**: Line plot with markers
- **Interpretation**:
  - Shows scalability characteristics
  - Linear growth = O(n²) complexity (since grid area grows quadratically)
  - Exponential growth suggests algorithmic inefficiency
  - Flat line indicates good scalability

### 2. Scalability Analysis Chart (4-panel detailed analysis)

**Location**: Middle section of report  
**Format**: 2x2 grid focusing on scaling behavior  
**Purpose**: Detailed analysis of how performance changes with problem size

#### Panel 1: Performance vs Object Count (Top-Left)
- **X-axis**: Number of objects
- **Y-axis**: Execution time in milliseconds
- **Chart Type**: Scatter plot with regression line
- **Interpretation**:
  - Regression line shows trend (linear, polynomial, etc.)
  - Tight clustering around line = predictable scaling
  - R² value indicates how well object count predicts performance
  - Ideal: Linear or sub-linear growth

#### Panel 2: Performance Distribution by Grid Size (Top-Right)
- **X-axis**: Grid size categories (15x15, 20x20, etc.)
- **Y-axis**: Execution time in milliseconds
- **Chart Type**: Box plot showing distribution
- **Box Plot Elements**:
  - Box: 25th-75th percentile (middle 50% of results)
  - Line in box: Median performance
  - Whiskers: Extend to non-outlier extremes
  - Dots: Outlier results
- **Interpretation**:
  - Wider boxes = more variable performance
  - Higher medians = slower average performance
  - Outliers suggest edge cases or optimization opportunities

#### Panel 3: Performance vs Object Density (Bottom-Left)
- **X-axis**: Objects per cell (object_count / grid_area)
- **Y-axis**: Execution time in milliseconds
- **Chart Type**: Scatter plot colored by grid size
- **Interpretation**:
  - Shows whether density (not just count) affects performance
  - Different colors reveal if grid size independently affects performance
  - Dense scenarios may trigger different algorithmic paths

#### Panel 4: Efficiency - Time per Object (Bottom-Right)
- **X-axis**: Object count
- **Y-axis**: Time per object in milliseconds (total_time / object_count)
- **Chart Type**: Line plot by grid size
- **Interpretation**:
  - Flat line = linear scaling (each object adds constant time)
  - Rising line = super-linear scaling (efficiency decreases with size)
  - Falling line = sub-linear scaling (efficiency improves with size)
  - Different colored lines show how grid size affects per-object efficiency

### 3. Spillage Impact Analysis Chart (3-panel comparison)

**Location**: Bottom section of report  
**Format**: 1x3 horizontal layout  
**Purpose**: Analyze performance impact of spillage modeling

#### Panel 1: Execution Time by Spillage Mode (Left)
- **X-axis**: Spillage enabled (True/False)
- **Y-axis**: Execution time in milliseconds
- **Chart Type**: Box plot comparison
- **Interpretation**:
  - Compares performance distributions with and without spillage
  - Higher box for "True" = spillage adds computational overhead
  - Box width shows variability - spillage may make performance less predictable

#### Panel 2: Memory Usage by Spillage Mode (Center)
- **X-axis**: Spillage enabled (True/False)  
- **Y-axis**: Memory delta in megabytes
- **Chart Type**: Box plot comparison
- **Interpretation**:
  - Shows memory overhead of spillage tracking
  - Spillage requires additional data structures for affected cells
  - Higher memory usage expected with spillage enabled

#### Panel 3: Performance Distribution Comparison (Right)
- **X-axis**: Execution time in milliseconds
- **Y-axis**: Frequency
- **Chart Type**: Overlapping histograms
- **Colors**: Light blue (without spillage), light coral (with spillage)
- **Interpretation**:
  - Shows complete distribution shapes, not just summary statistics
  - Shift to right (coral histogram) indicates spillage slowdown
  - Overlap shows performance ranges where spillage impact is minimal

## Operation Performance Comparison Table

**Purpose**: Detailed statistical comparison of different benchmark operations  
**Location**: Below charts in tabular format

### Table Columns Explained:

#### Operation Name
- **Pure Env Update 25X25**: Environment update benchmark on 25x25 grid
- **Full Calculation 25X25**: Complete field calculation from scratch on 25x25 grid  
- **Spillage Enabled 25X25**: Full calculation with spillage modeling
- **Spillage Disabled 25X25**: Full calculation without spillage modeling
- **Strategic Analysis 25X25**: Strategy planning and analysis phase

#### Statistical Metrics:
- **Mean (ms)**: Average execution time across all test runs
- **Std (ms)**: Standard deviation - measure of performance consistency
- **Min (ms)**: Fastest recorded execution time
- **Max (ms)**: Slowest recorded execution time
- **Samples**: Number of test runs performed
- **95% CI**: 95% confidence interval for the mean

### Key Performance Expectations:

1. **Pure Env Update < Full Calculation**: Environment updates should be much faster than full calculations
2. **Spillage Enabled > Spillage Disabled**: Spillage modeling adds computational overhead
3. **Lower Standard Deviation = More Consistent**: Predictable performance is desirable

## Benchmark Type Differences: Quick vs Comprehensive

### Quick Test Configuration:
- **Grid Sizes**: 20x20, 25x25 (2 sizes)
- **Object Counts**: 35, 55 (2 densities)  
- **Seeds**: 31, 42 (2 random scenarios)
- **Iterations**: 5 per test
- **Focus**: Fast turnaround for development iteration
- **Environment Update**: Tests spillage=True only

### Comprehensive Test Configuration:
- **Grid Sizes**: 15x15, 20x20, 25x25, 30x30, 35x35 (5 sizes)
- **Object Counts**: 20, 35, 55, 75, 100 (5 densities)
- **Seeds**: 31, 42, 123, 456, 789 (5 random scenarios)  
- **Iterations**: 10 per test (20 for environment updates)
- **Focus**: Statistical significance and scaling analysis
- **Environment Update**: Tests multiple spillage configurations

## Spillage Configuration Details

### "Pure Env Update 25X25" Spillage Setting:
- **Default**: spillage=True (line 119 in benchmark_config.py)
- **Rationale**: Environment updates are most relevant when spillage tracking is active
- **Impact**: Measures the cost of updating affected_cells and spillage_affected_cells

### Spillage Comparison Operations:
- **"Spillage Enabled"**: use_spillage=True - tracks spillage effects and propagation
- **"Spillage Disabled"**: use_spillage=False - simpler calculation without spillage modeling
- **"Full Calculation"**: Default spillage setting (typically True)

## Statistical Analysis Features

### Confidence Intervals:
- **95% CI**: Range where true mean is likely to fall
- **Calculation**: mean ± (t-statistic × standard_error)
- **Interpretation**: Narrower intervals = more precise measurements

### Significance Testing:
- **Purpose**: Determines if performance differences are statistically meaningful
- **Method**: t-tests comparing operation means
- **Result**: p-values indicating confidence in observed differences

### Regression Analysis:
- **Object Count Scaling**: Fits polynomial models to predict complexity
- **Grid Size Scaling**: Analyzes quadratic growth patterns
- **R² Values**: Measure how well models predict performance

## Performance Optimization Insights

### Reading the Charts for Optimization:

1. **Identify Bottlenecks**: Operations with highest mean times in bar charts
2. **Find Inconsistencies**: High standard deviation or wide box plots
3. **Assess Scalability**: Non-linear trends in scaling charts
4. **Compare Alternatives**: Spillage enabled vs disabled trade-offs
5. **Validate Changes**: Before/after comparisons using confidence intervals

### Performance Targets:

- **Pure Env Update**: Should be <10% of Full Calculation time
- **Spillage Overhead**: Should be <50% performance penalty
- **Scalability**: Near-linear growth with object count, quadratic with grid size
- **Consistency**: Standard deviation <20% of mean execution time

This documentation provides the complete framework for interpreting every aspect of the benchmark report, enabling data-driven performance optimization decisions.