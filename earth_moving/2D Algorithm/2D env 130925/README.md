# Earth Moving Algorithm - Clean Flat Structure

This directory contains the complete earth moving algorithm with clean file naming and no redundant files.

## File Organization

### Core Algorithm (core_*.py)
- `core_cell.py` - Grid cell implementation
- `core_env.py` - Main environment and field calculations  
- `core_spillage_model.py` - Spillage physics simulation
- `core_search.py` - A* pathfinding algorithms
- `core_agents.py` - Agent behavior and movement
- `core_visualizer.py` - Visualization rendering

### Strategic Infrastructure (strategic_*.py)
- `strategic_scenario_manager.py` - Multi-scenario management
- `strategic_orchestrator.py` - High-level strategic coordination
- `strategic_strategy_planner.py` - Strategy selection and planning  
- `strategic_move_history.py` - Move tracking and history

### Benchmarking System (benchmark_*.py)
- `benchmark_config.py` - Test scenario configurations
- `benchmark_runner.py` - Automated benchmark execution
- `benchmark_performance_tracker.py` - High-precision timing and metrics
- `benchmark_analyzer.py` - Statistical analysis of results
- `benchmark_analyzer_lite.py` - Lightweight analysis version
- `benchmark_report_generator.py` - HTML report generation

### Demo Applications (demo_*.py)
- `demo_interactive_comprehensive.py` - **Main demo** with all features: visual, analysis, benchmarking
- `demo_enhanced_with_benchmarks.py` - Advanced benchmarking integration

### Interface Support (interface_*.py)
- `interface_real_sim.py` - Real simulation integration interface

### Documentation (docs_*.md)
- `docs_algorithm.md` - Algorithm documentation
- `docs_benchmark_report_guide.md` - Complete benchmark report guide
- `docs_benchmarking_analysis.md` - Benchmarking analysis
- `docs_strategic_infrastructure.md` - Strategic infrastructure docs

## Quick Start

**Main demo (recommended):**
```bash
python demo_interactive_comprehensive.py
```

**Advanced benchmarking:**
```bash
python demo_enhanced_with_benchmarks.py
```

## Key Features

- ✅ **Clean naming** - All files follow consistent `prefix_name.py` pattern
- ✅ **No redundancy** - Eliminated duplicate demo files 
- ✅ **Single main entry** - `demo_interactive_comprehensive.py` has all features
- ✅ **Proper imports** - All import paths fixed for flat structure
- ✅ **Complete functionality** - All fixes and optimizations preserved

## Recent Improvements

1. **Fixed naming consistency:**
   - `real_sim_interface.py` → `interface_real_sim.py`
   - `performance_tracker.py` → `benchmark_performance_tracker.py`
   - `demo_fully_fixed.py` → `demo_interactive_comprehensive.py`

2. **Eliminated redundant files:**
   - Removed `demo_visual_strategic.py`, `demo_quick_visual.py`, `demo_main.py`
   - Consolidated all functionality into `demo_interactive_comprehensive.py`

3. **All visualization working:**
   - Fixed visualizer import issues
   - Complete strategic interface with save/load, analysis, benchmarking

The system is now clean, organized, and fully functional! 🎯