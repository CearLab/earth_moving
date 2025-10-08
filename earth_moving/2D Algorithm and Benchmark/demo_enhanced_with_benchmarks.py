"""
Enhanced Demo with Integrated Benchmarking System
Complete strategic infrastructure with performance evaluation capabilities
"""

import time
import json
from pathlib import Path
from typing import Optional, Dict, Any

class StateJSONEncoder(json.JSONEncoder):
    """Custom JSON encoder for environment state serialization"""
    def default(self, obj):
        if hasattr(obj, '__dict__'):
            return obj.__dict__
        elif isinstance(obj, tuple):
            return list(obj)
        elif hasattr(obj, 'wkt'):  # Shapely geometry objects
            return obj.wkt
        return super().default(obj)

def convert_tuple_keys_for_json(obj):
    """Convert tuple keys to strings for JSON serialization"""
    if isinstance(obj, dict):
        new_dict = {}
        for key, value in obj.items():
            if isinstance(key, tuple):
                # Convert tuple key to string like "(x,y)"
                str_key = f"({key[0]},{key[1]})"
            else:
                str_key = str(key)
            new_dict[str_key] = convert_tuple_keys_for_json(value)
        return new_dict
    elif isinstance(obj, list):
        return [convert_tuple_keys_for_json(item) for item in obj]
    else:
        return obj

def convert_string_keys_to_tuples(obj):
    """Convert string keys back to tuples after JSON deserialization"""
    if isinstance(obj, dict):
        new_dict = {}
        for key, value in obj.items():
            # Check if key looks like a tuple string "(x,y)"
            if isinstance(key, str) and key.startswith('(') and key.endswith(')') and ',' in key:
                try:
                    # Parse "(x,y)" back to tuple
                    coords = key[1:-1].split(',')
                    if len(coords) == 2:
                        tuple_key = (int(coords[0]), int(coords[1]))
                    else:
                        tuple_key = key  # Keep as string if parsing fails
                except (ValueError, IndexError):
                    tuple_key = key  # Keep as string if parsing fails
            else:
                tuple_key = key
            new_dict[tuple_key] = convert_string_keys_to_tuples(value)
        return new_dict
    elif isinstance(obj, list):
        return [convert_string_keys_to_tuples(item) for item in obj]
    else:
        return obj

# Import base demo and benchmarking components
from demo_interactive_comprehensive import ComprehensiveDemo
from benchmark_config import BenchmarkConfig, BenchmarkType, StrategyType
from benchmark_runner import BenchmarkRunner, BenchmarkRunConfig
from benchmark_analyzer_lite import BenchmarkAnalyzerLite
# Try to import report generator, fallback if dependencies missing
try:
    from benchmark_report_generator import BenchmarkReportGenerator
    HAS_REPORT_GENERATOR = True
except ImportError as e:
    print(f"Report generator not available (missing dependencies): {e}")
    HAS_REPORT_GENERATOR = False
    BenchmarkReportGenerator = None
from benchmark_performance_tracker import PerformanceTracker

class EnhancedDemoWithBenchmarks(ComprehensiveDemo):
    """Enhanced strategic demo with integrated benchmarking capabilities"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize benchmarking components
        self.benchmark_config = BenchmarkConfig()
        self.performance_tracker = PerformanceTracker()
        self.benchmark_analyzer = BenchmarkAnalyzerLite()
        self.report_generator = BenchmarkReportGenerator() if HAS_REPORT_GENERATOR else None
        
        # Create benchmark results directory
        self.benchmark_results_dir = Path("benchmark_results")
        self.benchmark_results_dir.mkdir(exist_ok=True)
        
        print("Enhanced Demo with Benchmarking System initialized!")
        print(f"Benchmark results will be saved to: {self.benchmark_results_dir}")
    
    def run_quick_performance_test(self):
        """Run a quick performance test of current configuration"""
        if not self.env:
            print("No environment loaded. Please create a simulation first.")
            return
        
        print("\n=== QUICK PERFORMANCE TEST ===")
        print("Testing current environment configuration...")
        
        # Test different operations
        test_results = {}
        
        # 1. Test full calculation
        print("Testing full strategic calculation...")
        with self.performance_tracker.measure_operation("full_calculation_test"):
            self.calculate_strategic_fields_now()
        
        latest_result = self.performance_tracker._current_metrics[-1] if hasattr(self.performance_tracker, '_current_metrics') and self.performance_tracker._current_metrics else None
        if latest_result:
            test_results['full_calculation'] = {
                'time_ms': latest_result.execution_time_ms,
                'memory_mb': latest_result.memory_delta_mb
            }
        
        # 2. Test strategic analysis
        print("Testing strategic analysis...")
        with self.performance_tracker.measure_operation("analysis_test"):
            self.analyze_strategic_situation()
        
        if hasattr(self.performance_tracker, '_current_metrics') and len(self.performance_tracker._current_metrics) >= 2:
            latest_result = self.performance_tracker._current_metrics[-1]
            test_results['strategic_analysis'] = {
                'time_ms': latest_result.execution_time_ms,
                'memory_mb': latest_result.memory_delta_mb
            }
        
        # 3. Test save/load performance
        print("Testing save/load performance...")
        test_save_name = "performance_test_temp"
        
        with self.performance_tracker.measure_operation("save_test"):
            self.save_current_state(test_save_name)
        
        with self.performance_tracker.measure_operation("load_test"):
            self.load_saved_state(test_save_name)
        
        # Display results
        print("\n=== QUICK TEST RESULTS ===")
        for operation, metrics in test_results.items():
            print(f"{operation.replace('_', ' ').title()}:")
            print(f"  Time: {metrics['time_ms']:.2f}ms")
            print(f"  Memory: {metrics['memory_mb']:.2f}MB")
        
        # Cleanup temp save
        temp_save_dir = self.storage_path / test_save_name
        if temp_save_dir.exists():
            import shutil
            shutil.rmtree(temp_save_dir)
        
        print("\nQuick performance test completed!")
    
    def run_comprehensive_benchmark_suite(self):
        """Run comprehensive benchmark suite"""
        print("\n=== COMPREHENSIVE BENCHMARK SUITE ===")
        print("This will run extensive performance tests with multiple configurations.")
        print("Estimated time: 5-15 minutes depending on system performance.")
        
        choice = input("\nProceed with comprehensive benchmarks? (y/N): ").strip().lower()
        if choice != 'y':
            print("Benchmark cancelled.")
            return
        
        # Configure benchmark runner
        run_config = BenchmarkRunConfig(
            output_directory=str(self.benchmark_results_dir),
            parallel_execution=False,  # Keep sequential for stability
            verbose=True,
            create_summary_report=True,
            continue_on_error=True
        )
        
        runner = BenchmarkRunner(run_config)
        
        print("\nSelect benchmark configuration:")
        print("1. Quick test (faster, fewer scenarios)")
        print("2. Comprehensive test (slower, extensive coverage)")
        print("3. Custom test (select specific benchmark types)")
        
        config_choice = input("Enter choice (1-3): ").strip()
        
        if config_choice == "1":
            config_type = "quick"
            specific_types = None
        elif config_choice == "2":
            config_type = "comprehensive" 
            specific_types = None
        elif config_choice == "3":
            config_type = "comprehensive"
            specific_types = self._select_benchmark_types()
        else:
            print("Invalid choice. Using quick configuration.")
            config_type = "quick"
            specific_types = None
        
        print(f"\nStarting {config_type} benchmark suite...")
        start_time = time.time()
        
        try:
            # Run benchmarks
            results = runner.run_benchmark_suite(config_type, specific_types)
            
            execution_time = time.time() - start_time
            print(f"\n=== BENCHMARK SUITE COMPLETED ===")
            print(f"Total execution time: {execution_time:.1f} seconds")
            print(f"Results saved to: {runner.output_dir}")
            
            # Offer to generate report
            generate_report = input("\nGenerate comprehensive HTML report? (Y/n): ").strip().lower()
            if generate_report != 'n':
                self._generate_benchmark_report(runner.output_dir)
            
        except Exception as e:
            print(f"Benchmark suite failed: {e}")
            import traceback
            traceback.print_exc()
    
    def _select_benchmark_types(self) -> list:
        """Allow user to select specific benchmark types"""
        print("\nAvailable benchmark types:")
        benchmark_types = list(BenchmarkType)
        
        for i, bench_type in enumerate(benchmark_types, 1):
            print(f"{i}. {bench_type.value.replace('_', ' ').title()}")
        
        print("Enter benchmark type numbers separated by commas (e.g., 1,3,5):")
        selection = input("Selection: ").strip()
        
        if not selection:
            return None
        
        try:
            indices = [int(x.strip()) - 1 for x in selection.split(',')]
            selected_types = [benchmark_types[i] for i in indices if 0 <= i < len(benchmark_types)]
            
            print(f"Selected: {', '.join(t.value for t in selected_types)}")
            return selected_types
            
        except (ValueError, IndexError):
            print("Invalid selection. Using all benchmark types.")
            return None
    
    def _generate_benchmark_report(self, results_dir: Path):
        """Generate comprehensive benchmark report"""
        # Find the most recent results file
        results_files = list(results_dir.glob("benchmark_detailed_*.json"))
        if not results_files:
            print("No benchmark results found for report generation.")
            return
        
        latest_results = max(results_files, key=lambda p: p.stat().st_mtime)
        
        print(f"Generating report from: {latest_results.name}")
        
        try:
            if self.report_generator:
                report_path = self.report_generator.generate_html_report(
                    str(latest_results),
                    report_title="Earth Moving Algorithm Performance Analysis"
                )
                
                print(f"Comprehensive HTML report generated: {report_path}")
                
                # Ask if user wants to open the report
                open_report = input("Open report in default browser? (Y/n): ").strip().lower()
                if open_report != 'n':
                    import webbrowser
                    webbrowser.open(f"file://{Path(report_path).absolute()}")
            else:
                print("HTML report generation not available (missing dependencies)")
                print("Install with: pip install matplotlib seaborn pandas")
                
        except Exception as e:
            print(f"Report generation failed: {e}")
    
    def analyze_existing_benchmarks(self):
        """Analyze existing benchmark results"""
        print("\n=== BENCHMARK ANALYSIS ===")
        
        # Find available results
        results_files = list(self.benchmark_results_dir.glob("benchmark_detailed_*.json"))
        
        if not results_files:
            print("No benchmark results found.")
            print(f"Run benchmarks first or check directory: {self.benchmark_results_dir}")
            return
        
        print("Available benchmark results:")
        for i, file_path in enumerate(results_files, 1):
            file_time = time.ctime(file_path.stat().st_mtime)
            print(f"{i}. {file_path.name} ({file_time})")
        
        try:
            choice = int(input("\nSelect results file (number): ").strip()) - 1
            if 0 <= choice < len(results_files):
                selected_file = results_files[choice]
                
                print(f"\nAnalyzing: {selected_file.name}")
                
                # Load and analyze
                self.analyzer.load_benchmark_results(str(selected_file))
                results_key = selected_file.stem
                
                # Print summary
                self.analyzer.print_analysis_summary(results_key)
                
                # Offer detailed report
                detailed_choice = input("\nGenerate detailed analysis report? (Y/n): ").strip().lower()
                if detailed_choice != 'n':
                    report = self.analyzer.generate_performance_report(results_key)
                    print("Detailed analysis report generated.")
                
                # Offer visualizations
                viz_choice = input("Generate performance visualizations? (Y/n): ").strip().lower()
                if viz_choice != 'n':
                    plots = self.analyzer.create_performance_visualizations(results_key)
                    print("Performance visualizations generated.")
                
            else:
                print("Invalid selection.")
                
        except (ValueError, IndexError):
            print("Invalid input.")
    
    def compare_benchmark_results(self):
        """Compare multiple benchmark results"""
        print("\n=== BENCHMARK COMPARISON ===")
        
        results_files = list(self.benchmark_results_dir.glob("benchmark_detailed_*.json"))
        
        if len(results_files) < 2:
            print("Need at least 2 benchmark results for comparison.")
            return
        
        print("Available results for comparison:")
        for i, file_path in enumerate(results_files, 1):
            file_time = time.ctime(file_path.stat().st_mtime)
            print(f"{i}. {file_path.name} ({file_time})")
        
        print("\nSelect 2 results to compare (enter two numbers separated by space):")
        try:
            choices = [int(x) - 1 for x in input("Selection: ").strip().split()]
            
            if len(choices) != 2 or not all(0 <= c < len(results_files) for c in choices):
                print("Invalid selection.")
                return
            
            file1, file2 = results_files[choices[0]], results_files[choices[1]]
            
            print(f"\nComparing:")
            print(f"  A: {file1.name}")
            print(f"  B: {file2.name}")
            
            # Load both results
            analyzer1 = BenchmarkAnalyzerLite()
            analyzer2 = BenchmarkAnalyzerLite()
            
            data1 = analyzer1.load_benchmark_results(str(file1))
            data2 = analyzer2.load_benchmark_results(str(file2))
            
            df1 = analyzer1.extract_performance_data(data1)
            df2 = analyzer2.extract_performance_data(data2)
            
            if df1.empty or df2.empty:
                print("No comparable data found in results.")
                return
            
            # Compare key metrics
            print("\n=== PERFORMANCE COMPARISON ===")
            
            metrics = ['execution_time_ms', 'memory_delta_mb']
            
            for metric in metrics:
                if metric in df1.columns and metric in df2.columns:
                    stats1 = analyzer1.calculate_statistical_summary(df1[metric].values)
                    stats2 = analyzer2.calculate_statistical_summary(df2[metric].values)
                    
                    comparison = analyzer1.compare_groups(
                        df1[metric].values, df2[metric].values,
                        file1.stem, file2.stem
                    )
                    
                    print(f"\n{metric.replace('_', ' ').title()}:")
                    print(f"  {file1.stem[:20]}: {stats1.mean:.2f} ± {stats1.std:.2f}")
                    print(f"  {file2.stem[:20]}: {stats2.mean:.2f} ± {stats2.std:.2f}")
                    print(f"  Significance: {comparison.interpretation}")
            
        except (ValueError, IndexError):
            print("Invalid input format.")
    
    def benchmark_current_scenario(self):
        """Benchmark the current loaded scenario"""
        if not self.env:
            print("No environment loaded. Please create a simulation first.")
            return
        
        print("\n=== CURRENT SCENARIO BENCHMARK ===")
        print(f"Environment: {self.env.grid_size}x{self.env.grid_size}")
        print(f"Objects: {sum(cell.num_objects for cell in self.env.cells_with_objects)}")
        print(f"Strategy: {self.selected_strategy.value}")
        print(f"Spillage: {'ON' if self.use_spillage_model else 'OFF'}")
        
        iterations = input("\nNumber of iterations for benchmark (default 5): ").strip()
        try:
            iterations = int(iterations) if iterations else 5
        except ValueError:
            iterations = 5
        
        print(f"\nRunning {iterations} iterations...")
        
        # Run benchmark on current scenario
        def current_scenario_operation():
            # Force recalculation from scratch
            if hasattr(self, 'recalculate_everything_from_scratch'):
                self.recalculate_everything_from_scratch()
            else:
                self.calculate_strategic_fields_now()
            
            return {
                'grid_size': self.env.grid_size,
                'object_count': sum(cell.num_objects for cell in self.env.cells_with_objects),
                'strategy': self.selected_strategy.value,
                'spillage': self.use_spillage_model
            }
        
        result = self.performance_tracker.run_benchmark(
            operation_name="current_scenario_benchmark",
            operation_func=current_scenario_operation,
            iterations=iterations,
            scenario_id=f"user_scenario_{int(time.time())}",
            grid_size=self.env.grid_size,
            object_count=sum(cell.num_objects for cell in self.env.cells_with_objects),
            strategy=self.selected_strategy.value,
            use_spillage=self.use_spillage_model
        )
        
        # Display results
        print(f"\n=== BENCHMARK RESULTS ===")
        print(f"Average Time: {result.average_time_ms:.2f}ms")
        print(f"Standard Deviation: {result.std_deviation * 1000:.2f}ms")
        print(f"Time Range: {result.min_time * 1000:.2f}ms - {result.max_time * 1000:.2f}ms")
        print(f"Average Memory Delta: {result.average_memory_delta:.2f}MB")
        print(f"Peak Memory Usage: {result.peak_memory_usage:.2f}MB")
        
        # Save individual result
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        result_file = self.benchmark_results_dir / f"scenario_benchmark_{timestamp}.json"
        
        export_data = self.performance_tracker.export_results(str(result_file))
        print(f"\nResults saved to: {result_file}")
    
    def save_current_state(self, save_name: str):
        """Save current environment state"""
        if not self.env:
            print("No environment to save.")
            return
        
        try:
            save_path = self.storage_path / save_name
            save_path.mkdir(exist_ok=True)
            
            # Get full environment state with all calculations
            full_state = self.env.get_state()
            
            # Convert tuple keys to strings for JSON serialization
            json_compatible_state = convert_tuple_keys_for_json(full_state)
            
            # Save complete state with all calculated fields
            with open(save_path / "environment_state.json", 'w') as f:
                json.dump(json_compatible_state, f, indent=2, cls=StateJSONEncoder)
            
            # Save demo configuration
            config = {
                'selected_strategy': self.selected_strategy.value,
                'use_spillage_model': self.use_spillage_model,
                'current_scenario_id': self.current_scenario_id
            }
            with open(save_path / "demo_config.json", 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"State saved to: {save_path}")
            
        except Exception as e:
            print(f"Failed to save state: {e}")
    
    def save_state_with_timestamp(self):
        """Save current environment state with user-provided name and timestamp"""
        if not self.env:
            print("No environment to save.")
            return
        
        # Ask user for save name
        user_name = input("Enter save name: ").strip()
        if not user_name:
            print("Save name required.")
            return
        
        # Add timestamp to save name
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_name = f"{user_name}_{timestamp}"
        
        print(f"Saving as: {save_name}")
        self.save_current_state(save_name)
    
    def load_state_by_selection(self):
        """Load saved state by numbered selection"""
        if not self.storage_path.exists():
            print("No saved states found.")
            return
        
        saves = [d for d in self.storage_path.iterdir() if d.is_dir()]
        
        if not saves:
            print("No saved states found.")
            return
        
        print("\nAvailable saved states:")
        valid_saves = []
        for i, save_dir in enumerate(saves, 1):
            state_file = save_dir / "environment_state.json"
            if state_file.exists():
                import time
                mod_time = time.ctime(state_file.stat().st_mtime)
                print(f"{i}. {save_dir.name} (saved: {mod_time})")
                valid_saves.append(save_dir.name)
            else:
                print(f"{i}. {save_dir.name} (incomplete save - skipping)")
        
        if not valid_saves:
            print("No valid saved states found.")
            return
        
        try:
            choice = input(f"\nEnter number (1-{len(valid_saves)}) or press Enter to cancel: ").strip()
            if not choice:
                print("Load cancelled.")
                return
            
            choice_num = int(choice)
            if 1 <= choice_num <= len(valid_saves):
                selected_save = valid_saves[choice_num - 1]
                print(f"Loading: {selected_save}")
                self.load_saved_state(selected_save)
            else:
                print(f"Invalid selection. Please choose 1-{len(valid_saves)}")
                
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    def load_saved_state(self, save_name: str):
        """Load previously saved environment state"""
        try:
            save_path = self.storage_path / save_name
            
            if not save_path.exists():
                print(f"Save file not found: {save_path}")
                return
            
            # Load environment state
            state_file = save_path / "environment_state.json"
            config_file = save_path / "demo_config.json"
            
            if not state_file.exists():
                print("Environment state file not found.")
                return
            
            with open(state_file, 'r') as f:
                json_state = json.load(f)
            
            # Convert string keys back to tuples
            state = convert_string_keys_to_tuples(json_state)
            
            # Create new environment with parameters from state
            from core_env import SimulationEnv
            self.env = SimulationEnv(
                grid_size=state.get('grid_size', 25),
                target_zone_radius=state.get('target_zone_radius', 10),
                agent_positions=None,
                num_random_objects=0,  # Will be restored from state
                seed=None
            )
            
            # Restore complete environment state with all calculated fields
            self.env.set_state(state)
            
            print(f"Environment restored with {len(self.env.cells_with_objects)} cells containing objects")
            print("All calculated fields (heat maps, paths, distances) have been restored")
            
            # Load demo configuration if available
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = json.load(f)
                
                # Restore configuration
                from strategic_strategy_planner import PlanningStrategy
                try:
                    self.selected_strategy = PlanningStrategy(config.get('selected_strategy', 'greedy_nearest'))
                    self.use_spillage_model = config.get('use_spillage_model', False)
                    self.current_scenario_id = config.get('current_scenario_id')
                except:
                    print("Warning: Could not restore all configuration settings")
            
            # Reinitialize visualizer
            try:
                from core_visualizer import SimulationVisualizer
                self.visualizer = SimulationVisualizer(self.env, screen_size=800)
            except:
                self.visualizer = None
            
            print(f"State loaded from: {save_path}")
            
        except Exception as e:
            print(f"Failed to load state: {e}")
    
    def list_saved_states(self):
        """List all saved states"""
        if not self.storage_path.exists():
            print("No saved states found.")
            return
        
        saves = [d for d in self.storage_path.iterdir() if d.is_dir()]
        
        if not saves:
            print("No saved states found.")
            return
        
        print("\nAvailable saved states:")
        for i, save_dir in enumerate(saves, 1):
            state_file = save_dir / "environment_state.json"
            if state_file.exists():
                import time
                mod_time = time.ctime(state_file.stat().st_mtime)
                print(f"{i}. {save_dir.name} (saved: {mod_time})")
            else:
                print(f"{i}. {save_dir.name} (incomplete save)")
    
    def calculate_strategic_fields_now(self):
        """Calculate strategic fields (alias for compatibility)"""
        if not self.env:
            print("No environment loaded.")
            return
        
        self.calculate_on_demand()
    
    def run_enhanced_menu(self):
        """Enhanced menu with benchmarking capabilities"""
        while True:
            print("\n" + "="*80)
            print("ENHANCED STRATEGIC DEMO WITH BENCHMARKING")
            print("="*80)
            print("Complete strategic infrastructure + performance evaluation system")
            
            if self.env:
                total_objects = sum(cell.num_objects for cell in self.env.cells_with_objects)
                fields_status = "Ready" if hasattr(self.env, '_fields_calculated') else "On-demand"
                print(f"\nEnvironment: {self.env.grid_size}x{self.env.grid_size}, {total_objects} objects")
                print(f"Strategy: {self.selected_strategy.value}")
                print(f"Spillage: {'ON' if self.use_spillage_model else 'OFF'}")
                print(f"Strategic fields: {fields_status}")
                if self.current_scenario_id:
                    print(f"Active scenario: {self.current_scenario_id[:8]}")
            else:
                print("\nNo environment loaded")
            
            print("\n--- SETUP ---")
            print("1. Create quick simulation (no spillage - fast)")
            print("2. Create simulation with spillage (slower)")
            
            print("\n--- OPERATE ---") 
            print("3. Start visual interface")
            print("4. Calculate strategic fields now")
            print("5. Analyze strategic situation")
            print("6. Recalculate everything from scratch")
            
            print("\n--- SAVE/LOAD ---")
            print("7. Save current state")
            print("8. Load saved state")
            print("9. List saved states")
            
            print("\n--- BENCHMARKING ---")
            print("10. Quick performance test")
            print("11. Benchmark current scenario")
            print("12. Run comprehensive benchmark suite")
            print("13. Analyze existing benchmarks")
            print("14. Compare benchmark results")
            
            print("\n--- EXIT ---")
            print("0. Exit")
            
            choice = input("\nEnter your choice: ").strip()
            
            try:
                if choice == "0":
                    print("Goodbye!")
                    break
                elif choice == "1":
                    self.create_simulation_quick(use_spillage=False)
                elif choice == "2":
                    self.create_simulation_quick(use_spillage=True)
                elif choice == "3":
                    if self.env and hasattr(self, 'visualizer') and self.visualizer:
                        print("Starting visual interface...")
                        self.run_visual_interface()
                    else:
                        print("Environment or visualizer not available")
                elif choice == "4":
                    self.calculate_strategic_fields_now()
                elif choice == "5":
                    self.analyze_strategic_situation()
                elif choice == "6":
                    if hasattr(self, 'recalculate_everything_from_scratch'):
                        self.recalculate_everything_from_scratch()
                    else:
                        print("Recalculation method not available in base demo")
                elif choice == "7":
                    if self.env:
                        self.save_state_with_timestamp()
                    else:
                        print("No environment to save")
                elif choice == "8":
                    self.load_state_by_selection()
                elif choice == "9":
                    self.list_saved_states()
                elif choice == "10":
                    self.run_quick_performance_test()
                elif choice == "11":
                    self.benchmark_current_scenario()
                elif choice == "12":
                    self.run_comprehensive_benchmark_suite()
                elif choice == "13":
                    self.analyze_existing_benchmarks()
                elif choice == "14":
                    self.compare_benchmark_results()
                else:
                    print("Invalid choice. Please enter a number from 0-14.")
                    
            except KeyboardInterrupt:
                print("\n\nOperation cancelled by user.")
            except Exception as e:
                print(f"Error: {e}")
                print("Continuing with menu...")

def main():
    """Main entry point"""
    demo = EnhancedDemoWithBenchmarks()
    
    print("=" * 80)
    print("EARTH MOVING ALGORITHM - ENHANCED DEMO WITH BENCHMARKING")
    print("=" * 80)
    print()
    print("Features:")
    print("• Complete strategic infrastructure (scenarios, history, orchestrator)")
    print("• Visual interface with real-time field visualization") 
    print("• Comprehensive benchmarking system")
    print("• Performance analysis and reporting")
    print("• Statistical comparison tools")
    print("• Professional HTML report generation")
    print()
    
    demo.run_enhanced_menu()

if __name__ == "__main__":
    main()