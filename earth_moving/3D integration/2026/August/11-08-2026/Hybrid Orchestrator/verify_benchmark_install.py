import ast
from pathlib import Path

names = [
    "benchmark_telemetry.py",
    "simulation_event_logger.py",
    "shared_path_allocator.py",
    "orchestrator_hybrid_multi_astar_scheduled.py",
    "analyze_benchmark_runs.py",
]
for name in names:
    ast.parse(Path(name).read_text(encoding="utf-8-sig"), filename=name)
print(f"AST parse passed for {len(names)} changed modules")
