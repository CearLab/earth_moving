from pathlib import Path
import tempfile

from benchmark_telemetry import BenchmarkTelemetry
from simulation_event_logger import SimulationEventLogger


with tempfile.TemporaryDirectory() as temporary:
    # Reproduce the notebook's very long OneDrive experiment path. The compact
    # benchmark names must remain below the legacy Windows MAX_PATH boundary.
    long_log_dir = Path(temporary) / ("long_experiment_directory_" * 5)
    logger = SimulationEventLogger(long_log_dir)
    telemetry = BenchmarkTelemetry(
        logger.log_dir,
        logger.run_id,
        event_callback=lambda event_type, sim_time, **data: logger.event(
            event_type, sim_time, data.pop("agent_id", None), **data
        ),
    )
    print("benchmark initialized:", telemetry.config_dict())
    for output in (
        telemetry.pushes_path, telemetry.snapshots_path,
        telemetry.run_summary_path, telemetry.data_dir / "00001.json.gz",
    ):
        print("path length:", len(str(output)), output)
        assert len(str(output)) < 260
    telemetry.close(0.0, "diagnostic", {})
    logger.close(0.0, "diagnostic")
