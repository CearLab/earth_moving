"""Check imports and record the host/dependency context without starting physics."""

from __future__ import annotations

from datetime import datetime, timezone
import importlib
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

from experiment_utils import DEFAULT_CONFIG, atomic_write_json, load_config, resolve_from_config


REQUIRED_IMPORTS = (
    "numpy", "scipy", "matplotlib", "pygame", "pybullet", "shapely", "yaml", "psutil",
)


def main():
    config, config_path = load_config(DEFAULT_CONFIG)
    failures = {}
    versions = {}
    for name in REQUIRED_IMPORTS:
        try:
            module = importlib.import_module(name)
            versions[name] = getattr(module, "__version__", "available")
        except Exception as exc:
            failures[name] = repr(exc)
    result_root = resolve_from_config(config_path, config["results_root"])
    disk = shutil.disk_usage(result_root.parent)
    try:
        freeze = subprocess.check_output(
            [sys.executable, "-m", "pip", "freeze"], text=True, timeout=60,
        ).splitlines()
    except Exception:
        freeze = []
    payload = {
        "schema": 1,
        "captured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "hostname": platform.node(), "platform": platform.platform(),
        "machine": platform.machine(), "processor": platform.processor(),
        "python": sys.version, "python_executable": sys.executable,
        "logical_cpu_count": os.cpu_count(),
        "disk_total_gib": disk.total / 2**30,
        "disk_free_gib": disk.free / 2**30,
        "module_versions": versions,
        "import_failures": failures,
        "pip_freeze": freeze,
    }
    output = result_root / "host_environment.json"
    atomic_write_json(output, payload)
    if failures:
        raise SystemExit(f"SERVER ENVIRONMENT FAIL: {failures}")
    print(f"SERVER ENVIRONMENT PASS: {os.cpu_count()} logical CPUs, {disk.free / 2**30:.1f} GiB free")
    print(f"Recorded: {output}")


if __name__ == "__main__":
    main()
