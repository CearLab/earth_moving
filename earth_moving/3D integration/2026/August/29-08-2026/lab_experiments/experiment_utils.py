"""Shared, standard-library-heavy helpers for the lab experiment runner."""

from __future__ import annotations

import csv
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping

import yaml


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "experiment_config.yaml"


def load_config(path: str | Path = DEFAULT_CONFIG) -> tuple[dict, Path]:
    config_path = Path(path).resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"configuration must be a mapping: {config_path}")
    return config, config_path


def resolve_from_config(config_path: Path, value: str | Path) -> Path:
    path = Path(value)
    return path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()


def config_hash(config: Mapping[str, Any]) -> str:
    encoded = json.dumps(config, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def stable_json_hash(value: Any) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def atomic_write_text(path: str | Path, text: str) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", newline="", delete=False,
        dir=str(destination.parent), prefix=destination.name + ".", suffix=".tmp",
    ) as stream:
        stream.write(text)
        temporary = Path(stream.name)
    os.replace(temporary, destination)
    return destination


def atomic_write_json(path: str | Path, value: Any) -> Path:
    return atomic_write_text(path, json.dumps(value, indent=2, sort_keys=True) + "\n")


def atomic_write_csv(
    path: str | Path,
    rows: Iterable[Mapping[str, Any]],
    fieldnames: Iterable[str],
) -> Path:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(fieldnames)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8-sig", newline="", delete=False,
        dir=str(destination.parent), prefix=destination.name + ".", suffix=".tmp",
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name) for name in fieldnames})
        temporary = Path(stream.name)
    os.replace(temporary, destination)
    return destination


def read_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def read_latest_run(log_dir: str | Path) -> dict[str, str]:
    latest = Path(log_dir) / "LATEST_RUN.txt"
    values: dict[str, str] = {}
    if not latest.exists():
        return values
    for line in latest.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    return values


def read_json(path: str | Path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return default


REQUIRED_OUTPUT_KEYS = (
    "JSONL", "TASKS_CSV", "PUSHES_CSV", "SNAPSHOTS_CSV",
    "RUN_SUMMARY_JSON", "ARTIFACT_DIR",
)


def validate_run_outputs(run_dir: str | Path) -> tuple[bool, list[str], dict[str, str], dict]:
    latest = read_latest_run(run_dir)
    missing = [
        key for key in REQUIRED_OUTPUT_KEYS
        if not latest.get(key) or not Path(latest[key]).exists()
    ]
    summary = read_json(latest.get("RUN_SUMMARY_JSON", ""), default={}) or {}
    if not summary:
        missing.append("readable_RUN_SUMMARY_JSON")
    return not missing, sorted(set(missing)), latest, summary

