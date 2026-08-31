"""Run the project tests when pytest is unavailable (as on the simulator PC)."""

from __future__ import annotations

import importlib.util
import inspect
from pathlib import Path
import tempfile
import traceback


def main():
    root = Path(__file__).resolve().parent
    passed = failed = 0
    for path in sorted(root.glob("test_*.py")):
        spec = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except ModuleNotFoundError as exc:
            print(f"SKIP {path.name}: optional dependency {exc.name!r} is unavailable")
            continue
        for name, function in inspect.getmembers(module, inspect.isfunction):
            if not name.startswith("test_"):
                continue
            signature = inspect.signature(function)
            try:
                if not signature.parameters:
                    function()
                elif tuple(signature.parameters) == ("tmp_path",):
                    with tempfile.TemporaryDirectory() as temporary:
                        function(Path(temporary))
                else:
                    print(f"SKIP {path.name}::{name}: unsupported fixtures {tuple(signature.parameters)}")
                    continue
                passed += 1
                print(f"PASS {path.name}::{name}")
            except Exception:
                failed += 1
                print(f"FAIL {path.name}::{name}")
                traceback.print_exc()
    print(f"\n{passed} passed, {failed} failed")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
