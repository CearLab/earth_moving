# Python simulator

Requires Python 3.10 (the verified environment used 3.10.21).
From this folder:

```sh
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python thesis/scripts/interactive_scenario.py --scenario thesis/seminar/scenarios/seminar_live_55.json
```

After activating the environment, Windows users can also double-click `Start simulator.cmd`.
The original seminar launchers contain machine-specific environment paths; use the command above or the new launcher.
F1/F2 open the offline guides. Saves go to `thesis/seminar/interactive_saved`.

Run the UI/model regression smoke test with:

```sh
python thesis/scripts/interactive_scenario.py --self-test --test-output self_test_output
```

Core planner: `Hybrid Orchestrator/`. UI and scenario API: `thesis/scripts/`.
Scenarios, guides and demo cues: `thesis/seminar/`.
The selective regression script writes its historical validation output; preserve that record before running it.
This is a distribution snapshot of the active repaired planner, not a separate development branch of the algorithm.
