# 2D Algorithm Simulator

Two complete editions of the interactive material-moving planner:

- **exe/**: Windows x64 standalone edition built November 15, 2026. Download/clone the complete folder and double-click **2D Algorithm Simulator.exe**. Keep `_internal`, `data` and licenses beside it. No installed Python required.
- **python/**: Python source, planner, scenarios, offline guides and regression evidence. See its README for installation and launch instructions.

Both open the prepared 25 x 25, 55-object seminar scene. Select a pile, choose T (target) or H (staging), then Enter. U undoes; S saves; F1/F2 open guides.

The executable is the frozen November 15 release. Python files are the November 18 handoff snapshot. BUILD_INFO.json and SOURCE_MANIFEST.json record provenance.
The standalone build passed isolation checks on the development machine; a separate clean Windows PC was not tested.

After an action, target searches update selectively; derived heat and staging rebuild globally. The UI also performs a full verification calculation, so its elapsed time is not a selective-update benchmark. Heat is a planning score, and spillage is an approximate estimator rather than calibrated physics.

User saves and unrelated project experiment archives are excluded. Existing repository code and historical results are unchanged.
