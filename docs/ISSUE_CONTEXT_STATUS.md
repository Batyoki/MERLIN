# Status of the items in `issue_context.md`

✅ fixed on branch `claude/brave-ride-bizkpi` (PR #1) · 🟡 partly done / verified only in simulation · ❌ still to do.
"How" points to `docs/CODE_FIXES.md` where a fix is already described.

## Control and torque (fixed)

| Item | Status | What was done / what remains |
|---|---|---|
| Torque: master torque read, passed through the loop, output "too rough or no change at all" | ✅ 🟡 | Root causes found and fixed (CODE_FIXES A1–A13): mA treated as A; master fed its own current back (positive feedback); 25–75 mA current cap below gravity; gravity at the wrong shoulder angle; joint-index bug in slave gravity; estimator/feedback tuning. Torque mimicking now works in simulation (`docs/TORQUE_STUDIES.md` S2: correlation 0.94–0.98). **Remaining:** run on the arms with the checklist at the end of CODE_FIXES. |
| Control: "Start Control — don't know what algorithm it runs or which topic it publishes to" | ❌ (verified) | It runs **nothing**. `gui.py:1702` only shows a message box ("Publishing to /arm_bench/{mode}_command") followed by `# TODO: Integrate with ROS`. |
| Control: "Load Control Config" | ❌ (verified) | Also only a message box (`gui.py:1689`); the file is never read. |
| Control: types of control modes | 🟡 | The bilateral controller now really has 4 modes (`control_mode`: unilateral / pp / pf / misa), selectable via `ros2 launch controller controller_misa.launch.py control_mode:=…`. The GUI dropdown is not wired to it yet. |
| MPC Controller button: "difficult to trigger, need to test it runs" | 🟡 | The button runs `controller_misa.launch.py`, which **could never have worked** (it asked for `controllers.yaml`, and `config/` wasn't installed). Fixed: the launch file and config are installed, and `launch_drivers:=true` starts both arms in the right modes. **Remaining:** the button should pass `launch_drivers:=true` and also open RViz with the robot loaded. |
| Reviews: equations | ✅ | `docs/MATH_AND_COMPARISON.md` §1; the code implements them. |
| Reviews: MPC not compared, no baseline/ablation | ✅ 🟡 | Simulation baselines (unilateral, P-P, P-F), ablations, delay and stiffness sweeps, and a stability map. Hardware repeat remaining. |
| Reviews: RMSE issue | ✅ | 0.3418 is the mean of per-joint RMSEs; pooled is 0.3583. `controller/metrics.py` and `tools/evaluate_tracking.py` compute pooled RMSE, lag, and lag-compensated RMSE from logs. Cause of the large RMSE found (B1–B3). |
| Reviews: force-feedback path never measured | 🟡 | Measured in simulation against exact ground truth (S1, S2, S3). Hardware protocol with known masses / kitchen scale in MATH_AND_COMPARISON §4. |
| Reviews: check figures | 🟡 | New figures generated (comparison, joint tracking, S1–S5) and replacements proposed. Paper not edited yet. |
| Reviews: prior work | ❌ | Reference list proposed (MATH_AND_COMPARISON §4.5). Paper not edited. |
| Reviews: "software infrastructure, not control theory" | 🟡 | Contribution reframed: momentum-observer-based sensorless feedback, energy-tank passivity (with proposition), and a correct offset-free MPC for a position-controlled slave, plus ablations. Paper not edited. |

## UI / GUI (all still to do)

| Item | Status | Notes / how |
|---|---|---|
| Change title to MERLIN | ❌ | Titles are still "arm-bench – …" (`gui.py:131,234,427,535`), "ARM Bench – Control Algorithm Development Studio" (`matlab_integration_gui.py:33`) and "RViz / Launch Manager". |
| Remove "first opening window" and "operation mode window"; open directly at the controls panel; no model / 4-arm selection | ❌ | The flow is still Arm Selection → Operation Mode → Mode Selection → Monitoring (`gui.py:126, 234, 427, 535`). Make `launch_monitoring_interface` the entry point and start on the Control tab. |
| Remove the Overview section | ❌ | Still the first tab (`gui.py:619`). |
| Seamless back/forth navigation; no way back from the MATLAB window | ❌ | The PyQt5 tuning window (`control_simulator_gui.py`) is a separate application with no return path. Launch it as a child window, or add a "Back to MERLIN" action that closes it and raises the main window. |
| One "Run ROS2 files" button (RViz, launch files, nodes) under Visualization | ❌ | Visualization still has Launch RViz, 2 Robot Simulation, 4 Robot Simulation and a checkbox (`gui.py:843-881`). `rviz_launch_gui.py` already has a launch manager to reuse. |
| One button per operation mode (slider, WebXR, VR, …) | ❌ | Teleoperation has Joystick, Camera (WebXR), Joint Controller and MPC (`gui.py:906-942`). |
| Custom control: only "open MATLAB parameter tuning" | ❌ | Remove the mode dropdown, frequency, Load and Start buttons (`gui.py:955-1040`), which do nothing anyway. |
| No colour/font/style changes, only deletions and navigation | — | Constraint noted for the UI work. |
| Motor status: only connected motors, no hardcoded values, search per port | ❌ | Hardcoded XM430/XL430 by index and 30 °C default (`gui.py:1452-1463`). How: CODE_FIXES D13. |
| Cameras: test with multiple cameras | ❌ | Not testable here (no cameras). |
| Cameras: calibration button running the Python script | ❌ | The ROS calibration node is a stub, and `session.py` can't import (CODE_FIXES D2–D4). Fix those, then add a button that runs the AprilTag calibration. |
| RLDS collection "seems to be working" | ❌ **actually broken** | It records **synthetic** joint data (sin/cos/random) and no camera frames (CODE_FIXES D1). The saved datasets contain only JPGs. |
| Param tuning: which files run; two sets, one buggy | ✅ (identified) | **In use:** `matlab_integration_enhanced.py` → `control_simulator/control_simulator_gui.py` (PyQt5), from both the GUI button and `arm-bench matlab-tune`. **Unused set:** `arm_bench/matlab_integration_gui.py` + `matlab_integration_ui.py` + `matlab_integration.py` and `control_simulator_gui_tkinter.py`. Both have bugs (CODE_FIXES D7, D8). |
| Param tuning: pick parameters from the script (`#parameter` comment syntax) | ❌ | Proposed syntax: `# @param name default min max` above a module-level assignment. Parse with `ast` + regex in `control_simulator_gui.py` when a script is loaded. |
| Teleop: joystick never tested; citation? | ❌ | Not cited in the paper. Cite the source repo. Test needs hardware. |
| Teleop: WebXR never tested; citation? | ❌ | Comes from `sudhirpratapyadav/manipulator_teleop` (+ mink); not cited. Add both. |
| Joint slider works in RViz sim | — | No change. |
| Multi-arm visualization done; hybrid sim = launch parameters | — | No change. |

## README / docs

| Item | Status | Notes |
|---|---|---|
| README "seriously outdated" | ❌ | Still describes "ARM Bench" and commands that don't match the GUI. The new controller docs exist (`docs/CODE_FIXES.md`, `docs/MATH_AND_COMPARISON.md`, `docs/TORQUE_STUDIES.md`, comments in `config/controller.yaml`, header of `controller_misa.launch.py`), but the top-level README still needs rewriting. |
| All valid commands; which module does what | ❌ | Can be generated from `cli.py` (Typer commands) plus the package layout. |
| Control file format and how to use it; how to write control logic in the MATLAB window (language, example); how to save and compare | ❌ | Needs the `# @param` syntax decision above, and one worked example (a PD controller script). |
