# Status of the items in `issue_context.md`

✅ done on branch `claude/brave-ride-bizkpi` (PR #1) · 🟡 done in code, needs hardware / paper work · ❌ still to do.
"Verified" means exercised headless (Xvfb screenshots, simulated camera / Dynamixel bus / `ros2`), not on the robots.

## Control and torque

| Item | Status | What was done / what remains |
|---|---|---|
| Torque: master torque read, passed through the loop, output "too rough or no change at all" | 🟡 | Root causes fixed (CODE_FIXES A1–A13). Torque mimicking works in simulation (`docs/TORQUE_STUDIES.md` S2, correlation 0.94–0.98). **Remaining:** first run on the arms (checklist at the end of CODE_FIXES). |
| Control: Start Control (what does it run?) | ✅ verified | Starts the bilateral controller (`ros2 launch controller controller_misa.launch.py`) with the selected mode, frequency, config and optional drivers/RViz. **Stop Control** ends it (SIGINT to the launch group). Output goes to `~/.merlin/logs/controller_<time>.log`. |
| Control: Load Control Config | ✅ verified | Picks a controller YAML; it is checked for the `controller: ros__parameters:` section, then passed as `params_file`. |
| Control: types of control modes | ✅ | Dropdown: Proposed (MISA + MPC) / Position-force / Position-position / Unilateral. |
| MPC controller: run it from the button, and have RViz come up with the robot loaded | ✅ 🟡 | "Start both arm drivers" + "Open RViz with the robot" checkboxes pass `launch_drivers:=true use_rviz:=true` (RViz opens through the slave's Interbotix launch). Needs a ROS machine to confirm. |
| Reviews: equations · baselines/ablation · RMSE | ✅ | `docs/MATH_AND_COMPARISON.md`, `experiments/bilateral_sim`, `controller/metrics.py`. |
| Reviews: force-feedback path never measured | 🟡 | Simulation with exact ground truth (TORQUE_STUDIES S1–S3); hardware protocol in MATH_AND_COMPARISON §4. |
| Reviews: figures · prior work · "infrastructure not control" | 🟡 | Figures and reference list prepared; **paper text not edited**. |

## UI / GUI

| Item | Status | Notes |
|---|---|---|
| Change title to MERLIN | ✅ | Main window, its header, the "Run ROS 2 Files" window, the tuning window, the calibration and joint-slider windows, and the CLI. |
| Remove the first opening window and operation-mode window; open directly on the controls; no model / 4-arm selection | ✅ verified | `arm-bench start` opens the main window on the **Control** tab. The Arm Selection, Operation Mode and Mode Selection windows are deleted. |
| Remove the Overview section | ✅ | Tabs: Control · Motor Status · Cameras · Recording. |
| Seamless navigation; no way back from the MATLAB window | ✅ verified | MERLIN hides while the tuning window is open and returns when it closes. The tuning window has a "← Back to MERLIN" button; the calibration and Run-ROS-2 windows have Back buttons. |
| One "Run ROS 2 files" button (RViz, launch files, nodes) under Visualization | ✅ | Opens the launch manager, which now lists launch files **and** nodes (`ros2 run`) and has "Open RViz". The 2/4-robot PyBullet buttons and the checkbox are removed. |
| One button per operation mode | ✅ | Teleoperation: Joint Sliders · Joystick · WebXR. Bilateral master–slave has its own panel (Start/Stop). VR: no code exists, so no button was added. |
| Custom control: only MATLAB parameter tuning | ✅ | That panel now has only "MATLAB Parameter Tuning". |
| No colour/font/style changes | ✅ | Same palette, fonts and button styles; only removals, renames and the new controls. |
| Motor status: only connected motors, no hardcoded values, search per port | ✅ verified (simulated bus) | Live from ROS `*/joint_states` when the driver runs; otherwise port + baud selector, "Scan Port", read-only Dynamixel reads (model, position, velocity, current/load, voltage, temperature). Empty when nothing answers. Temperature/voltage are not shown in ROS mode (the driver doesn't publish them). |
| Cameras: test with multiple cameras | 🟡 | The grid handles N cameras (2 per row) and "Scan Cameras" re-detects. Not testable here without cameras. |
| Cameras: calibration button | ✅ verified (synthetic images) | "Calibrate" on each camera: checkerboard intrinsics, AprilTag extrinsics, Plücker ray embeddings, saved per camera and copied into each episode. |
| RLDS collection | ✅ verified | Was saving frozen joint values (GUI) or made-up sine/random data (CLI). Now real `*/joint_states` + frames + calibration → `steps.jsonl` + JPEGs + `episode.h5`. |
| Param tuning: which files run; two sets, one buggy | ✅ | In use: `control_simulator/control_simulator_gui.py` (fixed, CODE_FIXES D7/D8). Unused Tkinter studio moved to `archive/`. |
| Param tuning: read parameters from the script (`#parameter` syntax) | ❌ | Not done. Proposed syntax: `# @param name default min max` above the assignment, parsed with `ast` when the script loads. |
| Teleop: joystick / WebXR never tested; citations | ❌ | Need hardware / a headset. Citations (D12) intentionally left for you. |

## README / docs

| Item | Status | Notes |
|---|---|---|
| README outdated; list of valid commands; which module does what | ✅ | Top-level `Readme.md` rewritten: start, record, controller launch, the GUI tabs, the module map, and where the docs are. |
| Control file format and how to use it | ✅ | `ros_ws/src/controller/config/controller.yaml` is the format (every key commented); Load Control Config takes a copy of it. |
| How to write control logic in the MATLAB window, with a worked example; save and compare | ❌ | Not done (depends on the `# @param` decision). |
