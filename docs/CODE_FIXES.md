# MERLIN — Code fixes (List 1)

Branch `claude/brave-ride-bizkpi`. Every item says **where**, **what is wrong**, **evidence**,
and **how it is fixed** (or how to fix it, for items not done yet).
Status: ✅ fixed on the branch · 🔧 fix described, not done · ⚠️ needs hardware verification.

Evidence comes from three sources:
- reading the code;
- running the original modules directly (the original MPC; the gravity indexing via pinocchio);
- the MuJoCo benchmark in `experiments/bilateral_sim/`. It uses the same RX200/VX300s URDFs, with a plant mass +8 % versus the controller model, joint friction the controller doesn't model, encoder quantisation, and xs_sdk current quantisation (2.69 mA) plus noise.

> **Simulator.** MuJoCo is not used anywhere in MERLIN's own code. It appears only in
> `requirements.txt` and in the `manipulator_teleop1` submodule (a third-party WebXR teleop
> for Panda/Kinova/OMX, with no Interbotix model). The bilateral code had PyBullet
> (`bilateral_teleop/bullet_arm_sim.py`, with hardcoded `/home/sp/...` URDF paths and no controller),
> Gazebo (`bilateral_gazebo.launch.py`), and a synthetic `dump/simulated_master_node.py`.
> The new benchmark is a real MuJoCo simulation of both arms.

---

## A. Torque mimicking / force feedback ("too rough or no change at all")

| # | Status | Where (original) | Problem | Fix |
|---|---|---|---|---|
| A1 | ✅ | `arm_interface.py:347` | `tau = kt * current` with `current` taken from `JointState.effort`, which xs_sdk publishes in **mA**. Every torque was 1000× too large, so every downstream clamp saturated. Neither state relay node converted it. | `CURRENT_UNIT_SCALE = 1e-3` in `ArmInterface.update`. |
| A2 | ✅ | `arm_interface.py:96-97` | Shadow (dual) motors ignored. The RX200 shoulder and VX300s shoulder/elbow have two motors (`shadows:` in `dump/rx200.yaml`), and only the primary's current is reported. | `n_motors` per joint: τ = n·k_t·I. Commanded current is divided by n. |
| A3 | ✅ | `misa_mpc_controller.py:466-500` | **Positive feedback on the master.** In current mode the measured current is about the commanded current, so `tau_ext_master = kt·I − G` is roughly the last command minus gravity. Adding it back into `tau_m_cmd` makes τ[k] = τ[k−1] + (feedback terms): an integrator that winds up to the clamps. | The master no longer estimates or feeds back its own "external" torque (`estimator="none"`). A master-side observer driven by the *commanded* torque is novelty item 1 (deferred). |
| A4 | ✅ | `controller.py:754-765` | Master current clipped to 0.5×[100,150,100,50,50] mA, i.e. 0.05–0.13 N·m. The RX200 shoulder needs 1.2–1.5 N·m just for gravity (computed with pinocchio), so the arm sags and any feedback is imperceptible. In simulation the operator felt a constant −1.2 N bias and nothing from the wall. | Limit in torque (`tau_master_max` ≈ 50 % stall: [2, 4, 2, 2, 0.75] N·m), then converted to mA per motor. |
| A5 | ✅ ⚠️ | `controller.py:222` + `misa_mpc_controller.py:255` | The −3.11 rad shoulder "correction" is applied before `G_m` is computed, so gravity compensation is evaluated about π away from the real pose. | Two separate parameters: `master_joint_offset` (raw → URDF convention, used for gravity) and `map_offset` (master → slave reference only). **Verify on hardware:** at the Interbotix sleep pose the RX200 shoulder should read ≈ −1.80 rad. |
| A6 | ✅ | `arm_interface.py:202, 239` | The 5-DOF `vx300s.urdf` in the package has **no forearm_roll**, yet `get_G`/`get_C` insert a 0 at index 3 and delete index 3. wrist_angle is evaluated as 0 and its gravity is dropped: −0.004 N·m instead of −0.35 N·m at q = [0, 0.3, 0, 0.4, 0]. That 0.35 N·m bias sat right at the 0.3 N·m dead zone. | New `dynamics.py` `ArmModel`: joints mapped **by name** (`getJointId`). Continuous gripper joint handled with `pin.neutral`. M(q) from `crba` (the old M was a constant matrix from `data.json`). |
| A7 | ✅ | `arm_interface.py:361-366`, paper Eq. 2 | **The static residual can't tell contact from motion.** On a position-controlled arm the motor current is dominated by M q̈ (the position loop accelerating the links). Measured in simulation: ±12 N·m RMS on the shoulder in *free motion*. Reflected to the operator, this is "rough". | Generalised-momentum observer on the slave (`observer.py`, De Luca 2003): ṙ = K_O(τ_ext − r), no q̈ needed. Free-motion tracking under force feedback went from 0.230 to 0.026 rad RMSE. |
| A8 | ✅ | (new code) | The observer must get **unfiltered** q̇. With the α = 0.3 IIR velocity (≈ 47 ms lag), p = M q̇ lags the torque integral. That acts as negative inertia and produced a master–slave limit cycle (free RMSE 0.113 rad). | `observer.update(q, dq_raw, tau_raw)`. Result: 0.016 rad. |
| A9 | ✅ | `misa_mpc_controller.py:451`, `controller.yaml` | Dead zones [0.5, 0.8, …] N·m with a 1000× unit error were meaningless. After the unit fix they hide light contacts. | [0.15, 0.3, 0.2, 0.1, 0.05] N·m (configurable). Re-tune on hardware from the observer's free-motion residual (≈ 3σ). |
| A10 | ✅ | `misa_mpc_controller.py:310-500` | Sign convention mixed: τ_motor − (C+G) is −τ_ext, then `−kof·τ_ext_s`. Paper Eq. 2/3 even has `−C + G`. | One convention everywhere: M q̈ + C q̇ + G = τ_motor + τ_ext, so τ̂_ext = G + C q̇ − τ_motor (environment torque), and τ_f = +k_of Γ τ̂_ext,s. |
| A11 | ✅ | `misa_mpc_controller.py:348-394` | Admittance integrated with explicit Euler using the **physical** inertia as M_sd (wrist ≈ 7e-4). ω_n Δt ≈ 2.4 at 20 ms, outside Euler's stability region. It was held only by `MAX_DDQ` / ±0.05 rad clamps, which chatter. | `admittance.py`: exact ZOH discretisation (matrix exponential), stable for every Δt. M_sd, D_sd, K_sd become design parameters (ω_n ≈ 10 rad/s, ζ = 1). Anti-windup at the q_e limit. |
| A12 | ✅ | `controller.yaml` gains | Gains in the wrong units/scale for these motors: Kmd = 120 N·m/rad on a joint with a 3 N·m clamp is bang-bang; Dmd = 4.4 N·m·s/rad saturates at 1 rad/s. | Rescaled defaults (see `config/controller.yaml`). Every value was checked in simulation. |
| A13 | ✅ | `kof` | In simulation, k_of = 0.6 limit-cycles with 60 ms delay (RMSE 0.30 rad, 106 N wall peaks). | Default k_of = 0.3 (0.016/0.024/0.040 rad RMSE at 0/20/60 ms delay). The energy tank (B4) makes higher gains survivable. |

## B. Slave tracking, the "MPC", and why Table I's RMSE is 0.36 rad

| # | Status | Where | Problem | Fix |
|---|---|---|---|---|
| B1 | ✅ | `mpc_optimizer.py` (old) | **The MPC output is always zero.** The decision variables are [dq_0..dq_{N−1}, τ_0..τ_{N−1}], but the rollout uses only τ; dq appears only in the cost dqᵀR dq. So dq* = 0 for any reference, and the slave integrated dq*_0. *Verified by running the original optimizer: error 0.4–0.7 rad → `dq_cmd` = 0.00 exactly, 5/5 trials.* | `PositionMPC`: decision variable = position command u, model = identified position loop, constraints on u, Δu and predicted q̇. |
| B2 | ✅ | old SLSQP | 17–48 ms per solve (finite-difference gradients, 48 variables) against a 20 ms loop budget. | Condensed QP per joint (N = 10 variables). Fixed-iteration ADMM with a pre-factorised KKT matrix: **0.8 ms mean, < 2 ms max**. |
| B3 | ✅ | `misa_mpc_controller.py:641` | The actual slave motion came from `q ← 0.95 q + 0.05 q_ref` at 50 Hz: a first-order lag with τ = 0.39 s. That is the cause of the 0.36 rad RMSE and the "delays at direction changes". Simulation reproduces it: original code RMSE 0.161 rad, measured lag **336 ms**. | Removed. The MPC command goes straight to the Dynamixel loop, and the reference is previewed with the master velocity. Measured lag **8 ms**, RMSE **0.016 rad**. |
| B4 | ✅ | master | The master MPC was commented out (`m_ok = True`), and there was no passivity mechanism. | Explicit impedance law plus an **energy tank** (`passivity.py`). Without it k_of = 1.5 diverges (RMSE 0.188 rad, 71 N); with it, 0.031 rad and 39 N. |
| B5 | ✅ | `mpc_optimizer.py` (new) | A linear MPC can't predict the static gravity sag of a position loop without gravity feed-forward, so the first version tracked *worse* than sending q_ref directly. | Offset-free MPC: parallel internal model plus output-disturbance estimate d̂ = LPF(q − q̂). |
| B6 | ✅ | `VIPERX300S_DQ_MAX = 0.4`, `MAX_DQ = 0.4`, ±0.5 rad clip, 0.12 rad/cycle limiter | Stacked ad-hoc limits on top of the lag. | One set of explicit MPC constraints (`dq_slave_max`, `du_slave_max`) and the joint limits. |
| B7 | ✅ | `controller.py:146-156` | `ArmDynamics("reactorx150")` loads `rx200.urdf`, and the docstrings say the master is a ViperX and the slave a ReactorX150, the reverse of the hardware. | Named `rx200` / `vx300s` throughout. |

## C. Plumbing: things that never ran as intended

| # | Status | Where | Problem | Fix |
|---|---|---|---|---|
| C1 | ✅ | `controller_misa.launch.py` | Loads `config/controllers.yaml`, but the file is `controller.yaml`, and `setup.py` doesn't install `config/`. **The YAML gains were never loaded**; hardcoded `declare_parameter` defaults ran instead. The GUI's "Start control" button launches exactly this file. | `setup.py` installs `config/*.yaml`, `launch/*.launch.py` and the URDFs. The launch file loads `controller.yaml` and can optionally start both Interbotix drivers with the right modes (`launch_drivers:=true`). |
| C2 | ✅ | `arm_interface.py:147,154,158`, `controller.py:98`, `misa_mpc_controller.py:709` | Absolute paths `/home/sp/Desktop/...`. `data.json` isn't in the repo at all, so the node can't start on any other machine. | `dynamics.resolve_urdf()` ($MERLIN_URDF_DIR → package share → source dir). `data.json` is no longer needed: M, C, G and J all come from pinocchio. |
| C3 | ✅ | `misa_mpc_controller.py:300,333,337,363,365,624` | `self.n` is undefined (it should be `self.n_joints`), so every NaN fallback path raises `AttributeError`. `self.get_logger` doesn't exist on that class. | Rewritten. |
| C4 | ✅ | `controller.py:482, 351` | Only the right pair runs; the left publishers are "NEED TO CREATE". The paper's Fig. 2 shows four arms. | `sides: ["L","R"]` parameter; each side is a `Side` object with its own subscriptions and publishers. |
| C5 | ✅ | `slave_state_node.py` | Relays /slaveR/joint_states with `pop(3)` (assumes index order) and publishes **L data on the R topic** (line 119). | Relays deleted. Joints extracted by name from the driver topics. |
| C6 | ✅ | `setup.py` entry points | `simulated_master`, `joystick_teleop` and `keyboard_teleop` point to modules that live in `dump/` (not importable). The `sim_master_*` launches pass a 5-element `master_Mmd_diag` that is reshaped to 8×8, which crashes. | Broken entry points and launches removed. |
| C7 | ✅ | node | No mode config for the master, and current commands are ignored unless the arm group is in `current` mode. | `config/modes_master.yaml` (arm: current, gripper torque off) and `config/modes_slave.yaml` (position, no profile). |
| C8 | ✅ | node | On stale data the loop `return`ed, so the last current command stayed applied. | Stale → master gravity-only torque, controller reset. |
| C9 | ✅ | node | `print` at 50 Hz and `get_logger().fatal()` every tick. | Throttled logs; JSON diagnostics at 10 Hz (tank level, γ, MPC time, loop time). |
| C10 | ✅ | — | No log of the data the paper's table is computed from. | `log_dir` → CSV per side. `tools/evaluate_tracking.py` produces Table I; `tools/identify_position_loop.py` gives ω, ζ for the MPC from any log. |
| C11 | ✅ | — | No tests. | `ros_ws/src/controller/test/test_control_math.py` (7 tests: gravity indexing, mass matrix, admittance stability, MPC constraints, observer convergence, tank invariant, pooled RMSE/lag). All pass. |

## D. Rest of the repository (audited; not changed on this branch)

| # | Status | Where | Finding | How to fix |
|---|---|---|---|---|
| D1 | 🔧 | `arm_bench_dev/src/arm_bench/rlds_recorder.py:114-122` | **The RLDS recorder writes synthetic data**: positions = sin(t+i), velocities = cos(t+i), efforts = random, temperatures = random. `_capture_camera_frames` is `pass`. Anything recorded through it is not robot data. | Subscribe to `/<ns>/joint_states` (positions, velocities, efforts in mA → convert) and camera topics. Store with the real timestamps. Write HDF5 as the paper claims, or say JSON. |
| D2 | 🔧 | `ros_ws/src/data_collection_pkg/session.py:13-17` | Imports `..calibration.apriltag`, `..cameras.usb`, `..storage.schema`, `..storage.writer`, but the package is flat (`apriltag.py`, `usb.py`, …), so `ImportError` on import. | Change to `from .apriltag import …`, `from .usb import …`, `from .schema import …`, `from .writer import …`, `from .store import …`. |
| D3 | 🔧 | `data_collection_pkg/nodes/session_node.py:42` | `self.recorder.stop()[cite: 6]` is a pasted chatbot citation. It raises `NameError`/`TypeError` on shutdown. The same `[cite: N]` text appears in comments in the calibration and writer nodes. | Delete the `[cite: …]` fragments. |
| D4 | 🔧 | `data_collection_pkg/nodes/calibration_node.py`, `writer_node.py` | Stubs: "TODO: add a subscriber". They never receive images, so nothing is calibrated or written through ROS. | Add `sensor_msgs/Image` subscribers via cv_bridge and call `calibrator.detect` / `writer.write`. Add a GUI button for calibration (your issue list). |
| D5 | 🔧 | `arm_bench_dataset_2026092*` | Contain **only 82 JPGs**: no joint states, no intrinsics/extrinsics, no ray embeddings, no HDF5. Grepping the repo for Plücker/ray-embedding code finds none. | Either implement it (rays = K⁻¹[u,v,1]ᵀ rotated by R_wc, plus the camera centre) or remove the claim from the paper. |
| D6 | 🔧 | `arm_bench/webxr/server.py:24-53` | The optional-import guard for fastapi is broken: `WebSocket` annotations are evaluated anyway, so **without fastapi the whole `arm_bench` package fails to import**. All 4 test files fail at collection. | Add `from __future__ import annotations` at the top of `server.py`, or set `WebSocket = object` in the `except ImportError:` branch. |
| D7 | 🔧 | `matlab_integration_gui.py:661-780`, `control_simulator_gui.py:412` | `lambda: …str(e)` inside `except … as e` runs after `e` has been deleted (Python 3), so error dialogs themselves raise `NameError`. | `except Exception as e: msg = str(e); root.after(0, lambda m=msg: …)`. |
| D8 | 🔧 | `control_simulator/matlab_engine_interface.py:85-89` | `sys` used without import. The folder name `__init__,py` is a typo (comma). `test_simulator.py` imports `arm_bench_dev.src…`, which fails. This is the **second, buggy param-tuning implementation** from your notes. The GUI and CLI use `arm_bench/matlab_integration_enhanced.py`. | Delete `control_simulator/`, or fix it and merge it into `matlab_integration_enhanced.py`. Keep one. |
| D9 | 🔧 | `tests/test_launch_manager.py` | Fails: expects `/tmp/ros_ws/install/setup.bash` to exist. | Create a temporary dir with a fake `setup.bash` in the test. |
| D10 | 🔧 | `InterbotixDriver/install/…` | Build/install artifacts committed, including symlinks to `/home/yash_sai/...`. 96 MB. | `git rm -r --cached InterbotixDriver/{build,install,log}` and add them to `.gitignore`. |
| D11 | 🔧 | `bilateral_node/claude.py`, `teleoperation_hopefully`, `bilateral_teleop` (MoveIt FK/IK) | Three more teleop implementations. `bilateral_kinematics.py` needs `mink` + `torch`. None is the paper's MISA controller. | Keep one teleop path (`controller`); move the rest to `archive/`. |
| D12 | 🔧 | `manipulator_teleop1` (WebXR), `arm_bench/kinematics.py` joystick | Third-party code (sudhirpratapyadav/manipulator_teleop; mink by Kevin Zakka). **Not cited in the paper.** | Cite both in the paper and in the README. |
| D13 | 🔧 | `gui.py:1452-1463` | Motor status is hardcoded: XM430 for i ≤ 3, otherwise XL430; temperature defaults to 30 °C. | Read model number, temperature and voltage from the Dynamixel control table via `dynamixel_sdk`. Show only motors that respond to a ping (per port). |

---

## How to reproduce

```bash
pip install pin mujoco numpy scipy matplotlib pytest
python3 -m pytest ros_ws/src/controller/test/test_control_math.py     # 7 tests
python3 experiments/bilateral_sim/run.py                             # ~1 min
#   -> experiments/bilateral_sim/results/{results.md,results.json,comparison.png,joint_tracking.png}

# on the robots
colcon build --packages-select controller
ros2 launch controller controller_misa.launch.py launch_drivers:=true log_dir:=/tmp/merlin
python3 tools/identify_position_loop.py /tmp/merlin/bilateral_R_*.csv    # -> pos_loop_omega/zeta
python3 tools/evaluate_tracking.py /tmp/merlin/bilateral_R_*.csv --markdown
```

## Hardware checklist before the first run ⚠️
1. `ros2 topic echo /masterR/joint_states --once`: at the sleep pose the shoulder should read ≈ −1.80 rad. If not, set `master_joint_offset`.
2. Confirm the **effort unit** once. Hold the slave still, hang a known mass m at the gripper, and compare τ = n·k_t·I·10⁻³ against Jᵀ m g. This also calibrates k_t.
3. Start with `control_mode: unilateral` (the master is then gravity compensation only), then `pp`, then `misa` with `kof: 0.1` and increase it.
4. Watch `controller/teleop/diagnostics`: `gamma` < 1 means the tank is limiting the feedback.
