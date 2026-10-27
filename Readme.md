# MERLIN: Multi-arm Experimental Robotics Learning and INtegration

MERLIN is a testbed for Interbotix X-Series robot arms. It combines **sensorless bilateral teleoperation** (force
reflection from motor current, master–slave with different arms), **multi-arm tooling** (GUI, motor monitor, RViz and
launch management), a **geometry-aware data-collection pipeline** (camera calibration, synchronized joint/camera/action
episodes in HDF5), and an **in-GUI environment for developing and tuning control laws** (MATLAB engine). The reference
setup is a ReactorX 200 master and a ViperX 300S slave.

> **Status.** The software, the controller mathematics and the simulation study are implemented and tested. The
> *revised* bilateral controller has **not** been run on the real arms; all of its results are MuJoCo simulation.
> The only hardware result is the initial implementation (RMSE 0.358 rad, see [Results](#results)). The protocol for the
> hardware measurements is in [`docs/HARDWARE_VALIDATION.md`](docs/HARDWARE_VALIDATION.md).

Authors: Saipushkar Nagaraj, Yash Sanjeev Halbhavi, Ameer K. Mulla (IIT Dharwad). Video demonstration:
<https://youtu.be/_An5y608v_o>.

## Contents
1. [Features and their status](#features-and-their-status)
2. [Architecture](#architecture)
3. [The bilateral controller](#the-bilateral-controller)
4. [Results](#results)
5. [Requirements and installation](#requirements-and-installation)
6. [Usage](#usage)
7. [Repository layout](#repository-layout)
8. [Documentation](#documentation)
9. [Tests and reproducibility](#tests-and-reproducibility)
10. [Limitations](#limitations)
11. [Third-party software and acknowledgements](#third-party-software-and-acknowledgements)
12. [Citation](#citation) · [License](#license)

## Features and their status

| Feature | Where | Status |
|---|---|---|
| Bilateral teleoperation, RX200 → VX300s, 50 Hz (modes: proposed MISA+MPC, position–force, position–position, unilateral) | `ros_ws/src/controller` | implemented, unit-tested, simulated; **not run on hardware** |
| Sensorless external-torque estimation (generalised-momentum observer) | `controller/observer.py` | simulated; needs `k_t` calibration on hardware |
| Slave position MPC (offset-free, constrained QP solved by ADMM) | `controller/mpc_optimizer.py` | simulated; position-loop model must be identified on hardware |
| Energy-tank passivity guard, Jacobian force mapping between different arms | `passivity.py`, `force_mapping.py` | simulated; tank does not cover communication delay |
| Teleoperation: joint sliders, joystick, WebXR (phone) | `arm_bench_dev/src/arm_bench` | implemented; WebXR server checked by loopback test only |
| Motor status (ROS first, serial fallback), camera scan/calibration (checkerboard + AprilTag) | `arm_bench_dev` | implemented |
| Episode recorder: joint states, motor currents, **commanded actions**, camera frames, calibration (`steps.jsonl`, `episode.h5`) | `episode_recorder.py` | implemented, unit-tested with stubs; not yet run with real cameras |
| Control Simulator: free-text control laws, `@param` declarations, automatic tuning (MATLAB engine) | `arm_bench_dev/src/control_simulator` | parser/GUI tested; MATLAB engine itself not run here |
| MuJoCo benchmark and torque studies, formal derivations | `experiments/bilateral_sim`, `docs/derivations` | complete |
| Hardware pre-flight checks (cameras, joystick, WebXR, ROS rates) | `python -m arm_bench.hardware_check` | statistics unit-tested; devices not available in development |

## Architecture

```
                    GUI / CLI (arm-bench)
      ┌───────────┬──────────┬───────────┬──────────────┬───────────────┐
      │ Teleop    │ Bilateral│ Recorder  │ Camera       │ Control       │
      │ joint /   │ controller│ (HDF5)   │ calibration  │ Simulator     │
      │ joystick/ │ (ROS 2)  │           │              │ (MATLAB)      │
      │ WebXR     │          │           │              │               │
      └─────┬─────┴────┬─────┴─────┬─────┴──────┬───────┴───────────────┘
            │          │ <arm>/commands/joint_group, <arm>/joint_states
            └──────────┴───── Interbotix xs_sdk (ROS 2 Humble) ─── Dynamixel arms
```

Bilateral data flow (50 Hz): master joint state and current → estimators → admittance → reference for the slave → MPC →
position command to the slave; slave contact torque → Jacobian map → energy-tank gate → torque on the master.

## The bilateral controller

Master RX200 in current mode, slave VX300S in position mode, forearm-roll locked (5 active joints). Let
`M q̈ + C q̇ + G = τ_m + τ_ext`, with `τ_m = n·k_t·I` from the current published in mA.

| Block | Equation | Why |
|---|---|---|
| Observer | `ṙ = K_O(τ_ext − r)`, `p = M q̇` (no acceleration needed) | the static residual `G + C q̇ − τ_m = τ_ext − M q̈` shows a false contact torque in motion |
| Slave admittance | `M_d q̈_e + D_d q̇_e + K_d q_e = dz(τ̂_ext,s)`, `q_s,ref = S q_m + o + q_e`, exact ZOH discretisation | explicit Euler diverges for the original parameters |
| Master | `τ_cmd = Ĝ − D q̇_m + γ τ_f` | the original law integrated its own command (pole at z = 1) |
| Force map | `τ_f = k_of J_mᵀ S_f F̂_s − k_om K_md (S q_m + o − q_s)`, `F̂_s` by damped least squares | one-to-one joint-torque reflection over-renders ≈ 1.4× between these arms |
| Slave MPC | `min Σ w_k (q̂_k − r_k)² + R Σ (Δu_k)²` s.t. `u` limits, `|Δu|`, `|v̂|` on the identified position loop `q̈ = ω²(u−q) − 2ζω q̇ + c`, offset-free | the slave takes position commands, the old MPC's output was identically zero |
| Energy tank | `Σ Δt γ P_out ≤ T₀ − T_min + Σ Δt P_in` | bounds the power fed back to the operator (not delay-robust) |

Assumptions, proofs and the unit tests that check them numerically: [`docs/derivations/merlin_derivations.pdf`](docs/derivations/merlin_derivations.pdf)
(source `.tex` alongside). A comparison with the first implementation is in
[`docs/EVALUATION_METHODOLOGY.md`](docs/EVALUATION_METHODOLOGY.md).

## Results

Two kinds of numbers exist and must not be mixed.

**Hardware (initial implementation, bimanual RX200/VX300S, human operator):** overall pooled RMSE **0.3583 rad**
(mean of per-joint RMSEs 0.3418 rad), MAE 0.2267 rad, max 1.5555 rad. A later code analysis traced the lag to a smoothing step
(`q ← 0.95 q + 0.05 q_ref`, time constant 0.39 s) and to an MPC output that was identically zero.

**Simulation (MuJoCo 3.17, revised controller; plant with +8 % link mass, unmodelled friction, quantised noisy current;
5 seeds, mean ± std):**

| Controller | RMSE free motion (rad) | RMSE whole task (rad) | lag (ms) | force correlation |
|---|---|---|---|---|
| Original code (re-implemented) | 0.220 ± 0.016 | 0.164 ± 0.010 | 338 ± 6 | −0.58 ± 0.03 |
| Position–position | 0.024 ± 0.001 | 0.027 ± 0.001 | 30 ± 2 | 0.41 ± 0.24 |
| Position–force | 0.025 ± 0.001 | 0.025 ± 0.002 | 30 ± 2 | 0.23 ± 0.64 |
| **Proposed** | **0.007 ± 0.001** | **0.016 ± 0.005** | **< 20 (0 at the 20 ms resolution)** | 0.48 ± 0.34 |

Unfavourable results are in the documents on purpose: the whole-task *maximum* error of the proposed controller is larger
(0.30 ± 0.20 rad, the admittance lets the slave yield in contact), the force correlation in the benchmark is noisy, vertical
force is over-rendered by 1.6 × at `k_of = 0.3`, and the energy tank does not give delay robustness. Torque mimicking
(forces on the slave reproduced at the master): correlation 0.94–0.98 in the dedicated test. Details, setup, software
versions, metric definitions and the caveats: [`docs/EVALUATION_METHODOLOGY.md`](docs/EVALUATION_METHODOLOGY.md),
[`docs/TORQUE_STUDIES.md`](docs/TORQUE_STUDIES.md), figures in `experiments/bilateral_sim/results/`.

## Requirements and installation

* Ubuntu 22.04, ROS 2 Humble, Python ≥ 3.10; Interbotix X-Series driver (see [`InterbotixDriver/README.md`](InterbotixDriver/README.md)).
* Python packages: `numpy scipy pyyaml typer rich matplotlib pillow` (base); `opencv-python pygame fastapi uvicorn websockets
  dynamixel-sdk` (full); `pin h5py pupil-apriltags` (dynamics, episode files, AprilTag calibration); `PyQt5` and
  `matlabengine` (Control Simulator); `mujoco` (benchmark, see `experiments/bilateral_sim/requirements.txt`).
  The top-level `requirements.txt` is a full environment snapshot of a development machine, not a minimal list.

```bash
git clone --recurse-submodules https://github.com/Batyoki/MERLIN.git && cd MERLIN
source ~/interbotix_ws/install/setup.bash          # after installing the Interbotix driver
cd ros_ws && colcon build && source install/setup.bash && cd ..
pip install -e "arm_bench_dev[full]"
pip install pin h5py pupil-apriltags
```

## Usage

**Application.** `arm-bench start` opens the GUI (Control, Motor Status, Cameras, Recording). Control tab: RViz and ROS launch
runner; teleoperation (joint sliders, joystick, WebXR); *Bilateral Teleoperation* (choose a mode, optional controller
config, *Start*/*Stop*; logs in `~/.merlin/logs/`); *MATLAB Parameter Tuning*. Cameras: scan, start, calibrate
(results in `~/.merlin/calibration/`). Recording writes `steps.jsonl`, `camera_frames/`, `episode.h5` per episode,
including the commanded actions when the controller is running.

**Command line.**

```bash
arm-bench start [--arm vx300s]
arm-bench record start [--camera 0 --camera 2] [--rate 30] [--duration 60] [--output DIR]
arm-bench setup | cameras | matlab-tune | --help

ros2 launch controller controller_misa.launch.py launch_drivers:=true control_mode:=misa \
     params_file:=/path/gains.yaml log_dir:=/tmp/merlin
python3 tools/identify_position_loop.py /tmp/merlin/bilateral_R_*.csv --plot   # ω, ζ of the slave position loop
python3 tools/evaluate_tracking.py /tmp/merlin/bilateral_R_*.csv --markdown    # paper-style table from a log
```

Controller parameters are in `ros_ws/src/controller/config/controller.yaml` (documented line by line). **Do not enable
`misa` on the real arms before the first-run protocol** in `docs/HARDWARE_VALIDATION.md` (offsets, `k_t` calibration,
position-loop identification, `kof: 0.1` first).

**Control Simulator with `@param`.** Parameters, bounds and descriptions are declared in the control script:

```matlab
% @param min=0 max=50 desc="proportional gain"
Kp = 8.0;
u = Kp*e + Ki*integral + Kd*derivative;
```

Walkthrough with a complete tuning session: [`docs/CONTROL_SIMULATOR_WALKTHROUGH.md`](docs/CONTROL_SIMULATOR_WALKTHROUGH.md).

**Hardware pre-flight checks.**

```bash
python3 -m arm_bench.hardware_check cameras --seconds 10      # fps, gaps, inter-camera offsets
python3 -m arm_bench.hardware_check joystick                  # idle offsets and range
python3 -m arm_bench.hardware_check webxr                     # server loopback (no phone)
python3 -m arm_bench.hardware_check ros                       # joint_states rates, command topics
```

## Repository layout

| Path | Content |
|---|---|
| `arm_bench_dev/` | Application (Python package `arm_bench`, command `arm-bench`), `control_simulator/`, tests |
| `ros_ws/src/controller` | Bilateral controller node: observer, admittance, MPC, passivity, force mapping, metrics; `config/`, `launch/`, `test/` |
| `ros_ws/src/data_collection_pkg` | Camera publisher, AprilTag calibration, frame writer nodes |
| `ros_ws/src/arm_launch`, `moveit_2_config`, `config_teleop` | Multi-arm launch, MoveIt 2 configuration, teleoperation configuration |
| `experiments/bilateral_sim` | MuJoCo benchmark (`run.py`, `multi_seed.py`, `model_validation.py`, `torque_studies.py`) and committed `results/` |
| `tools/` | `evaluate_tracking.py`, `identify_position_loop.py` |
| `docs/` | Documentation index below; `derivations/` (LaTeX + PDF) |
| `paper/` | Revised manuscript source, change log, generated tables |
| `archive/` | Superseded code (ignored by colcon) |
| `manipulator_teleop1/` | Git submodule: third-party WebXR teleoperation code |

## Documentation

| Document | Content |
|---|---|
| [`docs/derivations/merlin_derivations.pdf`](docs/derivations/merlin_derivations.pdf) | Formal derivation of every equation, assumptions, proofs, limits |
| [`docs/EVALUATION_METHODOLOGY.md`](docs/EVALUATION_METHODOLOGY.md) | Old vs. new MPC, how the tables were produced, software, metrics, multi-seed results |
| [`docs/TORQUE_STUDIES.md`](docs/TORQUE_STUDIES.md) | Studies S1–S5 (estimation, mimicking, weight, stiffness, stability map) |
| [`docs/CODE_FIXES.md`](docs/CODE_FIXES.md), [`docs/MATH_AND_COMPARISON.md`](docs/MATH_AND_COMPARISON.md) | Defects found and fixed; novelty and comparison with baselines |
| [`docs/HARDWARE_VALIDATION.md`](docs/HARDWARE_VALIDATION.md) | First-run protocol and what remains unmeasured |
| [`docs/CONTROL_SIMULATOR_WALKTHROUGH.md`](docs/CONTROL_SIMULATOR_WALKTHROUGH.md) | `@param` syntax and a worked tuning session |
| [`docs/ISSUE_CONTEXT_STATUS.md`](docs/ISSUE_CONTEXT_STATUS.md) | Status of the open issues |
| [`paper/CHANGES.md`](paper/CHANGES.md) | Changes to the manuscript and decisions for the authors |

## Tests and reproducibility

```bash
cd ros_ws/src/controller && python3 -m pytest test/test_control_math.py test/test_derivations.py   # control math + derivation checks
cd arm_bench_dev && PYTHONPATH=src python3 -m pytest tests            # GUI back-ends, recorder, parser, hardware checks
python3 experiments/bilateral_sim/run.py                              # single-seed benchmark
python3 experiments/bilateral_sim/multi_seed.py --seeds 5             # table above
python3 experiments/bilateral_sim/model_validation.py                 # N-step prediction of the slave model
python3 experiments/bilateral_sim/torque_studies.py                   # S1–S5 (≈ 5–10 min)
python3 paper/make_tables.py                                          # LaTeX tables from the result files
```
Simulation numbers were produced with Python 3.13, NumPy 2.5, SciPy 1.18, MuJoCo 3.17, Pinocchio 4.1 (recorded in
`results/*.json`). Results depend on the library versions only through the physics integrator.

## Limitations

* The revised controller has no hardware measurement. The simulated servo's position loop is our own PD law; `ω, ζ`, `k_t`,
  friction and the real latency must be identified on the arms.
* The energy tank guards the master port only; stability under communication delay is not guaranteed (two-port tank / TDPA is future work).
* The scheme exchanges three signals (master position; slave contact torque and position), not four.
* No collision checking, workspace bounds or software emergency stop. No depth, tactile or haptic channels in the episode schema.
* USB cameras are not hardware-synchronised; the recorder stores capture times for alignment.
* A few old dataset folders (`arm_bench_dataset_*`) from the earlier recorder are still tracked; they contain camera frames only.

## Third-party software and acknowledgements

* WebXR teleoperation: based on [`manipulator_teleop`](https://github.com/sudhirpratapyadav/manipulator_teleop) (S. P. Yadav), included as a submodule.
* Cartesian joystick teleoperation uses [`mink`](https://github.com/kevinzakka/mink) (K. Zakka).
* [Interbotix X-Series ROS 2 packages](https://github.com/Interbotix) (xs_sdk, descriptions), [Pinocchio](https://github.com/stack-of-tasks/pinocchio), [MuJoCo](https://mujoco.org), ROS 2 Humble.
* The ADMM scheme follows OSQP (Stellato et al., 2020); the momentum observer follows De Luca & Mattone. Full references in `docs/derivations/`.
* This work is partially supported by the Anusandhan National Research Foundation (ANRF), Government of India.

## Citation

The manuscript is under review. Until it is published, please cite the repository:

```bibtex
@misc{merlin2025,
  title  = {{MERLIN}: Multi-arm Experimental Robotics Learning and INtegration},
  author = {Nagaraj, Saipushkar and Halbhavi, Yash Sanjeev and Mulla, Ameer K.},
  year   = {2025},
  howpublished = {\url{https://github.com/Batyoki/MERLIN}},
  note   = {Manuscript under review}
}
```

## License

MIT, Copyright (c) 2025 Saipushkar Nagaraj. See [`LICENSE`](LICENSE). Third-party components keep their own licenses.
