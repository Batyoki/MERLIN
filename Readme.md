# MERLIN — Multi-arm Experimental Robotics Learning and INtegration

A testbed for Interbotix X-Series arms:
- bilateral (master–slave) teleoperation with sensorless force feedback;
- other teleoperation modes (joint sliders, joystick, WebXR);
- dataset recording with camera calibration;
- control-algorithm tuning.

## Repository layout

| Path | What it is |
|---|---|
| `arm_bench_dev/` | The MERLIN application (Python package `arm_bench`, command `arm-bench`) |
| `ros_ws/src/controller` | Bilateral controller node (MISA + MPC, momentum observer, energy tank) |
| `ros_ws/src/data_collection_pkg` | ROS camera publisher, AprilTag calibration and frame writer nodes |
| `ros_ws/src/arm_launch` | `multi_arm.launch.py` — bring up several arms |
| `ros_ws/src/moveit_2_config` | MoveIt 2 configuration |
| `experiments/bilateral_sim` | MuJoCo benchmark and torque studies used in the paper |
| `tools/` | `evaluate_tracking.py` (paper Table I from a log), `identify_position_loop.py` |
| `docs/` | `CODE_FIXES.md`, `MATH_AND_COMPARISON.md`, `TORQUE_STUDIES.md`, `ISSUE_CONTEXT_STATUS.md` |
| `archive/` | Superseded code, kept for reference (ignored by colcon) |

## Install

```bash
# 1. ROS 2 Humble + Interbotix driver: see InterbotixDriver/README.md
# 2. MERLIN ROS packages
cd ros_ws && colcon build && source install/setup.bash && cd ..
# 3. MERLIN application
pip install -e "arm_bench_dev[full]"
pip install pin h5py pupil-apriltags     # dynamics, episode files, AprilTag calibration
```

## Run

```bash
arm-bench start                 # opens MERLIN on the Control tab
```

**Control tab**
- **Visualization → Run ROS 2 Files:** open RViz, run any package's launch file, or run a single node.
- **Teleoperation:** Joint Sliders (publishes to `<arm>/commands/joint_group`), Joystick, WebXR.
- **Bilateral Teleoperation:**
  - Pick a mode: Proposed (MISA + MPC), Position-force, Position-position or Unilateral.
  - Set the control frequency and, optionally, *Load Control Config* (a copy of `ros_ws/src/controller/config/controller.yaml`).
  - Then *Start* / *Stop*. With "Start both arm drivers" the RX200 master runs in current mode and the VX300s slave in position mode. "Open RViz" shows the slave.
  - Controller logs go to `~/.merlin/logs/`.
- **Custom Control Algorithms → MATLAB Parameter Tuning:** MERLIN comes back when that window closes.

**Motor Status:** live data from ROS `*/joint_states`. Without a running driver, choose a port and press *Scan Port* for a read-only scan. Only motors that answer are listed.

**Cameras:** *Scan Cameras*, *Start All Cameras*, and *Calibrate* per camera (checkerboard intrinsics + AprilTag extrinsics). Results go to `~/.merlin/calibration/`.

**Recording:**
- records every arm's joint states, camera frames and calibration;
- writes `steps.jsonl`, `camera_frames/` and `episode.h5` per episode.

## Command line

```bash
arm-bench start [--arm vx300s]
arm-bench record start [--camera 0 --camera 2] [--rate 30] [--duration 60] [--output DIR]
arm-bench record list
arm-bench cameras               # list cameras
arm-bench setup                 # scan Dynamixel motors and cameras
arm-bench matlab-tune           # control-algorithm tuning window
arm-bench --help                # all commands

ros2 launch controller controller_misa.launch.py launch_drivers:=true use_rviz:=true \
     control_mode:=misa params_file:=/path/gains.yaml log_dir:=/tmp/merlin
python3 tools/evaluate_tracking.py /tmp/merlin/bilateral_R_*.csv --markdown
```

## Tests

```bash
cd arm_bench_dev && PYTHONPATH=src python3 -m pytest tests          # GUI back-ends, launcher
cd ros_ws/src/controller && python3 -m pytest test                 # control math
python3 experiments/bilateral_sim/run.py                           # MuJoCo benchmark
python3 experiments/bilateral_sim/torque_studies.py                # torque studies
```
