"""Helpers for launching the ROS 2 MPC controller from the application."""

import os
import shlex
import subprocess
from typing import Optional, Sequence


def build_mpc_launch_command(workspace_path: str) -> list:
    """Build a shell command that sources the ROS workspace and launches the controller launch file."""
    workspace_path = os.path.abspath(workspace_path or "")
    setup_script = os.path.join(workspace_path, "install", "setup.bash")
    shell_script = (
        "source /opt/ros/$ROS_DISTRO/setup.bash 2>/dev/null || true; "
        f"source {shlex.quote(setup_script)}; "
        "ros2 launch controller controller_misa.launch.py"
    )
    return ["bash", "-lc", shell_script]


def launch_mpc_controller(workspace_path: str, env: Optional[dict] = None):
    """Start the controller launch file in a background process."""
    command = build_mpc_launch_command(workspace_path)
    workspace_dir = os.path.abspath(workspace_path or ".")
    process_env = os.environ.copy()
    if env:
        process_env.update(env)
    return subprocess.Popen(
        command,
        cwd=workspace_dir,
        env=process_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
