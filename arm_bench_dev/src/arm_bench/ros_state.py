"""
Live joint states from ROS 2, shared by the motor-status tab and the recorder.

Subscribes to every ``*/joint_states`` topic (one per Interbotix arm, e.g.
/masterR/joint_states, /slaveR/joint_states) in a background thread and keeps
the latest message per arm. Works without ROS installed: ``available`` is then
False and every query returns nothing.

The Interbotix xs_sdk publishes JointState.effort as motor current in mA.
"""

import threading
import time
from typing import Dict, List, Optional

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class ArmJointState:
    __slots__ = ("arm", "names", "position", "velocity", "effort_mA", "stamp", "received")

    def __init__(self, arm, names, position, velocity, effort, stamp):
        self.arm = arm
        self.names = list(names)
        self.position = list(position)
        self.velocity = list(velocity)
        self.effort_mA = list(effort)
        self.stamp = stamp
        self.received = time.time()

    def as_dict(self) -> dict:
        return dict(names=self.names, position=self.position, velocity=self.velocity,
                    effort_mA=self.effort_mA, stamp=self.stamp)


class RosJointStateSource:
    """Background subscriber to all */joint_states topics."""

    _instance = None

    @classmethod
    def shared(cls) -> "RosJointStateSource":
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self, discovery_period: float = 2.0):
        self.available = False
        self._latest: Dict[str, ArmJointState] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._node = None
        self._subscribed = set()
        self._discovery_period = discovery_period
        if not ROS_AVAILABLE:
            return
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = Node("merlin_gui_state")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self.available = True
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._thread.start()
        except Exception as exc:  # ROS installed but no daemon / bad env
            print(f"ROS joint-state source unavailable: {exc}")
            self.available = False

    def _discover(self):
        for topic, types in self._node.get_topic_names_and_types():
            if topic.endswith("/joint_states") and "sensor_msgs/msg/JointState" in types \
                    and topic not in self._subscribed:
                arm = topic[: -len("/joint_states")].strip("/") or "robot"
                self._node.create_subscription(
                    JointState, topic, lambda msg, a=arm: self._on_msg(a, msg), 10)
                self._subscribed.add(topic)

    def _on_msg(self, arm: str, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        state = ArmJointState(arm, msg.name, msg.position, msg.velocity, msg.effort, stamp)
        with self._lock:
            self._latest[arm] = state

    def _spin(self):
        last = 0.0
        while not self._stop.is_set():
            if time.time() - last > self._discovery_period:
                try:
                    self._discover()
                except Exception:
                    pass
                last = time.time()
            self._executor.spin_once(timeout_sec=0.05)

    def arms(self, max_age: float = 1.0) -> List[str]:
        now = time.time()
        with self._lock:
            return sorted(a for a, s in self._latest.items() if now - s.received <= max_age)

    def latest(self, max_age: float = 1.0) -> Dict[str, ArmJointState]:
        now = time.time()
        with self._lock:
            return {a: s for a, s in self._latest.items() if now - s.received <= max_age}

    def get(self, arm: str) -> Optional[ArmJointState]:
        with self._lock:
            return self._latest.get(arm)

    def shutdown(self):
        self._stop.set()
