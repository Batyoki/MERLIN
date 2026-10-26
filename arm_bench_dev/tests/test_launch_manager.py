import unittest

from arm_bench.launch_manager import build_ros2_launch_command


class LaunchManagerTests(unittest.TestCase):
    def test_build_command_without_workspace_keeps_ros2_launch_form(self):
        cmd = build_ros2_launch_command('demo_pkg', 'demo.launch.py', 'robot_model:=rx200')
        self.assertEqual(cmd[0], 'ros2')
        self.assertEqual(cmd[1], 'launch')
        self.assertEqual(cmd[2], 'demo_pkg')
        self.assertEqual(cmd[3], 'demo.launch.py')
        self.assertIn('robot_model:=rx200', cmd)

    def test_build_command_with_workspace_sources_setup_bash(self):
        cmd = build_ros2_launch_command('demo_pkg', 'demo.launch.py', 'robot_model:=rx200', '/tmp/ros_ws')
        self.assertEqual(cmd[0], 'bash')
        self.assertEqual(cmd[1], '-lc')
        self.assertIn('/tmp/ros_ws/install/setup.bash', cmd[2])
        self.assertIn('ros2 launch demo_pkg demo.launch.py robot_model:=rx200', cmd[2])


if __name__ == '__main__':
    unittest.main()
