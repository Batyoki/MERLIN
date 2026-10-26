import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/install/bilateral_teleop'
