import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kan/lab/course/unitree_go2_ros2_jazzy1/install/yolo_ros'
