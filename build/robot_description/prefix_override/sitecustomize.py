import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/long/delta_robot_prj/demo_repo2305/install/robot_description'
