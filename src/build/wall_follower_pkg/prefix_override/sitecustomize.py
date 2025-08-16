import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aryanganesh/dev_ws/src/install/wall_follower_pkg'
