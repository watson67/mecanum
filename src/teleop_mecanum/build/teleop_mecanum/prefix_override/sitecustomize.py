import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eswarm/mecanum/src/teleop_mecanum/install/teleop_mecanum'
