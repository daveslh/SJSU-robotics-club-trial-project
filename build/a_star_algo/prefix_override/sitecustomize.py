import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/davidhuang/Desktop/project_root/install/a_star_algo'
