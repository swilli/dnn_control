'''
usage:
./python2.7 vconvexities.py <path_to_data_files>

examples:
./python2.7 vconvexities.py results/convexity/
'''

from os import listdir
from vconvexity import visualize_convexity
import sys

path = sys.argv[1]
file_names = listdir(path)
file_paths = [path + name for name in file_names if name.endswith(".txt")]

for file_path in file_paths:
	visualize_convexity(file_path, True)