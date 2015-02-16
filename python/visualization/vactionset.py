'''
usage:
./python2.7 vactionset.py <data_file>

examples:
./python2.7 vactionset.py lspi_action_set.txt

'''

import sys
import matplotlib.pyplot as plt
from numpy import array, zeros
from numpy.matlib import repmat
from numpy.linalg import norm
from mayavi import mlab


file_name = sys.argv[1]

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

root = zeros([num_samples, 3])

mlab.figure()
mlab.quiver3d(root[:,0], root[:,1], root[:,2], data[:,0], data[:,1], data[:,2], mode="2darrow")
mlab.show()


