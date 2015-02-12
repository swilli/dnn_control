'''
usage:
./python2.7 vconvexity.py <data_file>

examples:
./python2.7 vconvexity.py convexity.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, nan, diff
from numpy.matlib import repmat
from numpy.linalg import norm

file_name = sys.argv[1]
print("preparing data... ")

result_file = open(file_name, 'r')
seed, dimension = [int(value) for value in result_file.readline().split(',')]
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

def discontinuous(x):
    x[diff(x) >= 0.5] = nan
    return x

plt.plot(data[:,0], discontinuous(data[:,1]))
plt.title("Convexity of problem (seed: {0}, dimension: {1})".format(seed, dimension))
plt.xlabel('Parameter value')
plt.ylabel('Fitness')
plt.show()