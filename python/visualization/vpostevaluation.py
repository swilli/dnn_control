'''
usage:
./python2.7 vpostevaluation.py <data_file>

examples:
./python2.7 vpostevaluation.py post_evaluation.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.matlib import repmat
file_name = sys.argv[1]

print("preparing data... ")

result_file = open(file_name, 'r')

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line[1:]] for line in data]
data = [value for value in data if value < 1e10]
data = array(data)

plt.boxplot(data)
plt.show()