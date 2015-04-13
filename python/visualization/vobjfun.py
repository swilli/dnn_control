'''
usage:
./python2.7 vobjfun.py <data_file>


examples:
./python2.7 vobjfun.py task.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
import matplotlib

matplotlib.rcParams.update({'font.size': 22})


file_name = sys.argv[1]

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
lines = [line for line in lines if "generation" in line]
lines = [line.split(" ") for line in lines]
lines = [[val for val in line if val != ""] for line in lines]
lines = [line[2] for line in lines]
lines = [float(val) for val in lines]
data = array(lines)

plt.plot(lines)
plt.title("Mean Error vs Generation")
plt.xlabel("generation")
plt.ylabel("mean error [m]")
plt.show()

