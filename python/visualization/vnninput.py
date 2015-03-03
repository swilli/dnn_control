'''
usage:
./python2.7 vnninput.py <data_file> [time]

default time is 3600

examples:
./python2.7 vnninput.py nn_input.txt
./python2.7 vnninput.py nn_input.txt 7200.0
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.matlib import repmat
from numpy.linalg import norm


end_time = 3600.0

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

times = data[:, 0]
times = [val for val in times if val <= end_time]
num_samples = len(times)
heights = data[0:num_samples, 1]
velocities = data[0:num_samples, 2]
nn_input = data[0:num_samples, 3:]

fig = plt.figure(1)

fig.subplots_adjust(hspace=1.0)

plt.subplot(311)
plt.plot(times, heights)
plt.title('Height vs Time')
plt.xlabel('time [s]')
plt.ylabel('height [m]')

plt.subplot(312)
plt.plot(times, velocities)
plt.title('Velocity vs Time')
plt.xlabel('time [s]')
plt.ylabel('velocity [m/s]')

plt.subplot(313)
plt.plot(times, nn_input)
plt.title('NN input vs Time')
plt.xlabel('time [s]')
plt.ylabel('NN input')
plt.show()



