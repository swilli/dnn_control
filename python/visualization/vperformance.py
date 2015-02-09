'''
usage:
./python2.7 vperformance.py <data_file> [time] 

default time is 3600

examples:
./python2.7 vperformance.py performance.txt 7200.0

'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.matlib import repmat


end_time = 3600.0

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])

print("preparing data... ")

result_file = open(file_name, 'r')

simulation_seed = int(result_file.readline())
target_position = array([float(value) for value in result_file.readline().split(',')])

lines = result_file.readlines()
num_samples = len(lines)
data = [line.split(',') for line in lines]
for i in range(num_samples):
    for j in range(len(data[i])):
        data[i][j] = float(data[i][j])

result_file.close()
data = array(data)
times = data[:, 0]
times = [val for val in times if val <= end_time]
num_samples = len(times)
positions = data[0:num_samples, 1:4]
thrusts = data[0:num_samples, 4:7]

errors = repmat(target_position, num_samples, 1) - positions


fig = plt.figure(1)
fig.subplots_adjust(hspace=1.0)

plt.subplot(321)
plt.plot(times, errors[:, 0])
plt.title('X Offset vs Time')
plt.xlabel('time [s]')
plt.ylabel('offset [m]')

plt.subplot(322)
plt.step(times, thrusts[:, 0])
plt.title('X Thrust vs Time')
plt.xlabel('time [s]')
plt.ylabel('thrust [N]')

plt.subplot(323)
plt.plot(times, errors[:, 1])
plt.title('Y Offset vs Time')
plt.xlabel('time [s]')
plt.ylabel('offset [m]')

plt.subplot(324)
plt.step(times, thrusts[:, 1])
plt.title('Y Thrust vs Time')
plt.xlabel('time [s]')
plt.ylabel('thrust [N]')

plt.subplot(325)
plt.plot(times, errors[:, 2])
plt.title('Z Offset vs Time')
plt.xlabel('time [s]')
plt.ylabel('offset [m]')

plt.subplot(326)
plt.step(times, thrusts[:, 2])
plt.title('Z Thrust vs Time')
plt.xlabel('time [s]')
plt.ylabel('thrust [N]')

plt.show()



