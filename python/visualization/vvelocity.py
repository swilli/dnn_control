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

divergence = data[0:num_samples, 1:4]
optical_flow = data[0:num_samples,4:7]

fig = plt.figure(1)
plt.subplot(231)
plt.plot(times, divergence[:, 0])
plt.title('X divergence vs Time')
plt.xlabel('time [s]')
plt.ylabel('divergence [1/s]')

plt.subplot(232)
plt.plot(times, divergence[:, 1])
plt.title('Y divergence vs Time')
plt.xlabel('time [s]')
plt.ylabel('divergence [1/s]')

plt.subplot(233)
plt.plot(times, divergence[:, 2])
plt.title('Z divergence vs Time')
plt.xlabel('time [s]')
plt.ylabel('divergence [1/s]')

plt.subplot(234)
plt.plot(times, optical_flow[:, 0])
plt.title('X optical flow vs Time')
plt.xlabel('time [s]')
plt.ylabel('optical flow [1/s]')

plt.subplot(235)
plt.plot(times, optical_flow[:, 1])
plt.title('Y optical flow vs Time')
plt.xlabel('time [s]')
plt.ylabel('optical flow [1/s]')

plt.subplot(236)
plt.plot(times, optical_flow[:, 2])
plt.title('Z optical flow vs Time')
plt.xlabel('time [s]')
plt.ylabel('optical flow [1/s]')

plt.show()