'''
usage:
./python2.7 vevaluation.py <data_file> [time] [obj_fun]

default time is 3600, default obj_fun is False

examples:
./python2.7 vevaluation.py evaluation.txt
./python2.7 vevaluation.py evaluation.txt 7200.0
./python2.7 vevaluation.py evaluation.txt 7200.0 true
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.matlib import repmat
from numpy.linalg import norm


end_time = 3600.0
obj_fun = False

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])
    if len(sys.argv) > 3:
        obj_fun = bool(sys.argv[3])

print("preparing data... ")

result_file = open(file_name, 'r')

simulation_seed = int(result_file.readline())
target_position = array([float(value) for value in result_file.readline().split(',')])
asteroid_params = result_file.readline()

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

times = data[:, 0]
times = [val for val in times if val <= end_time]
num_samples = len(times)
positions = data[0:num_samples, 1:4]
thrusts = data[0:num_samples, 4:7]
velocities = data[0:num_samples, 7:10]

errors = repmat(target_position, num_samples, 1) - positions

fig = plt.figure(1)
if obj_fun:
    fitness = norm(errors, axis=1)

    plt.plot(times, fitness)
    plt.title('Fitness vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('Fitness')

else:
    fig.subplots_adjust(hspace=1.0)

    plt.subplot(331)
    plt.plot(times, errors[:, 0])
    plt.title('X Offset vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('offset [m]')

    plt.subplot(332)
    plt.plot(times, errors[:, 1])
    plt.title('Y Offset vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('offset [m]')

    plt.subplot(333)
    plt.plot(times, errors[:, 2])
    plt.title('Z Offset vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('offset [m]')

    plt.subplot(334)
    plt.plot(times, velocities[:, 0])
    plt.title('X Velocity vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')

    plt.subplot(335)
    plt.plot(times, velocities[:, 1])
    plt.title('Y Velocity vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')
    
    plt.subplot(336)
    plt.plot(times, velocities[:, 2])
    plt.title('Z Velocity vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('velocity [m/s]')

    plt.subplot(337)
    plt.step(times, thrusts[:, 0])
    plt.title('X Thrust vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('thrust [N]')

    plt.subplot(338)
    plt.step(times, thrusts[:, 1])
    plt.title('Y Thrust vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('thrust [N]')

    plt.subplot(339)
    plt.step(times, thrusts[:, 2])
    plt.title('Z Thrust vs Time')
    plt.xlabel('time [s]')
    plt.ylabel('thrust [N]')

plt.show()



