'''
usage:
./python2.7 vevaluation.py <data_file> [start_time] [end_time] [x_axis_is_time] [show_magnitude]

default start_time is 0.0, default end_time is 3600, default x_axis_is_time is True, default show_magnitude is True

examples:
./python2.7 vevaluation.py evaluation.txt
./python2.7 vevaluation.py evaluation.txt 0.0 7200.0
./python2.7 vevaluation.py evaluation.txt 0.0 7200.0 true
./python2.7 vevaluation.py evaluation.txt 0.0 7200.0 true true
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, pi
from numpy.matlib import repmat
from numpy.linalg import norm
import matplotlib.ticker as mtick

from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid

import seaborn as sns
sns.set_context("notebook", font_scale=3, rc={"lines.linewidth": 2.5, "grid.linewidth": 2.0})
sns.set_style("whitegrid")

start_time = 0.0
end_time = 3600.0
x_axis_is_time = True
show_magnitude = True


file_name = sys.argv[1]
if len(sys.argv) > 2:
    start_time = float(sys.argv[2])
    if len(sys.argv) > 3:
        end_time = float(sys.argv[3])
        if len(sys.argv) > 4:
            x_axis_is_time = sys.argv[4] == 'True'
            if len(sys.argv) > 5:
                show_magnitude = sys.argv[5] == 'True'


print("preparing data... ")

result_file = open(file_name, 'r')

simulation_seed = int(result_file.readline())
target_position = array([float(value) for value in result_file.readline().split(',')])
asteroid_params = result_file.readline()
asteroid_params = [float(value) for value in asteroid_params.split(',')]
semi_axis = asteroid_params[0:3]
density = asteroid_params[3]
angular_velocity_xz = [asteroid_params[4], asteroid_params[5]]
time_bias = asteroid_params[6]

asteroid = Asteroid(semi_axis, density, angular_velocity_xz, time_bias)
asteroid_period = asteroid.angular_velocity_period()

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

times = data[:, 0]
start_index = 0
end_index = len(times)
start_index_set = False

for i in range(len(times)):
    val = times[i]
    if val >= start_time:
        if not start_index_set:
            start_index = i
            start_index_set = True

    if val > end_time:
        end_index = i - 1
        break

times = times[start_index:end_index]
num_samples = len(times)
x_axis_label = "Time [s]"

if not x_axis_is_time:
    x_axis_label = "Period"
    val = 0
    ticks = []
    while val < times[-1]:
        ticks.append(val)
        val += asteroid_period

    ticks = array(ticks)

print(times[-1])
positions = data[start_index:end_index, 1:4]
thrusts = data[start_index:end_index, 4:7]
velocities = data[start_index:end_index, 7:10]

errors = repmat(target_position, num_samples, 1) - positions

fig = plt.figure(1)
fig.subplots_adjust(hspace=1.0, wspace=0.4)

if show_magnitude:
    errors = norm(errors, axis=1)
    thrusts = norm(thrusts, axis=1)
    velocities = norm(velocities, axis=1)

    plt.subplot(311)
    plt.plot(times, errors)
    plt.title('Offset vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('offset [m]')
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(312)
    plt.plot(times, velocities)
    plt.title('Velocity vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('velocity [m/s]')
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(313)
    plt.plot(times, thrusts)
    plt.title('Thrust vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('thrust [N]')
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

else:
    plt.subplot(331)
    plt.plot(times, errors[:, 0])
    plt.title('X Offset vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Offset [m]')
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(332)
    plt.plot(times, errors[:, 1])
    plt.title('Y Offset vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Offset [m]')
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(333)
    plt.plot(times, errors[:, 2])
    plt.title('Z Offset vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Offset [m]')
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(334)
    plt.plot(times, velocities[:, 0])
    plt.title('X Velocity vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Velocity [m/s]')
    plt.ylim(-1,1)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(335)
    plt.plot(times, velocities[:, 1])
    plt.title('Y Velocity vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Velocity [m/s]')
    plt.ylim(-1,1)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(336)
    plt.plot(times, velocities[:, 2])
    plt.title('Z Velocity vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Velocity [m/s]')
    plt.ylim(-1,1)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(337)
    plt.step(times, thrusts[:, 0])
    plt.title('X Thrust vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Thrust [N]')
    plt.ylim(-30,30)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(338)
    plt.step(times, thrusts[:, 1])
    plt.title('Y Thrust vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Thrust [N]')
    plt.ylim(-30,30)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

    plt.subplot(339)
    plt.step(times, thrusts[:, 2])
    plt.title('Z Thrust vs Time')
    plt.xlabel(x_axis_label)
    plt.ylabel('Thrust [N]')
    plt.ylim(-30,30)
    plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
    plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
    if not x_axis_is_time:
        plt.xticks(ticks, [val for val in range(len(ticks))])

plt.show()



