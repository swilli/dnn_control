'''
usage:
./python2.7 vheight.py <data_file> [time] [x_axis_is_time]

default time is 3600. default x_axis_is_time is False 

examples:
./python2.7 vheight.py trajectory.txt
./python2.7 vheight.py trajectory.txt 7200.0
./python2.7 vheight.py trajectory.txt 7200.0 True
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, linspace, arange
from numpy.linalg import norm
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
from scipy.integrate import odeint

import seaborn as sns
sns.set_context("notebook", font_scale=4.0, rc={"lines.linewidth": 5, "grid.linewidth": 2.0})
sns.set_style("whitegrid")

x_axis_is_time = False
end_time = 3600

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])
    if len(sys.argv) > 3:
        x_axis_is_time = sys.argv[3] == 'True'

print("preparing data... ")

result_file = open(file_name, 'r')

sim_params = [float(value) for value in result_file.readline().split(',')]
frequency = sim_params[7]
semi_axis = sim_params[0:3]
density = sim_params[3]
angular_velocity_xz = [sim_params[4], sim_params[5]]
time_bias = sim_params[6]

asteroid = Asteroid(semi_axis, density, angular_velocity_xz, time_bias)
asteroid_period = asteroid.angular_velocity_period()

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
states = [line.split(',') for line in lines]
states = [[float(value) for value in line] for line in states]
data = array(states)
data = data[0:end_time * frequency,:]

times = arange(0.0, len(data) * frequency, frequency)

heights = data[:,3:7]

x_axis_label = "Time [s]"

if not x_axis_is_time:
    x_axis_label = "Period"
    val = 0
    ticks = []
    while val < times[-1]:
        ticks.append(val)
        val += asteroid_period

    ticks = array(ticks)

magn_height = norm(heights, axis=1)
plt.plot(magn_height)
plt.xlabel(x_axis_label)
plt.ylabel('Altitude [m]')
plt.ticklabel_format(style='sci', axis='y', scilimits=(0,0))
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
#plt.yscale('log')
if not x_axis_is_time:
    plt.xticks(ticks, [val for val in range(len(ticks))])

#plt.xlim(plt.ylim()[0], len(heights) * 1.0 / frequency)
plt.show()
#plt.savefig(file_name.replace(".txt", ".svg"))
