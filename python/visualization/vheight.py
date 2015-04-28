'''
usage:
./python2.7 vbbox.py <data_file> [time] [x_axis_is_time]

default time is 3600. default x_axis_is_time is False 

examples:
./python2.7 vbbox.py trajectory.txt
./python2.7 vbbox.py trajectory.txt 7200.0
./python2.7 vbbox.py trajectory.txt 7200.0 0.1
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, linspace, arange
from numpy.linalg import norm
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
from visual import ellipsoid, box, rate, color, vector, arrow, scene, sphere, label, display
from scipy.integrate import odeint

import seaborn as sns
sns.set_context("notebook", font_scale=2.5, rc={"lines.linewidth": 2.5})

x_axis_is_time = False
end_time = 3600

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])
    if len(sys.argv) > 3:
        x_axis_is_time = sys.argv[4] == 'True'

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

x_axis_label = "time [s]"

if not x_axis_is_time:
    x_axis_label = "periods"
    val = 0
    ticks = []
    while val < times[-1]:
        ticks.append(val)
        val += asteroid_period

    ticks = array(ticks)

magn_height = norm(heights, axis=1)

fig = plt.figure(1)
plt.plot(magn_height)
plt.xlabel(x_axis_label)
plt.ylabel('Altitude [m]')
if not x_axis_is_time:
    plt.xticks(ticks, [val for val in range(len(ticks))])

plt.savefig(file_name.replace(".txt", ".svg"))
