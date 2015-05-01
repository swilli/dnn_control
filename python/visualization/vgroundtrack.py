'''
usage:
./python2.7 vgroundtrack.py <data_file> [time] 

default time is 3600. default

examples:
./python2.7 vgroundtrack.py trajectory.txt
./python2.7 vgroundtrack.py trajectory.txt 7200.0
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, linspace, arange
from numpy.linalg import norm
from numpy import pi
import math
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
from scipy.integrate import odeint

import seaborn as sns
sns.set_context("notebook", font_scale=5, rc={"lines.linewidth": 4})
sns.set_style("whitegrid")

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

positions = data[:,0:3]
heights = data[:,3:7]

surface_positions = positions - heights

latitude_longitudes = [asteroid.latitude_and_longitude_at_position(pos.tolist()) for pos in surface_positions]
latitudes = [val for val, _ in latitude_longitudes]
longitudes = [val for _, val in latitude_longitudes]

latitudes = [math.cos(val) * 90.0 for val in latitudes]
longitudes = [val / (2.0 * pi) * 360 for val in longitudes]

plt.plot(longitudes, latitudes)
plt.plot(longitudes[0],latitudes[0], 'go', markersize=15, label="Start")
plt.plot(longitudes[-1], latitudes[-1], 'ro', markersize=15, label="End")
plt.legend()
plt.xlabel("Longitude [deg]")
plt.ylabel("Latitude [deg]")
plt.show()