'''
usage:
./python2.7 vbbox.py <data_file> [time] [divergence] 

default time is 3600. default divergence is 0.0001

examples:
./python2.7 vbbox.py trajectory.txt
./python2.7 vbbox.py trajectory.txt 7200.0
./python2.7 vbbox.py trajectory.txt 7200.0 0.1
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, linspace
from numpy.linalg import norm
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid
from visual import ellipsoid, box, rate, color, vector, arrow, scene, sphere, label, display
from scipy.integrate import odeint

divergence = 0.0001
end_time = 3600

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])

if len(sys.argv) > 3:
    divergence = float(sys.argv[3])

print("preparing data... ")

result_file = open(file_name, 'r')

sim_params = [float(value) for value in result_file.readline().split(',')]
frequency = sim_params[7]

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
states = [line.split(',') for line in lines]
states = [[float(value) for value in line] for line in states]
data = array(states)
data = data[0:end_time * frequency,:]


heights = data[:,3:7]
magn_height = norm(heights, axis=1)

def d_dt(y,t0):
	return -divergence * y

t = linspace(0.0, end_time, len(data))
correct_heights = odeint(d_dt, magn_height[0], t)

fig = plt.figure(1)
plt.plot(magn_height)
plt.plot(correct_heights)
plt.legend(["Height", "Expected Height"])
plt.show()
