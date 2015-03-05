'''
usage:
./python2.7 vbbox.py <data_file> [time] 

default time is 3600

examples:
./python2.7 vbbox.py evaluation.txt
./python2.7 vbbox.py evaluation.txt 7200.0
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.linalg import norm
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid


end_time = 3600.0

file_name = sys.argv[1]
if len(sys.argv) > 2:
    end_time = float(sys.argv[2])

print("preparing data... ")

result_file = open(file_name, 'r')

simulation_seed = int(result_file.readline())
target_position = array([float(value) for value in result_file.readline().split(',')])
asteroid_params = [float(value) for value in result_file.readline().split(',')]
asteroid = Asteroid(asteroid_params[0:3], asteroid_params[3], asteroid_params[4:6], asteroid_params[6])

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

surface_points = array([asteroid.intersect_line_to_center_from_position(pos.tolist()) for pos in positions])
norm_positions = norm(positions, axis=1)
norm_surface_points = norm(surface_points, axis=1)

norm_dist = norm_positions / norm_surface_points
fig = plt.figure(1)
plt.plot(times, norm_dist)
plt.show()



