'''
usage:
./python2.7 vic.py <data_file>


examples:
./python2.7 vic.py ic.txt
./python2.7 vic.py ic.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, sum, pi, mean, std
from numpy.matlib import repmat
from numpy.linalg import norm


num_bins = 50

file_name = sys.argv[1]

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line] for line in data]
data = array(data)

semi_axis_a = data[:, 0]
semi_axis_b = data[:, 1]
semi_axis_c = data[:, 2]
density = data[:, 3]
omega = data[:, 4:7]
altitude = data[:, 7]
mass = data[:, 8]
velocity = norm(data[:, 9:12], axis=1)

frequency = 2.0 * pi / norm(omega, axis = 1)

def min_max_mean_stdev(data):
	return min(data), max(data), mean(data), std(data)

min_frequency, max_frequency, mean_frequency, stdev_frequency = min_max_mean_stdev(frequency)
min_semi_axis_a, max_semi_axis_a, mean_semi_axis_a, stdev_semi_axis_a = min_max_mean_stdev(semi_axis_a)
min_semi_axis_b, max_semi_axis_b, mean_semi_axis_b, stdev_semi_axis_b = min_max_mean_stdev(semi_axis_b)
min_semi_axis_c, max_semi_axis_c, mean_semi_axis_c, stdev_semi_axis_c = min_max_mean_stdev(semi_axis_c)
min_density, max_density, mean_density, stdev_density = min_max_mean_stdev(density)
min_altitude, max_altitude, mean_altitude, stdev_altitude = min_max_mean_stdev(altitude)
min_mass, max_mass, mean_mass, stdev_mass = min_max_mean_stdev(mass)
min_velocity, max_velocity, mean_velocity, stdev_velocity = min_max_mean_stdev(velocity)

def print_stats(name, val_min, val_max, val_mean, val_stdev):
	print("{0}: min: {1}, max: {2}, mean: {3}, stdev: {4}".format(name, val_min, val_max, val_mean, val_stdev))

print("# samples: {0}".format(num_samples))
print_stats("frequency", min_frequency, max_frequency, mean_frequency, stdev_frequency)
print_stats("semi_axis_a", min_semi_axis_a, max_semi_axis_a, mean_semi_axis_a, stdev_semi_axis_a)
print_stats("semi_axis_b", min_semi_axis_b, max_semi_axis_b, mean_semi_axis_b, stdev_semi_axis_b)
print_stats("semi_axis_c", min_semi_axis_c, max_semi_axis_c, mean_semi_axis_c, stdev_semi_axis_c)
print_stats("density", min_density, max_density, mean_density, stdev_density)
print_stats("normalized altitude", min_altitude, max_altitude, mean_altitude, stdev_altitude)
print_stats("mass", min_mass, max_mass, mean_mass, stdev_mass)
print_stats("velocity", min_velocity, max_velocity, mean_velocity, stdev_velocity)

