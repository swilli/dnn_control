'''
usage:
./python2.7 vvelheight.py <data_file> c [time] 

default time is 3600

examples:
./python2.7 vvelheight.py vh.txt 0.1
./python2.7 vvelheight.py vh.txt 0.1 7200.0
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, ones
from numpy.linalg import norm
from boost_asteroid import boost_asteroid
Asteroid = boost_asteroid.BoostAsteroid


end_time = 3600.0
divergence = 0.0

file_name = sys.argv[1]
divergence = float(sys.argv[2])

if len(sys.argv) > 3:
    end_time = float(sys.argv[3])

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
error_divergence = data[0:num_samples, 1]
optical_flow = data[0:num_samples, 2]

divergence = divergence * ones([num_samples,1])
fig = plt.figure(1)
plt.plot(times, error_divergence)
plt.plot(times, optical_flow)
plt.plot(times, divergence)
plt.legend(["Divergence", "Ventrical flow", "Constant divergence"])
plt.show() 