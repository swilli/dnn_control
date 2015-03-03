'''
usage:
./python2.7 vic.py <data_file>


examples:
./python2.7 vic.py ic.txt
./python2.7 vic.py ic.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, sum, pi, mean
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

height = data[:, 0]
velocity = norm(data[:, 1:4] + data[:, 4:7], axis=1)
velocity_vertical = data[:, 1:4]
velocity_horizontal = data[:, 4:7]
frequency = 2.0 * pi / norm(data[:, 7:10], axis = 1)
fig = plt.figure(1)

fig.subplots_adjust(hspace=1.0)

scaling = 1e3

plt.subplot(15,1,1)
plt.hist(height, num_bins)
plt.title('Height')

plt.subplot(15,1,2)
plt.hist(velocity, num_bins)
plt.title('Velocity')

plt.subplot(15,1,3)
plt.hist(velocity_vertical[:, 0], num_bins)
plt.title('Velocity Vertical X')

plt.subplot(15,1,4)
plt.hist(velocity_vertical[:,1], num_bins)
plt.title('Velocity Vertical Y')

plt.subplot(15,1,5)
plt.hist(velocity_vertical[:,2], num_bins)
plt.title('Velocity Vertical Z')

plt.subplot(15,1,6)
plt.hist(velocity_horizontal[:,0], num_bins)
plt.title('Velocity Horizontal X')

plt.subplot(15,1,7)
plt.hist(velocity_horizontal[:,1], num_bins)
plt.title('Velocity Horizontal Y')

plt.subplot(15,1,8)
plt.hist(velocity_horizontal[:,2], num_bins)
plt.title('Velocity Horizontal Z')

plt.subplot(15,1,9)
plt.hist(velocity_vertical[:, 0] / height * scaling, num_bins)
plt.title('Velocity Vertical X / Height')

plt.subplot(15,1,10)
plt.hist(velocity_vertical[:,1] / height * scaling, num_bins)
plt.title('Velocity Vertical Y / Height')

plt.subplot(15,1,11)
plt.hist(velocity_vertical[:,2] / height * scaling, num_bins)
plt.title('Velocity Vertical Z / Height')

plt.subplot(15,1,12)
plt.hist(velocity_horizontal[:,0] / height * scaling, num_bins)
plt.title('Velocity Horizontal X / Height')

plt.subplot(15,1,13)
plt.hist(velocity_horizontal[:,1] / height * scaling, num_bins)
plt.title('Velocity Horizontal Y / Height')

plt.subplot(15,1,14)
plt.hist(velocity_horizontal[:,2] / height * scaling, num_bins)
plt.title('Velocity Horizontal Z / Height')


plt.subplot(15,1,15)
plt.hist(frequency, num_bins)
plt.title('Frequency')

#print(min(height))
#print(max(height))
#print(min(velocity_vertical / height))
#print(max(velocity_vertical / height))
#print(min(velocity_horizontal / height))
#print(max(velocity_horizontal / height))
#print(min(velocity))
#print(max(velocity))
#print(mean(frequency))
plt.show()




