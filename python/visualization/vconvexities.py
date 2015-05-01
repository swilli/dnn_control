'''
usage:
./python2.7 vconvexities.py <path_to_data_files>

examples:
./python2.7 vconvexities.py results/convexity/
'''

from os import listdir
import matplotlib.pyplot as plt
from numpy import array, nan, diff
from numpy.matlib import repmat
import matplotlib.ticker as mtick
from numpy.linalg import norm
from vconvexity import visualize_convexity
import sys

import seaborn as sns
sns.set_context("notebook", font_scale=3, rc={"lines.linewidth": 2.5})
sns.set_style("whitegrid")

'''
path = sys.argv[1]
file_names = listdir(path)
file_paths = [path + name for name in file_names if name.endswith(".txt")]

for file_path in file_paths:
	visualize_convexity(file_path, True)
'''


file_names = sys.argv[1:3]

print("preparing data... ")

def discontinuous(data):
    data[diff(data) >= 0.5] = nan
    return data

total_data = []
for file_name in file_names:
	result_file = open(file_name, 'r')
	seed, dimension = [int(value) for value in result_file.readline().split(',')]
	lines = result_file.readlines()
	result_file.close()

	num_samples = len(lines)
	data = [line.split(',') for line in lines]
	data = [[float(value) for value in line] for line in data]
	data = array(data)
	total_data += [data]

fig = plt.figure(1)
fig.subplots_adjust(hspace=1.0, wspace=0.4)

plt.subplot(121)
plt.plot(total_data[0][:,0], discontinuous(total_data[0][:,1]))
#plt.title("Convexity of problem (seed: {0}, dimension: {1})".format(seed, dimension))
plt.xlabel('Parameter Value')
plt.xlim(-5,5)
plt.ylabel('Negative Utility')
plt.ylim(545000, 547500)
ax = plt.gca()
ax.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.2e'))

plt.subplot(122)
plt.plot(total_data[1][:,0], discontinuous(total_data[1][:,1]))
#plt.title("Convexity of problem (seed: {0}, dimension: {1})".format(seed, dimension))
plt.xlabel('Parameter Value')
plt.xlim(-5,5)
plt.ylabel('Negative Utility')
plt.ylim(545000, 546000)
ax = plt.gca()
ax.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.2e'))
plt.show()