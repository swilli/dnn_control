'''
usage:
./python2.7 vobjfun.py <data_file>


examples:
./python2.7 vobjfun.py task.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
import matplotlib
import matplotlib.ticker as mtick

import seaborn as sns
sns.set_context("notebook", font_scale=4, rc={"lines.linewidth": 5})
sns.set_style("whitegrid")

file_name = sys.argv[1]

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
lines = [line for line in lines if "generation" in line]
lines = [line.split(" ") for line in lines]
lines = [[val for val in line if val != ""] for line in lines]
lines = [line[2] for line in lines]
values = [float(val) for val in lines]
minimas = []
for val in values:
	if len(minimas) == 0:
		minimas.append(val)
	else:
		if val < minimas[-1]:
			minimas.append(val)
		else:
			minimas.append(minimas[-1])

data = array(values)

plt.plot(data)
plt.xlabel("Generation")
plt.ylabel("Fitness")
plt.xscale("log")
ax = plt.gca()
ax.yaxis.set_major_formatter(mtick.FormatStrFormatter('%.2e'))
plt.show()

