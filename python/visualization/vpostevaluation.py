'''
usage:
./python2.7 vpostevaluation.py <data_file>

examples:
./python2.7 vpostevaluation.py post_evaluation.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array
from numpy.matlib import repmat
from numpy.linalg import norm

file_name = sys.argv[1]
labels = sys.argv[2:]

print("preparing data... ")

result_file = open(file_name, 'r')

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[float(value) for value in line[1:]] for line in data]
filtered_data = []
for line in data:
	add = True
	for value in line:
		if value > 1e10:
			add = False
			break

	if add:
		filtered_data += [line]

data = array(filtered_data)

plt.boxplot(data, labels=labels)
plt.title("Post Evaluation for 25000 different Initial Conditions")
plt.show()