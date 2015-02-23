'''
usage:
./python2.7 vpostevaluation.py <data_file>

examples:
./python2.7 vpostevaluation.py post_evaluation.txt
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, mean, minimum, maximum,std, sqrt
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
data = [[int(line[0])] + [float(value) for value in line[1:]] for line in data]
filtered_data = []
for line in data:
    if line[1] >= 1.0:
        print(line[0])
        continue
    else:
	    filtered_data += [line]

data = array(filtered_data)

mean_errors = data[:,1]
ssme_mean = mean(mean_errors,axis=0)
ssme_stdev = std(mean_errors, axis=0)
ssme_min = min(mean_errors)
ssme_max = max(mean_errors)

delta_errors = data[:,3] - data[:,2]
ssde_mean = mean(delta_errors,axis=0)
ssde_stdev = std(delta_errors, axis=0)
ssde_min = min(delta_errors)
ssde_max = max(delta_errors)

print("SSME: mean={0}, stdev={1}, min={2}, max={3}".format(ssme_mean, ssme_stdev, ssme_min, ssme_max))
print("SSDE: mean={0}, stdev={1}, min={2}, max={3}".format(ssde_mean, ssde_stdev, ssde_min, ssde_max))

plt.boxplot(data[:, 1], labels=labels)
plt.title("Post Evaluation for 25000 different Initial Conditions")
plt.xlabel('Controller')
plt.ylabel('Fitness')
plt.show()
