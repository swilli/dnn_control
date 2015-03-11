'''
usage:
./python2.7 vpostevaluation.py <data_file> [threshold] [skip_outliers]

default for threshold is 1.0, default for skip_outliers is False

examples:
./python2.7 vpostevaluation.py post_evaluation.txt
./python2.7 vpostevaluation.py post_evaluation.txt True
'''

import sys
import matplotlib.pyplot as plt
from numpy import array, mean, minimum, maximum,std, sqrt
from numpy.matlib import repmat
from numpy.linalg import norm

file_name = sys.argv[1]
skip_outliers = False
threshold = 1.0

if len(sys.argv) > 2:
	threshold = float(sys.argv[2])
	if len(sys.argv) > 3:
		skip_outliers = bool(sys.argv[3])

print("preparing data... ")

result_file = open(file_name, 'r')

lines = result_file.readlines()
result_file.close()

num_samples = len(lines)
data = [line.split(',') for line in lines]
data = [[int(line[0])] + [float(value) for value in line[1:]] for line in data]
filtered_data = []
outliers = 0
worst_case_seed = -1
worst_case_value = 0.0
for line in data:
	mean_error = line[1]
	if mean_error > worst_case_value:
		worst_case_value = mean_error
		worst_case_seed = line[0]

	if mean_error >= threshold:
		outliers += 1
		print(line)
        if skip_outliers:
        	continue

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

print("Outliers: {0}".format(outliers))

subtitle1 = "SSME: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssme_mean, ssme_stdev, ssme_min, ssme_max)
subtitle2 = "SSDE: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssde_mean, ssde_stdev, ssde_min, ssde_max)

print(subtitle1)
print(subtitle2)
print("Worst case seed: {0}".format(worst_case_seed))

plt.boxplot(data[:, 1])
#plt.title("Post Evaluation for {0} different Initial Conditions".format(num_samples)) # + '\n' + subtitle1 + '\n' + subtitle2)
#plt.xlabel('Controller')
plt.ylabel('Utility')
plt.show()
