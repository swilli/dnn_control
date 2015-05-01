'''
usage:
./python2.7 vpostevaluation.py <data_file> [threshold] [skip_outliers]

default for threshold is 1.0, default for skip_outliers is False

examples:
./python2.7 vpostevaluation.py post_evaluation.txt
./python2.7 vpostevaluation.py post_evaluation.txt True
'''

import sys
import matplotlib
import matplotlib.pyplot as plt
from numpy import array, mean, minimum, maximum,std, sqrt
from numpy.matlib import repmat
from numpy.linalg import norm

import seaborn as sns
sns.set_context("notebook", font_scale=2.5, rc={"lines.linewidth": 2.5})

file_name = sys.argv[1]
skip_outliers = False
threshold = 1.0

if len(sys.argv) > 2:
    threshold = float(sys.argv[2])
    if len(sys.argv) > 3:
        skip_outliers = sys.argv[3] == 'True'

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
best_case_seed = -1
best_case_value = 1e100

for line in data:
    mean_error = line[1]
    if mean_error > worst_case_value:
        if skip_outliers:
            if mean_error < threshold:
                worst_case_seed = line[0]
                worst_case_value = mean_error
        else:
            worst_case_seed = line[0]
            worst_case_value = mean_error

    if mean_error < best_case_value:
        if skip_outliers:
            if mean_error < threshold:
                best_case_seed = line[0]
                best_case_value = mean_error
        else:
            best_case_seed = line[0]
            best_case_value = mean_error
        
    if mean_error >= threshold:
        outliers += 1
        print(line)
        if skip_outliers:
            continue

    filtered_data += [line]

data = array(filtered_data)

mean_errors = data[:, 1]
ssme_mean = mean(mean_errors,axis=0)
ssme_stdev = std(mean_errors, axis=0)
ssme_min = min(mean_errors)
ssme_max = max(mean_errors)

delta_errors = data[:,3] - data[:,2]
ssde_mean = mean(delta_errors,axis=0)
ssde_stdev = std(delta_errors, axis=0)
ssde_min = min(delta_errors)
ssde_max = max(delta_errors)

predicted_fuel = data[:, 4]
predicted_fuel_mean = mean(predicted_fuel, axis=0)
predicted_fuel_stdev = std(predicted_fuel, axis=0)
predicted_fuel_min = min(predicted_fuel)
predicted_fuel_max = max(predicted_fuel)

used_fuel = data[:, 5]
used_fuel_mean = mean(used_fuel, axis=0)
used_fuel_stdev = std(used_fuel, axis=0)
used_fuel_min = min(used_fuel)
used_fuel_max = max(used_fuel)

print("Outliers: {0}".format(outliers))

text1 = "SSME: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssme_mean, ssme_stdev, ssme_min, ssme_max)
text2 = "SSDE: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssde_mean, ssde_stdev, ssde_min, ssde_max)

print(text1)
print(text2)

text1 = "Predicted Fuel: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (predicted_fuel_mean, predicted_fuel_stdev, predicted_fuel_min, predicted_fuel_max)
text2 = "Used Fuel: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (used_fuel_mean, used_fuel_stdev, used_fuel_min, used_fuel_max)
print(text1)
print(text2)

print("Worst case seed: {0}".format(worst_case_seed))
print("Best case seed: {0}".format(best_case_seed))

plt.boxplot(data[:, 1])
plt.ylabel('mean offset [m]')
plt.show()
