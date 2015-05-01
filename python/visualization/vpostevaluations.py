import sys
import matplotlib
import matplotlib.pyplot as plt
from numpy import array, mean, minimum, maximum,std, sqrt
from numpy.matlib import repmat
from numpy.linalg import norm

import seaborn as sns
sns.set_context("notebook", font_scale=5.0, rc={"lines.linewidth": 2.5})
sns.set_style("whitegrid")


file_names = sys.argv[1:4]
threshold = 100.0

print("preparing data... ")

total_data = []
for file_name in file_names:
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
            print(line)
            continue

        filtered_data += [line]

    total_data += [array(filtered_data)]

sns.boxplot([total_data[0][:,1], total_data[1][:,1], total_data[2][:,1]], widths=0.5)
plt.yscale('log')
plt.ylabel('Mean Offset [m]')
plt.xticks([1, 2, 3], ['DPS1', 'DPS2', 'LSPI1'])
plt.show()