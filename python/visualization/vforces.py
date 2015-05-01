'''
usage:
./python2.7 vforces.py <data_file> 


examples:
./python2.7 vforces.py data_set.txt
'''

import sys
import matplotlib
import matplotlib.pyplot as plt
from numpy import array, mean, minimum, maximum,std, sqrt, max, min
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
data = [[float(value) for value in line] for line in data]
data = array(data)

mean_x = mean(data[:, 0],axis=0)
mean_y = mean(data[:, 1],axis=0)
mean_z = mean(data[:, 2],axis=0)
std_x = std(data[:, 0], axis=0)
std_y = std(data[:, 1], axis=0)
std_z = std(data[:, 2], axis=0)
min_x = min(data[:, 0],axis=0)
min_y = min(data[:, 1],axis=0)
min_z = min(data[:, 2],axis=0)
max_x = max(data[:, 0],axis=0)
max_y = max(data[:, 1],axis=0)
max_z = max(data[:, 2],axis=0)

text = "\nmean x: %.5f, %.5f, %.5f, %.5f\nmean y: %.5f, %.5f, %.5f, %.5f\nmean z: %.5f, %.5f, %.5f, %.5f\n" % (mean_x, std_x, min_x, max_x, mean_y, std_y, min_y, max_y, mean_z, std_z, min_z, max_z)
print(text)

mean_max_x = mean(data[:, 4],axis=0)
mean_max_y = mean(data[:, 6],axis=0)
mean_max_z = mean(data[:, 8],axis=0)
std_max_x = std(data[:, 4], axis=0)
std_max_y = std(data[:, 6], axis=0)
std_max_z = std(data[:, 8], axis=0)
min_max_x = min(data[:, 4],axis=0)
min_max_y = min(data[:, 6],axis=0)
min_max_z = min(data[:, 8],axis=0)
max_max_x = max(data[:, 4],axis=0)
max_max_y = max(data[:, 6],axis=0)
max_max_z = max(data[:, 8],axis=0)

text = "max x: %.5f, %.5f, %.5f, %.5f\nmax y: %.5f, %.5f, %.5f, %.5f\nmax z: %.5f, %.5f, %.5f, %.5f" % (mean_max_x, std_max_x, min_max_x, max_max_x, mean_max_y, std_max_y, min_max_y, max_max_y, mean_max_z, std_max_z, min_max_z, max_max_z)
print(text)

'''
mean_min_x = mean(data[:, 3],axis=0)
mean_min_y = mean(data[:, 5],axis=0)
mean_min_z = mean(data[:, 7],axis=0)
std_min_x = std(data[:, 3], axis=0)
std_min_y = std(data[:, 5], axis=0)
std_min_z = std(data[:, 7], axis=0)
min_min_x = min(data[:, 3],axis=0)
min_min_y = min(data[:, 5],axis=0)
min_min_z = min(data[:, 7],axis=0)
max_min_x = max(data[:, 3],axis=0)
max_min_y = max(data[:, 5],axis=0)
max_min_z = max(data[:, 7],axis=0)



subtitle1 = "SSME: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssme_mean, ssme_stdev, ssme_min, ssme_max)
subtitle2 = "SSDE: mean=%.5f, stdev=%.5f, min=%.5f, max=%.5f" % (ssde_mean, ssde_stdev, ssde_min, ssde_max)

print(subtitle1)
print(subtitle2)
print("Worst case seed: {0}".format(worst_case_seed))

plt.boxplot(data[:, 1])
plt.ylabel('mean offset [m]')
plt.show()
'''
