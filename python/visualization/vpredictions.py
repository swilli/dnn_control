'''
usage:
./python2.7 vpredictions.py <data_file> [time]

default time is 60

examples:
./python2.7 vpredictions.py predictions.txt
./python2.7 vpredictions.py predictions.txt 3600
'''

import sys
from numpy import array
import matplotlib.pyplot as plt

end_time = 60
file_name = sys.argv[1]

if len(sys.argv) > 2:
    end_time = float(sys.argv[2])

print("preparing data... ")

result_file = open(file_name, 'r')
lines = result_file.readlines()
result_file.close()

times = [float(val) for val in range(len(lines)) if float(val) <= end_time]
num_samples = len(times)

lines = [line.split('|') for line in lines]
predicted = [val for _, val in lines]
correct = [val for val, _  in lines]
predicted = [line.split(',') for line in predicted[:num_samples]]
predicted = array([[float(value) for value in data_line] for data_line in predicted])
correct = [line.split(',') for line in correct[:num_samples]]
correct = array([[float(label) for label in label_line] for label_line in correct])

times = array(times)

fig = plt.figure(1)
fig.subplots_adjust(hspace=1.0)

if predicted.shape[1] == 3:
	plt.subplot(131)
	plt.plot(times, predicted[:, 0], 'r--', correct[:, 0], 'b-')
	plt.title('Velocity X vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(132)
	plt.plot(times, predicted[:, 1], 'r--', correct[:, 1], 'b-')
	plt.title('Velocity Y vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(133)
	plt.plot(times, predicted[:, 2], 'r--', correct[:, 2], 'b-')
	plt.title('Velocity Z vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity [m/s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

else:
	plt.subplot(231)
	plt.plot(times, predicted[:, 0], 'r--', correct[:,0], 'b-')
	plt.title('Position X vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('position (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(232)
	plt.plot(times, predicted[:, 1], 'r--', correct[:,1], 'b-')
	plt.title('Position Y vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('position (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(233)
	plt.plot(times, predicted[:, 2], 'r--', correct[:, 2], 'b-')
	plt.title('Position Z vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('position (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(234)
	plt.plot(times, predicted[:, 3], 'r--', correct[:,3], 'b-')
	plt.title('Velocity X vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(235)
	plt.plot(times, predicted[:, 4], 'r--', correct[:, 4], 'b-')
	plt.title('Velocity Y vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

	plt.subplot(236)
	plt.plot(times, predicted[:, 5], 'r--', correct[:, 5], 'b-')
	plt.title('Velocity Z vs Time')
	plt.xlabel('time [s]')
	plt.ylabel('velocity [m/s]')
	plt.ylabel('velocity (normalized)')
	plt.legend(['predicted', 'correct'])

plt.show()
