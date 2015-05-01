'''
usage:
./python2.7 vconvexity.py <data_file>

examples:
./python2.7 vconvexity.py convexity.txt
'''

def visualize_convexity(file_path, save=False):
	import matplotlib.pyplot as plt
	from numpy import array, nan, diff
	from numpy.matlib import repmat
	from numpy.linalg import norm

	import seaborn as sns
	sns.set_context("notebook", font_scale=2.5, rc={"lines.linewidth": 2.5})
	
	print("preparing data... ")
	result_file = open(file_path, 'r')
	seed, dimension = [int(value) for value in result_file.readline().split(',')]
	lines = result_file.readlines()
	result_file.close()

	num_samples = len(lines)
	data = [line.split(',') for line in lines]
	data = [[float(value) for value in line] for line in data]
	data = array(data)

	def discontinuous(data):
	    data[diff(data) >= 0.5] = nan
	    return data

	plt.plot(data[:,0], discontinuous(data[:,1]))
	#plt.title("Convexity of problem (seed: {0}, dimension: {1})".format(seed, dimension))
	plt.xlabel('parameter value')
	plt.ylabel('negative utility')
	if save:
		plt.savefig(file_path.replace(".txt", ".svg"))
		plt.close()
	else:
		plt.show()

if __name__ == '__main__':
	import sys
	file_path = sys.argv[1]
	visualize_convexity(file_path)

