from data_loader import load_sensor_files
from sklearn.decomposition import PCA
from numpy import array
from random import sample
from numpy.linalg import norm

num_samples = 100000
data_path = "/home/willist/Documents/dnn/data/"
training_set = load_sensor_files(data_path, num_samples=num_samples, shared=False)

pca = PCA(n_components=5)
pca.fit(training_set)

num_test_samples = len(training_set)/10
test_samples = sample(training_set, num_test_samples)
mean_error = 0.0
for sample in test_samples:
    s_compr = pca.transform(sample)
    s_decompr = pca.inverse_transform(s_compr)
    #print sample
    #print s_compr
    #print s_decompr
    mean_error += norm(sample - s_decompr)

print mean_error / num_test_samples