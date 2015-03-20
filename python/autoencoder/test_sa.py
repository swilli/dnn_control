from numpy import random
from numpy.linalg import norm
from random import sample
from stacked_autoencoder import StackedAutoencoder
from data_loader import load_autoencoder_weights, load_sensor_files

data_path = "/home/willist/Documents/dnn/data/"
autoencoder_weights_path = "/home/willist/Documents/dnn/autoencoder/"

sample_dimension = 9
history_length = 1
#hidden_layer_sizes = [45, 24, 12, 6]
#tied_weights = [True, True, True, True]
#sigmoid_compressions = [True, True, True, True]
#sigmoid_reconstructions = [True, True, True, True]
hidden_layer_sizes = [9]
tied_weights = [False]
linear_compressions = [True]
linear_reconstructions = [True]

autoencoder_weights_path += "conf_" + "_".join([str(value) for value in hidden_layer_sizes]) + "/"
autoencoder_weights = load_autoencoder_weights(autoencoder_weights_path)

numpy_rng = random.RandomState(89677)

stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, n_ins=sample_dimension,
                                         hidden_layers_sizes=hidden_layer_sizes,
                                         autoencoder_weights=autoencoder_weights,
                                         tied_weights=tied_weights,
                                         sigmoid_compressions=linear_compressions,
                                         sigmoid_reconstructions=linear_reconstructions)

num_training_samples = 0
num_test_samples = 10

training_set, test_set = load_sensor_files(data_path, num_training_samples=num_training_samples,
                                           num_test_samples=num_test_samples, history_length=history_length)

test_samples = test_set
mean_error = 0.0
num_tests = 0
for sample in test_samples:
    #print("===")
    s_compr = stacked_autoencoder.compress(sample)
    s_decompr = stacked_autoencoder.decompress(s_compr)
    #print("{0}".format(", ".join([str(value) for value in sample])))
    #print("{0}".format(", ".join([str(value) for value in s_compr])))
    #print("z: {0}".format(s_decompr))
    mean_error += norm(sample - s_decompr)
    num_tests += 1
    print("avg error: {0}".format(mean_error / num_tests))
    #print("")