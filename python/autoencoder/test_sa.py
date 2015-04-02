from numpy import random, array
from numpy.linalg import norm
from stacked_autoencoder import StackedAutoencoder
from data_loader import load_autoencoder_weights, load_data_set
import os

testing_path = "/home/willist/Documents/dnn/data/no_policy_rv/testing/"
autoencoder_weights_path = "/home/willist/Documents/dnn/autoencoder/conf_100_master_noise/"

num_test_samples = 10000
num_test_samples_per_file = 10000
history_length = 10
include_actions_in_history = False

batch_size = 1

supervised_sigmoid_activation = True

hidden_layer_sizes = [100]

tied_weights =              [False]
sigmoid_compressions =      [True]
sigmoid_reconstructions =   [True]

autoencoder_weights, supervised_layer_weights = load_autoencoder_weights()

numpy_rng = random.RandomState(89677)

testing_files = os.listdir(testing_path)
testing_files = [testing_path + name for name in testing_files if "set" in name]
test_set, test_labels = load_data_set(testing_files, num_test_samples_per_file, history_length, include_actions_in_history)
test_set = array(test_set)
test_labels = array(test_labels)

sample_dimension = test_set.shape[1]
label_dimension = test_labels.shape[1]

stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, autoencoder_weights_path)

error = 0.0
predicted_labels = stacked_autoencoder.predict(test_set)
for y, y_tilde in zip(test_labels, predicted_labels):
    error += norm(y-y_tilde)

print(error/test_labels.shape[0])


