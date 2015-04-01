from numpy import random, array
from numpy.linalg import norm
from stacked_autoencoder import StackedAutoencoder
from data_loader import load_autoencoder_weights, load_data_set
import os

testing_path = "/home/willist/Documents/dnn/data/no_policy_rv_1Hz/testing/"
autoencoder_weights_path = "/home/willist/Documents/dnn/autoencoder/conf_100_master_linear/"
prediction_file = "/home/willist/Documents/dnn/results/prediction.txt"

num_test_samples = 10000
num_test_samples_per_file = 10000
history_length = 10
include_actions_in_history = False

batch_size = 1

supervised_sigmoid_activation = False

hidden_layer_sizes = [100]

tied_weights =              [False]
sigmoid_compressions =      [False]
sigmoid_reconstructions =   [False]

autoencoder_weights, supervised_layer_weights = load_autoencoder_weights(autoencoder_weights_path)

numpy_rng = random.RandomState(89677)

testing_files = os.listdir(testing_path)
testing_files = [testing_path + name for name in testing_files if "set" in name]
test_set, test_labels = load_data_set(testing_files, num_test_samples_per_file, history_length, include_actions_in_history)
test_set = array(test_set)
test_labels = array(test_labels)

sample_dimension = test_set.shape[1]
label_dimension = test_labels.shape[1]

stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, n_ins=sample_dimension, n_outs=label_dimension,
                                         hidden_layers_sizes=hidden_layer_sizes,
                                         tied_weights=tied_weights,
                                         sigmoid_compressions=sigmoid_compressions,
                                         sigmoid_reconstructions=sigmoid_reconstructions,
                                         autoencoder_weights=autoencoder_weights,
                                         supervised_sigmoid_activation=supervised_sigmoid_activation,
                                         supervised_layer_weights=supervised_layer_weights)

simulation_time = 300
predicted_sequence = test_set[0, :].tolist()
correct_sequence = test_set[0, :].tolist()
for i in range(simulation_time):
    print("{0} seconds predicted".format(i+1))
    current_sample = predicted_sequence[-sample_dimension:]
    next_state = stacked_autoencoder.predict(current_sample)[0]
    predicted_sequence.extend(next_state)
    correct_sequence.extend(test_labels[i, :].tolist())

state_dimension = 6
predicted_states = array(predicted_sequence).reshape(len(predicted_sequence) / state_dimension, state_dimension).tolist()
correct_states = array(correct_sequence).reshape(len(correct_sequence) / state_dimension, state_dimension).tolist()

with open(prediction_file, 'w+') as prediction_file:
    data = ""
    for y, y_tilde in zip(correct_states, predicted_states):
        data += ", ".join(str(val) for val in y)
        data += " | "
        data += ", ".join(str(val) for val in y_tilde)
        data += "\n"

    prediction_file.write(data)



