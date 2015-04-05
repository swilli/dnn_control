from numpy import random, array
from random import sample
from numpy.linalg import norm
from stacked_autoencoder import StackedAutoencoder
from data_loader import load_data_set
import os

user_path = os.path.expanduser("~")

testing_path = user_path + "/Documents/dnn/data/random_policy/testing/"
autoencoder_weights_path = user_path + "/Documents/dnn/autoencoder/conf_7_random_policy/"
prediction_file = user_path + "/Documents/dnn/results/prediction.txt"

SINGLE_PREDICTION = True
num_test_samples = 250
num_test_samples_per_file = 250
history_length = 10
include_actions_in_history = True


batch_size = 1

numpy_rng = random.RandomState(89677)

if not SINGLE_PREDICTION:
    num_test_samples = num_test_samples_per_file

testing_files = os.listdir(testing_path)
testing_files = [testing_path + name for name in testing_files if "set" in name]
testing_files = sample(testing_files, num_test_samples / num_test_samples_per_file)
test_set, test_labels = load_data_set(testing_files, num_test_samples_per_file, history_length,
                                      feature_indexes=feature_indexes, label_indexes=label_indexes)

if not SINGLE_PREDICTION:
    test_set_no_history, _ = load_data_set(testing_files, num_test_samples_per_file, 1, include_actions_in_history)
    test_set_no_history = array(test_set_no_history)

test_set = array(test_set)
test_labels = array(test_labels)

sample_dimension = test_set.shape[1]
label_dimension = test_labels.shape[1]

stacked_autoencoder = StackedAutoencoder.from_config_path(numpy_rng, autoencoder_weights_path)

if SINGLE_PREDICTION:
    predicted_states = stacked_autoencoder.predict(test_set).tolist()
    correct_states = test_labels.tolist()
else:
    dim_state = 6
    dim_action = 3
    simulation_time = test_set_no_history.shape[0] - history_length
    state_action_history = test_set_no_history[:history_length, :].ravel().tolist()

    predicted_states = test_set_no_history[:history_length, :dim_state].tolist()
    correct_states = test_set_no_history[:history_length, :dim_state].tolist()

    for i in range(simulation_time):
        print("{0} seconds predicted".format(i+1))
        current_sample = state_action_history[-sample_dimension:]
        next_state = stacked_autoencoder.predict(current_sample)[0].tolist()
        predicted_states.append(next_state)
        state_action_history.extend(next_state)
        state_action_history.extend(test_set_no_history[history_length + i, -dim_action:].tolist())
        correct_next_state = test_labels[i, :].tolist()
        correct_states.append(correct_next_state)


with open(prediction_file, 'w+') as prediction_file:
    data = ""
    for y, y_tilde in zip(correct_states, predicted_states):
        data += ", ".join(str(val) for val in y)
        data += " | "
        data += ", ".join(str(val) for val in y_tilde)
        data += "\n"

    prediction_file.write(data)



