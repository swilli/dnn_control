from numpy import random, array
from numpy.linalg import norm
from stacked_autoencoder import StackedAutoencoder
from data_loader import load_data_set
import os

user_path = os.path.expanduser("~")

testing_path = user_path + "/Documents/dnn/data/fixed_ast_fixed_sc/testing/"
autoencoder_weights_path = user_path + "/Documents/dnn/autoencoder/conf_7_7_sigmoid/"
prediction_file = user_path + "/Documents/dnn/results/prediction.txt"

num_test_samples = 10000
num_test_samples_per_file = 10000
history_length = 10
include_actions_in_history = False

batch_size = 1

numpy_rng = random.RandomState(89677)

testing_files = os.listdir(testing_path)
testing_files = [testing_path + name for name in testing_files if "set" in name]
test_set, test_labels = load_data_set(testing_files, num_test_samples_per_file, history_length, include_actions_in_history)
test_set = array(test_set)
test_labels = array(test_labels)

sample_dimension = test_set.shape[1]
label_dimension = test_labels.shape[1]

stacked_autoencoder = StackedAutoencoder.from_config_path(numpy_rng, autoencoder_weights_path)

predicted_states = stacked_autoencoder.predict(test_set).tolist()
correct_states = test_labels.tolist()

#simulation_time = test_set.shape[0]
#predicted_sequence = test_set[0, :].tolist()
#correct_sequence = test_set[0, :].tolist()
#for i in range(simulation_time):
#    print("{0} seconds predicted".format(i+1))
#    current_sample = test_set[i, :]
#    next_state = stacked_autoencoder.predict(current_sample)[0]
#    predicted_sequence.extend(next_state)
#    correct_next_state = test_labels[i, :].tolist()
#    correct_sequence.extend(correct_next_state)


#state_dimension = 6
#predicted_states = array(predicted_sequence).reshape(len(predicted_sequence) / state_dimension, state_dimension).tolist()
#correct_states = array(correct_sequence).reshape(len(correct_sequence) / state_dimension, state_dimension).tolist()

with open(prediction_file, 'w+') as prediction_file:
    data = ""
    for y, y_tilde in zip(correct_states, predicted_states):
        data += ", ".join(str(val) for val in y)
        data += " | "
        data += ", ".join(str(val) for val in y_tilde)
        data += "\n"

    prediction_file.write(data)



