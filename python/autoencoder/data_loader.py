data_set_min_values = []
data_set_max_values = []


def shared_dataset(data, borrow=True, name=None):
    from theano import shared
    from theano import config as theano_config
    from numpy import asarray

    if name is None:
        shared_data = shared(asarray(data, dtype=theano_config.floatX), borrow=borrow)
    else:
        shared_data = shared(asarray(data, dtype=theano_config.floatX), borrow=borrow, name=name)
    return shared_data


def sliding_window(data, width=3):
    result = []
    window_index = range(width - 1, -1, -1)
    data_index = range(len(data) - width + 1)
    for i in data_index:
        sample = []
        for j in window_index:
            sample += data[i+j][:]
        result += [sample]
    return result


def historify(state_sets, action_sets, length):
    from numpy import array, concatenate

    result = []
    for states, actions in zip(state_sets, action_sets):
        states = array(states)
        actions = array(actions)
        state_action_pairs = concatenate((states, actions), axis=1).tolist()
        historyfied_set = sliding_window(state_action_pairs, length)
        result += [historyfied_set]

    return result


def load_sensor_file(file_path, num_lines=1000):
    lines = []
    with open(file_path, 'r') as sensor_data_file:
        for line in sensor_data_file:
            if line.startswith("#"):
                continue
            lines += [line]
            if len(lines) == num_lines:
                break

    lines = [line.split('|') for line in lines]
    states = [state for state, _ in lines]
    actions = [action for _, action in lines]
    states = [line.split(',') for line in states]
    states = [[float(value) for value in data_line] for data_line in states]
    actions = [line.split(',') for line in actions]
    actions = [[float(label) for label in label_line] for label_line in actions]

    return states, actions


def min_max_values(data_path):
    from os import listdir
    from numpy import max, min, maximum, minimum

    global data_set_min_values
    global data_set_max_values

    file_names = listdir(data_path)
    file_names = [name for name in file_names if "trajectory" not in name]
    file_names = sorted(file_names)
    file_paths = [data_path + name for name in file_names]
    max_values = []
    min_values = []
    for file_path in file_paths:
        print '... loading all samples from data file ' + file_path
        states, _ = load_sensor_file(file_path, 1000000)
        cur_max = max(states, axis=0)
        cur_min = min(states, axis=0)
        if len(max_values) == 0:
            max_values = cur_max
            min_values = cur_min
        else:
            max_values = maximum(max_values, cur_max)
            min_values = minimum(min_values, cur_min)

    data_set_min_values = min_values * 1.1
    data_set_max_values = max_values * 1.1


def normalize(data_sets):
    from numpy import array

    global data_set_min_values
    global data_set_max_values

    result = []
    for data_set in data_sets:
        data_set = array(data_set)
        for i in range(data_set.shape[1]):
            min_value = data_set_min_values[i]
            max_value = data_set_max_values[i]
            range_value = max_value - min_value
            data_set[:, i] = (data_set[:, i] - min_value) / range_value

        result += [data_set.tolist()]

    return result


def load_data_set(file_paths, num_samples_per_file, history_length):
    total_states = []
    total_actions = []
    for file_path in file_paths:
        print '... loading ' + str(num_samples_per_file) + ' samples from data file ' + file_path
        states, actions = load_sensor_file(file_path, num_samples_per_file)
        total_states = total_states + [states]
        total_actions = total_actions + [actions]

    normalized_states = total_states  # normalize(total_states)
    normalized_actions = total_actions

    print '... historifying data'
    state_action_pairs_with_history_set = historify(normalized_states, normalized_actions, history_length)

    total_data_set = []
    for data_set in state_action_pairs_with_history_set:
        total_data_set.extend(data_set)

    return total_data_set


def load_sensor_files(data_path,
                      num_training_samples=1000000,
                      num_training_samples_per_file=600,
                      num_test_samples=10000,
                      num_test_samples_per_file=10,
                      shared=True,
                      history_length=3):

    from os import listdir
    from random import sample
    from random import shuffle
    from numpy import array

    global data_set_min_values
    global data_set_max_values

    #min_max_values(data_path)

    data_set_min_values = array([-3994.3042216, -8327.502227, -11097.702649, -8428.7475976, -10442.7092077, -10654.8025452])
    data_set_max_values = array([3916.9638035, 6798.05434, 10873.8216037, 11264.354926, 8495.9342369, 13091.503392])

    print("minimum values in training data set: {0}".format(data_set_min_values))
    print("maximum values in training data set: {0}".format(data_set_max_values))

    file_names = listdir(data_path)
    file_names = [name for name in file_names if "trajectory" not in name]
    num_training_files = min([len(file_names), max([1, num_training_samples / num_training_samples_per_file])])
    training_file_names = sample(file_names, num_training_files)
    shuffle(training_file_names)

    test_file_names = [file_name for file_name in file_names if file_name not in training_file_names]
    num_test_files = min([len(test_file_names), max([1, num_test_samples / num_test_samples_per_file])])
    test_file_names = sample(test_file_names, num_test_files)
    shuffle(test_file_names)

    training_file_paths = [data_path + name for name in training_file_names]
    test_file_paths = [data_path + name for name in test_file_names]

    print("Loading training data")
    training_data = load_data_set(training_file_paths, num_training_samples_per_file, history_length)
    print("Loading test data")
    test_data = load_data_set(test_file_paths, num_test_samples_per_file, history_length)

    print("{0} training samples loaded.".format(len(training_data)))
    print("{0} test samples loaded.".format(len(test_data)))

    if shared:
        training_data = shared_dataset(training_data)

    return training_data, test_data


def load_autoencoder_weights(path_to_autoencoder_weights):
    from os import listdir
    from numpy import array

    num_files_per_layer = 4

    file_names = listdir(path_to_autoencoder_weights)
    file_names = sorted(file_names)
    file_paths = [path_to_autoencoder_weights + name for name in file_names]

    all_weights = []
    for i in xrange(len(file_paths)/num_files_per_layer):
        weights_file = open(file_paths[num_files_per_layer*i], 'r')
        weights_prime_file = open(file_paths[num_files_per_layer*i + 1], 'r')
        bias_file = open(file_paths[num_files_per_layer*i + 2], 'r')
        bias_prime_file = open(file_paths[num_files_per_layer*i + 3], 'r')

        bias_data = bias_file.readline()
        bias_file.close()
        bias = array([float(value) for value in bias_data.split(",")])

        bias_prime_data = bias_prime_file.readline()
        bias_prime_file.close()
        bias_prime = array([float(value) for value in bias_prime_data.split(",")])

        weights_data = weights_file.read()
        weights_file.close()
        weights = array([[float(value) for value in line.split(",")] for line in weights_data.split('\n')]).T

        weights_prime_data = weights_prime_file.read()
        weights_prime_file.close()
        weights_prime = array([[float(value) for value in line.split(",")] for line in weights_prime_data.split('\n')]).T

        weights = shared_dataset(weights, name="W")
        bias = shared_dataset(bias, name="b")
        weights_prime = shared_dataset(weights_prime)
        bias_prime = shared_dataset(bias_prime)

        all_weights.append((weights, bias, weights_prime, bias_prime))

    return all_weights