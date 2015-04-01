def shared_dataset(data, borrow=True, name=None):
    import theano
    from numpy import asarray

    if name is None:
        shared_data = theano.shared(asarray(data, dtype=theano.config.floatX), borrow=borrow)
    else:
        shared_data = theano.shared(asarray(data, dtype=theano.config.floatX), borrow=borrow, name=name)

    return shared_data


def sliding_window(data, label_length, sequence_length):
    design_matrix = []
    labels = []
    window_index = range(sequence_length)
    data_index = range(len(data) - sequence_length)
    for i in data_index:
        sample = []
        for j in window_index:
            sample += data[i+j][:]
        label = data[i+sequence_length][:label_length]
        design_matrix += [sample]
        labels += [label]

    return design_matrix, labels


def historify_and_label(state_sets, action_sets, sequence_length, with_actions):
    from numpy import array, concatenate

    result = []
    labels = []
    for states, actions in zip(state_sets, action_sets):
        states = array(states)
        label_length = states.shape[1]
        actions = array(actions)
        if with_actions:
            samples = concatenate((states, actions), axis=1).tolist()
        else:
            samples = states.tolist()
        historyfied_set, label_set = sliding_window(samples, label_length, sequence_length)
        result += [historyfied_set]
        labels += [label_set]

    return result, labels


def load_sensor_file(file_path, num_lines=None):
    lines = []
    with open(file_path, 'r') as sensor_data_file:
        if num_lines is None:
            data = sensor_data_file.read()
            lines = data.split('\n')
            lines = [line for line in lines if not line.startswith("#") and not line == ""]
        else:
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


def load_data_set(file_paths, num_samples_per_file, history_length, with_actions):
    total_states = []
    total_actions = []
    for file_path in file_paths:
        print '... loading ' + str(num_samples_per_file) + ' samples from data file ' + file_path
        states, actions = load_sensor_file(file_path, num_samples_per_file)
        total_states = total_states + [states]
        total_actions = total_actions + [actions]

    print '... historifying and labelling data'
    history_set, labels_set = historify_and_label(total_states, total_actions, history_length, with_actions)

    total_data_set = []
    total_label_set = []
    for data_set, label_set in zip(history_set, labels_set):
        total_data_set.extend(data_set)
        total_label_set.extend(label_set)

    return total_data_set, total_label_set


def load_sensor_files(training_data_path, testing_data_path,
                      num_training_samples=1000000,
                      num_training_samples_per_file=100,
                      num_test_samples=10000,
                      num_test_samples_per_file=10,
                      shared=True,
                      history_length=3,
                      with_actions=True):

    from os import listdir
    from os.path import isdir
    from random import sample
    from random import shuffle
    from numpy import array

    file_names = listdir(training_data_path)
    training_file_paths = [training_data_path + name for name in file_names]
    training_file_paths = [path for path in training_file_paths if not (isdir(path) or "trajectory" in path)]
    num_training_files = min([len(training_file_paths), max([1, num_training_samples / num_training_samples_per_file])])
    training_file_paths = sample(training_file_paths, num_training_files)
    shuffle(training_file_paths)

    file_names = listdir(testing_data_path)
    test_file_paths = [testing_data_path + name for name in file_names]
    test_file_paths = [path for path in test_file_paths if not (isdir(path) or "trajectory" in path)]
    num_test_files = min([len(test_file_paths), max([1, num_test_samples / num_test_samples_per_file])])
    test_file_paths = sample(test_file_paths, num_test_files)
    shuffle(test_file_paths)


    print("Loading training data")
    training_data, training_labels = load_data_set(training_file_paths, num_training_samples_per_file, history_length, with_actions)
    print("Loading test data")
    test_data, test_labels = load_data_set(test_file_paths, num_test_samples_per_file, history_length, with_actions)

    print("{0} training samples loaded.".format(len(training_data)))
    print("{0} test samples loaded.".format(len(test_data)))

    if shared:
        training_data = shared_dataset(training_data)
        training_labels = shared_dataset(training_labels)
        test_data = shared_dataset(test_data)
        test_labels = shared_dataset(test_labels)

    return training_data, training_labels, test_data, test_labels


def load_autoencoder_weights(path_to_deep_network_weights):
    from os import listdir
    from numpy import array

    num_files_per_layer = 4

    file_names = listdir(path_to_deep_network_weights)
    file_names = sorted(file_names)
    autoencoder_file_paths = [path_to_deep_network_weights + name for name in file_names if name.startswith("al")]
    supervised_file_paths = [path_to_deep_network_weights + name for name in file_names if name.startswith("sl")]

    autoencoder_weights = []
    for i in xrange(len(autoencoder_file_paths)/num_files_per_layer):
        weights_file = open(autoencoder_file_paths[num_files_per_layer*i], 'r')
        weights_prime_file = open(autoencoder_file_paths[num_files_per_layer*i + 1], 'r')
        bias_file = open(autoencoder_file_paths[num_files_per_layer*i + 2], 'r')
        bias_prime_file = open(autoencoder_file_paths[num_files_per_layer*i + 3], 'r')

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

        weights = shared_dataset(weights, name='W')
        bias = shared_dataset(bias, name='b')
        weights_prime = shared_dataset(weights_prime, name='Whid')
        bias_prime = shared_dataset(bias_prime, name='bvis')

        autoencoder_weights.append((weights, bias, weights_prime, bias_prime))

    supervised_weights_file = open(supervised_file_paths[0], 'r')
    supervised_weights_data = supervised_weights_file.read()
    supervised_weights_file.close()
    supervised_weights = shared_dataset(array([[float(value) for value in line.split(",")] for line in supervised_weights_data.split('\n')]).T, name='W')

    supervised_bias_file = open(supervised_file_paths[1], 'r')
    supervised_bias_data = supervised_bias_file.readline()
    supervised_bias = shared_dataset(array([float(value) for value in supervised_bias_data.split(",")]), name='b')

    return autoencoder_weights, (supervised_weights, supervised_bias)