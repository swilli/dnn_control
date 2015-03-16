def shared_dataset(data, borrow=True):
    from theano import shared
    from theano import config as theano_config
    from numpy import asarray

    shared_data = shared(asarray(data, dtype=theano_config.floatX), borrow=borrow)
    return shared_data


def historify(state_sets, action_sets, length):
    from numpy import array, concatenate

    def window_stack(a, stepsize=1, width=3):
        from numpy import hstack
        return hstack(a[i:1+i-width or None:stepsize] for i in range(0, width))

    result = []
    for states, actions in zip(state_sets, action_sets):
        states = array(states)
        actions = array(actions)
        state_action_pairs = concatenate((states, actions), axis=1)
        historyfied_set = window_stack(state_action_pairs, 1, length)
        result += [historyfied_set.tolist()]

    return result


def normalize(data_sets):
    from numpy import array, max, min

    result = []
    max_values = []
    min_values = []
    for data_set in data_sets:
        data_set = array(data_set)
        max_values = max_values + [max(data_set, axis=0).tolist()]
        min_values = min_values + [min(data_set, axis=0).tolist()]

    max_values = array(max_values)
    min_values = array(min_values)
    max_values = max(max_values, axis=0)
    min_values = min(min_values, axis=0)

    max_values *= 1.1
    min_values *= 1.1

    for data_set in data_sets:
        data_set = array(data_set)
        for i in range(data_set.shape[1]):
            min_value = min_values[i]
            max_value = max_values[i]
            range_value = max_value - min_value
            data_set[:, i] = (data_set[:, i] - min_value) / range_value

        result += [data_set.tolist()]

    return result


def load_sensor_file(file_path, num_lines=1000):
    sensor_data_file = open(file_path, 'r')
    lines = sensor_data_file.readlines()
    sensor_data_file.close()
    lines = [line for line in lines if not line.startswith("#")]
    lines = lines[:num_lines]
    lines = [line.split('|') for line in lines]
    states = [state for state, _ in lines]
    actions = [action for _, action in lines]
    states = [line.split(',') for line in states]
    states = [[float(value) for value in data_line] for data_line in states]
    actions = [line.split(',') for line in actions]
    actions = [[float(label) for label in label_line] for label_line in actions]

    return states, actions


def load_data_set(file_paths, num_training_samples, history_length):
    total_states = []
    total_actions = []
    for file_path in file_paths:
        print '... loading data file ' + file_path
        states, actions = load_sensor_file(file_path, num_training_samples / len(file_paths))
        total_states = total_states + [states]
        total_actions = total_actions + [actions]

    normalized_states = normalize(total_states)
    normalized_actions = total_actions

    state_action_pairs_with_history_set = historify(normalized_states, normalized_actions, history_length)

    total_data_set = []
    for data_set in state_action_pairs_with_history_set:
        total_data_set.extend(data_set)

    return total_data_set


def load_sensor_files(data_path,
                      num_training_samples=100000,
                      num_test_samples=10000,
                      shared=True,
                      history_length=3):

    from os import listdir
    from random import sample
    from random import shuffle

    file_names = listdir(data_path)
    file_names = [name for name in file_names if "trajectory" not in name]
    training_file_names = sample(file_names, len(file_names) / 2)
    shuffle(training_file_names)
    test_file_names = [file_name for file_name in file_names if file_name not in training_file_names]
    shuffle(test_file_names)
    training_file_paths = [data_path + name for name in training_file_names]
    test_file_paths = [data_path + name for name in test_file_names]

    print("Loading training data")
    training_data = load_data_set(training_file_paths, num_training_samples, history_length)
    print("{0} training samples loaded.".format(len(training_data)))
    print("Loading test data")
    test_data = load_data_set(test_file_paths, num_test_samples, history_length)
    print("{0} test samples loaded.".format(len(test_data)))

    if shared:
        training_data = shared_dataset(training_data)

    return training_data, test_data
