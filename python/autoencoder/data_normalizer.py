from data_loader import load_sensor_file


def prepare_folder_data(data_folder):
    from os import listdir
    from numpy import array
    from sklearn.preprocessing import MinMaxScaler

    file_names = listdir(data_folder)
    file_names = [name for name in file_names if "trajectory" not in name]
    file_names = sorted(file_names)
    file_paths = [data_folder + name for name in file_names]

    set_sizes = []
    total_states = []
    total_actions = []
    for file_path in file_paths:
        states, actions = load_sensor_file(file_path)
        set_sizes += len(states)
        total_states.extend(states)
        total_actions.extend(actions)

    total_states = array(total_states)
    total_actions = array(total_actions)

    states_scaler = MinMaxScaler()
    actions_scaler = MinMaxScaler()

    total_states = states_scaler.fit_transform(total_states)
    total_actions = actions_scaler.fit_transform(total_actions)

    normalized_state_sets = []
    normalized_action_sets = []
    start = 0
    for size in set_sizes:
        normalized_state_sets += [total_states[start:start+size, :].tolist()]
        normalized_action_sets += [total_actions[start:start+size, :].tolist()]
        start += size

    return normalized_state_sets, normalized_action_sets


def write_state_action_sets_files(data_path, state_sets, action_sets):
    id = 0
    for states, actions in zip(state_sets, action_sets):
        file_path = data_path + "set_{0}.txt".format(id)
        id += 1
        data_str = ""
        for state, action in zip(states, actions):
            data_str += ", ".join(str(val) for val in state) + " | " + ", ".join(str(val) for val in action)
            data_str += '\n'

        with open(file_path, 'w+') as output_file:
            output_file.write(data_str)


if __name__ == '__main__':
    from numpy.random import shuffle
    data_path = "/home/willist/Documents/dnn/data/"
    training_path = data_path + "training/"
    testing_path = data_path + "testing/"
    normalized_states, normalized_actions = prepare_folder_data(data_path)

    indexes = range(len(normalized_states))
    indexes = shuffle(indexes)

    split = len(indexes) * 2/3
    training_indexes, testing_indexes = indexes[:split], indexes[split:]

    write_state_action_sets_files(training_path, normalized_states[training_indexes], normalized_actions[training_indexes])
    write_state_action_sets_files(testing_path, normalized_states[testing_indexes], normalized_actions[testing_indexes])

