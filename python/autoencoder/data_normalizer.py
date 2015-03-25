def create_and_save_hist(data, index, output_folder):
    import matplotlib.pyplot as plt

    n_bins = 100
    plt.hist(data[:, index], bins=n_bins)
    file_path = output_folder + "hist_{0}.svg".format(index)
    print '... created histogram and saving it to file ' + file_path
    plt.savefig(file_path)
    plt.close()


def create_histograms(data, path_to_data):
    import thread
    import os

    output_folder = path_to_data + "histograms/"
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    n_features = data.shape[1]
    for i in range(n_features):
        #thread.start_new(create_and_save_hist, (data, i, output_folder, ))
        create_and_save_hist(data, i, output_folder)


def load_data(data_folder):
    from os import listdir
    from data_loader import load_sensor_file

    file_names = listdir(data_folder)
    file_names = [name for name in file_names if "sensor_stream" in name]
    file_names = sorted(file_names)
    file_paths = [data_folder + name for name in file_names]

    set_sizes = []
    total_states = []
    total_actions = []
    for file_path in file_paths:
        print '... loading all samples from data file ' + file_path
        states, actions = load_sensor_file(file_path)
        set_sizes.append(len(states))
        total_states.extend(states)
        total_actions.extend(actions)

    return total_states, total_actions, set_sizes


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
            print '... writing normalized set to file ' + file_path
            output_file.write(data_str)


def normalize_folder_data(input_data_path, output_data_path):
    from numpy import array
    from sklearn.preprocessing import MinMaxScaler

    total_states, total_actions, set_sizes = load_data(input_data_path)
    total_states = array(total_states)
    total_actions = array(total_actions)

    states_scaler = MinMaxScaler()
    actions_scaler = MinMaxScaler()

    total_states = states_scaler.fit_transform(total_states)
    total_actions = actions_scaler.fit_transform(total_actions)

    normalized_states_sets = []
    normalized_actions_sets = []
    start = 0
    for size in set_sizes:
        normalized_states_sets += [total_states[start:start+size, :].tolist()]
        normalized_actions_sets += [total_actions[start:start+size, :].tolist()]
        start += size

    training_path = output_data_path + "training/"
    testing_path = output_data_path + "testing/"
    if not os.path.exists(training_path):
        os.makedirs(training_path)

    if not os.path.exists(testing_path):
        os.makedirs(testing_path)

    indexes = range(len(normalized_states_sets))
    shuffle(indexes)

    split = len(indexes) * 2/3
    training_indexes, testing_indexes = indexes[:split], indexes[split:]

    training_states_set = []
    training_actions_set = []
    for index in training_indexes:
        training_states_set += [normalized_states_sets[index]]
        training_actions_set += [normalized_actions_sets[index]]

    testing_states_set = []
    testing_actions_set = []
    for index in testing_indexes:
        testing_states_set += [normalized_states_sets[index]]
        testing_actions_set += [normalized_actions_sets[index]]

    write_state_action_sets_files(training_path, training_states_set, training_actions_set)
    write_state_action_sets_files(testing_path, testing_states_set, testing_actions_set)


def analyze_folder_data(input_data_path, output_data_path):
    from numpy import array, concatenate
    total_states, total_actions, set_sizes = load_data(input_data_path)
    total_states = array(total_states)
    total_actions = array(total_actions)

    create_histograms(concatenate((total_states, total_actions), axis=1), output_data_path)


if __name__ == '__main__':
    from numpy.random import shuffle
    import os

    input_data_path = "/home/willist/Documents/dnn/data/raw/"
    output_data_path = "/home/willist/Documents/dnn/data/"
    analyze_folder_data(input_data_path, output_data_path)
    #normalize_folder_data(input_data_path, output_data_path)


