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


def load_data(data_folder, num_lines=None):
    from os import listdir
    from data_loader import load_sensor_file

    file_names = listdir(data_folder)
    file_names = [name for name in file_names if "sensor_stream" in name]
    file_names = sorted(file_names)
    file_paths = [data_folder + name for name in file_names]

    set_sizes = []
    total_states = []
    total_actions = []
    num_loaded = 0
    for file_path in file_paths:
        #print '... loading samples from data file ' + file_path
        states, actions = load_sensor_file(file_path, num_lines)
        num_loaded += 1
        if (num_loaded % 1000) == 0:
            print(num_loaded)

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


def normalize_folder_data(input_data_path, output_data_path, gaussian_state_scaling=True, num_samples=None):
    from numpy import array, concatenate, mean, std
    from sklearn.preprocessing import StandardScaler, MinMaxScaler

    total_states, total_actions, set_sizes = load_data(input_data_path, num_samples)
    total_states = array(total_states)
    total_actions = array(total_actions)

    if gaussian_state_scaling:
        states_scaler = StandardScaler()
    else:
        states_scaler = MinMaxScaler()

    actions_scaler = MinMaxScaler()

    states_scaler.fit(total_states)
    total_states = states_scaler.transform(total_states)
    if gaussian_state_scaling:
        total_states *= 0.25
        total_states += 0.5

    actions_scaler.fit(total_actions)
    total_actions = actions_scaler.transform(total_actions)

    create_histograms(concatenate((total_states, total_actions), axis=1), output_data_path)

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

    if gaussian_state_scaling:
        print("States standardization: Means: {0}, Stdevs: {1}".format(states_scaler.mean_, states_scaler.std_))
        print("States distribution: Means: {0}, Stdevs: {1}".format(mean(total_states, axis=0), std(total_states, axis=0)))
    else:
        print("States standardization: Minima: {0}, Ranges: {1}".format(states_scaler.data_min, states_scaler.data_range))
        print("States distribution: Means: {0}, Stdevs: {1}".format(mean(total_states, axis=0), std(total_states, axis=0)))

    print("Actions standardization: Minima: {0}, Ranges: {1}".format(actions_scaler.data_min, actions_scaler.data_range))
    print("Actions distribution: Means: {0}, Stdevs: {1}".format(mean(total_actions, axis=0), std(total_actions, axis=0)))


def analyze_folder_data(input_data_path, output_data_path, gaussian_state_scaling=True, num_samples=None):
    from numpy import array, concatenate, std, mean
    from sklearn.preprocessing import StandardScaler, MinMaxScaler

    total_states, total_actions, set_sizes = load_data(input_data_path, num_samples)

    total_states = array(total_states)
    total_actions = array(total_actions)

    if gaussian_state_scaling:
        states_scaler = StandardScaler()
    else:
        states_scaler = MinMaxScaler()

    actions_scaler = MinMaxScaler()

    states_scaler.fit(total_states)
    total_states = states_scaler.transform(total_states)
    if gaussian_state_scaling:
        total_states *= 0.25
        total_states += 0.5

    actions_scaler.fit(total_actions)
    total_actions = actions_scaler.transform(total_actions)

    create_histograms(concatenate((total_states, total_actions), axis=1), output_data_path)

    if gaussian_state_scaling:
        print("States standardization: Means: {0}, Stdevs: {1}".format(states_scaler.mean_, states_scaler.std_))
        print("States distribution: Means: {0}, Stdevs: {1}".format(mean(total_states, axis=0), std(total_states, axis=0)))
    else:
        print("States standardization: Minima: {0}, Ranges: {1}".format(states_scaler.data_min, states_scaler.data_range))
        print("States distribution: Means: {0}, Stdevs: {1}".format(mean(total_states, axis=0), std(total_states, axis=0)))

    print("Actions standardization: Minima: {0}, Ranges: {1}".format(actions_scaler.data_min, actions_scaler.data_range))
    print("Actions distribution: Means: {0}, Stdevs: {1}".format(mean(total_actions, axis=0), std(total_actions, axis=0)))


if __name__ == '__main__':
    from numpy.random import shuffle
    import os

    input_data_path = "/home/willist/Documents/dnn/data/raw/"
    output_data_path = "/home/willist/Documents/dnn/data/raw/"

    num_samples = None
    gaussian_standardization = False
    #analyze_folder_data(input_data_path, output_data_path, gaussian_standardization, num_samples)
    normalize_folder_data(input_data_path, output_data_path, gaussian_standardization, num_samples)


