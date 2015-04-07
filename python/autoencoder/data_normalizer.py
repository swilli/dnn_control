def create_and_save_hist(data, index, output_folder):
    import matplotlib.pyplot as plt

    n_bins = 100
    plt.hist(data[:, index], bins=n_bins)
    file_path = output_folder + "hist_{0}.svg".format(index)
    print '... created histogram and saving it to file ' + file_path
    plt.savefig(file_path)
    plt.close()


def create_histograms(data, output_folder):
    import os

    output_folder = output_folder + "histograms/"
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

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
        states, actions = load_sensor_file(file_path, num_lines)
        num_loaded += len(states)
        if (num_loaded % 100) == 0:
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


def normalize_folder_data(input_data_path, output_data_path, gaussian_state_scaling=False, logarithmic_state_transformation=True,
                          num_samples=None):
    from numpy import array, mean, std, min, ptp, log
    from sklearn.preprocessing import StandardScaler, MinMaxScaler

    total_states, total_actions, set_sizes = load_data(input_data_path, num_samples)
    total_states = array(total_states)
    total_actions = array(total_actions)

    if logarithmic_state_transformation:
        def transform(features):
            from math import log

            for i in range(len(features)):
                feature = features[i]
                sign = 1.0
                if feature >= 0.0:
                    feature += 1.0
                else:
                    sign = -1.0
                    feature -= 1.0

                feature = sign * log(sign * feature)
                features[i] = feature

        map(transform, total_states)

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

    log_str = ""
    if gaussian_state_scaling:
        log_str += "States standardization:\n\tMeans: [{0}]\n\tStdevs: [{1}]\n".format(", ".join(str(val) for val in states_scaler.mean_.tolist()),
                                                                                       ", ".join(str(val) for val in states_scaler.std_.tolist()))
        log_str += "Standardized states distribution:\n\tMeans: [{0}]\n\tStdevs: [{1}]\n".format(", ".join(str(val) for val in mean(total_states, axis=0).tolist()),
                                                                                                 ", ".join(str(val) for val in std(total_states, axis=0).tolist()))
    else:
        log_str += "States standardization:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in states_scaler.data_min.tolist()),
                                                                                        ", ".join(str(val) for val in states_scaler.data_range.tolist()))
        log_str += "Standardized states distribution:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in min(total_states, axis=0).tolist()),
                                                                                                  ", ".join(str(val) for val in ptp(total_states, axis=0).tolist()))

    log_str += "Actions standardization:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in actions_scaler.data_min.tolist()),
                                                                                     ", ".join(str(val) for val in actions_scaler.data_range.tolist()))
    log_str += "Standardized actions distribution:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in min(total_actions, axis=0).tolist()),
                                                                                               ", ".join(str(val) for val in ptp(total_actions, axis=0).tolist()))

    print(log_str)


def analyze_folder_data(data_path, gaussian_state_scaling=False, logarithmic_state_transformation=True, num_samples=None):
    from numpy import array, concatenate, std, mean, min, ptp
    from sklearn.preprocessing import StandardScaler, MinMaxScaler
    import os

    output_folder = data_path + "analysis/"
    if not os.path.exists(output_folder):
        os.mkdir(output_folder)

    total_states, total_actions, set_sizes = load_data(data_path, num_samples)

    total_states = array(total_states)
    total_actions = array(total_actions)

    if logarithmic_state_transformation:
        def transform(features):
            from math import log

            for i in range(len(features)):
                feature = features[i]
                sign = 1.0
                if feature >= 0.0:
                    feature += 1.0
                else:
                    sign = -1.0
                    feature -= 1.0

                feature = sign * log(sign * feature)
                features[i] = feature

        map(transform, total_states)

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

    create_histograms(concatenate((total_states, total_actions), axis=1), output_folder)

    log_str = ""
    if gaussian_state_scaling:
        log_str += "States standardization:\n\tMeans: [{0}]\n\tStdevs: [{1}]\n".format(", ".join(str(val) for val in states_scaler.mean_.tolist()),
                                                                                       ", ".join(str(val) for val in states_scaler.std_.tolist()))
        log_str += "Standardized states distribution:\n\tMeans: [{0}]\n\tStdevs: [{1}]\n".format(", ".join(str(val) for val in mean(total_states, axis=0).tolist()),
                                                                                                 ", ".join(str(val) for val in std(total_states, axis=0).tolist()))
    else:
        log_str += "States standardization:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in states_scaler.data_min.tolist()),
                                                                                        ", ".join(str(val) for val in states_scaler.data_range.tolist()))
        log_str += "Standardized states distribution:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in min(total_states, axis=0).tolist()),
                                                                                                  ", ".join(str(val) for val in ptp(total_states, axis=0).tolist()))

    log_str += "Actions standardization:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in actions_scaler.data_min.tolist()),
                                                                                     ", ".join(str(val) for val in actions_scaler.data_range.tolist()))
    log_str += "Standardized actions distribution:\n\tMinima: [{0}]\n\tRanges: [{1}]\n".format(", ".join(str(val) for val in min(total_actions, axis=0).tolist()),
                                                                                               ", ".join(str(val) for val in ptp(total_actions, axis=0).tolist()))

    print(log_str)

    log_path = data_path + "analysis/distributions.txt"

    with open(log_path, 'w+') as log_file:
        print '... writing distribution properties to file ' + log_path
        log_file.write(log_str)

if __name__ == '__main__':
    from numpy.random import shuffle
    import os

    user_path = os.path.expanduser("~")

    data_set_name = "optical_flow"
    num_samples = None
    gaussian_standardization = True
    logarithmic_state_transformation = False

    input_data_path = user_path + "/Documents/dnn/data/" + data_set_name + "/raw/"
    output_data_path = user_path + "/Documents/dnn/data/" + data_set_name + "/"

    analyze_folder_data(data_path=input_data_path,
                        gaussian_state_scaling=gaussian_standardization,
                        logarithmic_state_transformation=logarithmic_state_transformation,
                        num_samples=num_samples)

    normalize_folder_data(input_data_path=input_data_path,
                          output_data_path=output_data_path,
                          gaussian_state_scaling=gaussian_standardization,
                          logarithmic_state_transformation=logarithmic_state_transformation,
                          num_samples=num_samples)


