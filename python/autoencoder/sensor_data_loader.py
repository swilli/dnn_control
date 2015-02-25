def shared_dataset(data, borrow=True):
    from theano import shared
    from theano import config as theano_config
    from numpy import asarray

    shared_data = shared(asarray(data, dtype=theano_config.floatX), borrow=borrow)
    return shared_data


def load_sensor_file(file_path):
    from numpy import array

    print '... loading data' % file_path

    sensor_data_file = open(file_path, 'r')
    lines = sensor_data_file.readlines()
    lines = [line for line in lines if not line.startswith("#")]
    num_samples = len(lines)
    data_set = [line.split(',') for line in lines]
    for i in range(num_samples):
        for j in range(len(data_set[i])):
            data_set[i][j] = float(data_set[i][j])

    sensor_data_file.close()

    data_set = array(data_set)
    shared_data_set = shared_dataset(data_set)
    return shared_data_set


def load_sensor_files(data_path, num_training_samples=100000, num_test_samples=10000, shared=True):
    from numpy import array
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

    total_training_set = []
    for file_path in training_file_paths:
        print '... loading training data ' + file_path
        sensor_data_file = open(file_path, 'r')
        lines = sensor_data_file.readlines()
        sensor_data_file.close()
        lines = [line for line in lines if not line.startswith("#")]
        lines = lines[:len(lines)/10]
        data_set = [line.split(',') for line in lines]
        data_set = [[float(value) for value in data_line] for data_line in data_set]
        total_training_set = total_training_set + data_set
        if len(total_training_set) >= num_training_samples:
            break

    total_test_set = []
    for file_path in test_file_paths:
        print '... loading test data ' + file_path
        sensor_data_file = open(file_path, 'r')
        lines = sensor_data_file.readlines()
        sensor_data_file.close()
        lines = [line for line in lines if not line.startswith("#")]
        lines = sample(lines, 500)
        data_set = [line.split(',') for line in lines]
        data_set = [[float(value) for value in data_line] for data_line in data_set]
        total_test_set = total_test_set + data_set
        if len(total_test_set) >= num_test_samples:
            break

    total_training_set = array(total_training_set[:num_training_samples])
    total_test_set = array(total_test_set[:num_test_samples])

    if shared:
        total_training_set = shared_dataset(total_training_set)
        total_test_set = shared_dataset(total_test_set)

    return total_training_set, total_test_set

