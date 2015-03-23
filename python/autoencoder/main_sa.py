from numpy import random, mean
import os
from sys import exit
from time import clock
from data_loader import load_sensor_files, load_autoencoder_weights
from stacked_autoencoder import StackedAutoencoder
from numpy.linalg import norm
from random import sample
import time

ENABLE_FINE_TUNING = False
fine_tuning_epochs = 100

path_suffix = "master"

batch_size = 2
num_training_samples = 1000000
num_training_samples_per_file = 250
num_test_samples = 10000
num_test_samples_per_file = 100


history_length = 5


hidden_layer_sizes = [400, 300, 200, 150, 100, 75, 50, 25, 10]
corruption_levels = [0.1 ** (i+1) for i in range(len(hidden_layer_sizes))]
training_epochs = [800, 400, 400, 400, 400, 400, 400, 400, 400]
training_epochs = [val / 1 for val in training_epochs]
learning_rates = [0.000001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
tied_weights =              [False, True, True, True, True, True, True, True, True]
sigmoid_compressions =      [True, True, True, True, True, True, True, True, True]
sigmoid_reconstructions =   [False, True, True, True, True, True, True, True, True]


training_path = "/home/willist/Documents/dnn/data/training/"
testing_path = "/home/willist/Documents/dnn/data/testing/"

result_path = "/home/willist/Documents/dnn/autoencoder/"
autoencoder_weights_path = "/home/willist/Documents/dnn/autoencoder/"

training_set, test_set = load_sensor_files(training_path, testing_path, history_length=history_length,
                                           num_training_samples=num_training_samples,
                                           num_training_samples_per_file=num_training_samples_per_file,
                                           num_test_samples=num_test_samples,
                                           num_test_samples_per_file=num_test_samples_per_file)

# compute number of minibatches for training, validation and testing
n_train_batches = training_set.get_value(borrow=True).shape[0]
n_train_batches /= batch_size

# numpy random generator
#  start-snippet-3

numpy_rng = random.RandomState(89677)
print '... building the model'
# construct the stacked denoising autoencoder class

sample_dimension = training_set.get_value(borrow=True).shape[1]
print '... sample dimension %d' % sample_dimension
stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, n_ins=sample_dimension,
                                         hidden_layers_sizes=hidden_layer_sizes, tied_weights=tied_weights,
                                         sigmoid_compressions=sigmoid_compressions,
                                         sigmoid_reconstructions=sigmoid_reconstructions)


print '... getting the pre-training functions'
pretraining_fns = stacked_autoencoder.pretraining_functions(train_set_x=training_set, batch_size=batch_size)


if ENABLE_FINE_TUNING:
    print '... getting the fine-tune function'
    finetune_fn = stacked_autoencoder.finetune_function(train_set_x=training_set, batch_size=batch_size)


print '... pre-training the model'
start_time = clock()
# Pre-train layer-wise
for i in xrange(stacked_autoencoder.n_layers):
    # go through pretraining epochs
    learning_rate = learning_rates[i]
    corruption_level = corruption_levels[i]
    for epoch in range(training_epochs[i]):
        # go through the training set
        online_mean = 0.0
        for batch_index in xrange(n_train_batches):
            cur_cost = pretraining_fns[i](index=batch_index, corruption=corruption_level, lr=learning_rate)
            online_mean += (cur_cost - online_mean) / (batch_index + 1)

        print time.strftime("%c")
        print 'Pre-training layer %i, epoch %d, cost ' % (i, epoch),
        print mean(online_mean)

end_time = clock()

print 'The pre-training code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' % ((end_time - start_time) / 60.)


if ENABLE_FINE_TUNING:
    print '... fine-tuning the model'
    start_time = clock()
    # Fine tune complete network
    corruption = 0.0
    learning_rate = 0.0000001

    for epoch in range(fine_tuning_epochs):
        online_mean = 0.0
        for batch_index in xrange(n_train_batches):
            cost = finetune_fn(index=batch_index, corruption=corruption, lr=learning_rate)
            online_mean += (cur_cost - online_mean) / (batch_index + 1)
        print 'Fine-tuning epoch %d, cost ' % epoch,
        print mean(online_mean)


result_path += "conf_" + "_".join([str(value) for value in hidden_layer_sizes]) + "_" + path_suffix
if not os.path.exists(result_path):
    os.makedirs(result_path)

result_path += "/"

for i in xrange(stacked_autoencoder.n_layers):
    output_path = result_path + "l{0}W.txt".format(i)
    output_file = open(output_path, 'w+')
    W = stacked_autoencoder.autoencoder_layers[i].W.get_value(borrow=True).T.tolist()
    W_str = "\n".join(", ".join(map(str, value)) for value in W)
    output_file.write(W_str)
    output_file.close()

    output_path = result_path + "l{0}b.txt".format(i)
    output_file = open(output_path, 'w+')
    b = stacked_autoencoder.autoencoder_layers[i].b.get_value(borrow=True).T.tolist()
    b_str = ", ".join(str(value) for value in b)
    output_file.write(b_str)
    output_file.close()

    output_path = result_path + "l{0}W_prime.txt".format(i)
    output_file = open(output_path, 'w+')
    if stacked_autoencoder.autoencoder_layers[i].tied_weights:
        W_prime = stacked_autoencoder.autoencoder_layers[i].W.get_value(borrow=True).T.T.tolist()
    else:
        W_prime = stacked_autoencoder.autoencoder_layers[i].W_prime.get_value(borrow=True).T.tolist()
    W_prime_str = "\n".join(", ".join(map(str, value)) for value in W_prime)
    output_file.write(W_prime_str)
    output_file.close()

    output_path = result_path + "l{0}b_prime.txt".format(i)
    output_file = open(output_path, 'w+')
    b_prime = stacked_autoencoder.autoencoder_layers[i].b_prime.get_value(borrow=True).T.tolist()
    b_prime_str = ", ".join(str(value) for value in b_prime)
    output_file.write(b_prime_str)
    output_file.close()

test_samples = test_set
mean_error = 0.0
num_tests = 0
for sample in test_samples:
    s_compr = stacked_autoencoder.compress(sample)
    s_decompr = stacked_autoencoder.decompress(s_compr)
    #print("===")
    #print("x: {0}".format(sample))
    #print("y: {0}".format(s_compr))
    #print("z: {0}".format(s_decompr))
    mean_error += norm(sample - s_decompr)
    num_tests += 1
    print("avg error: {0}".format(mean_error / num_tests))
    #print("")


