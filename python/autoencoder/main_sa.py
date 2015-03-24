from numpy import random, mean
import os
from sys import exit
from time import clock
from data_loader import load_sensor_files
from stacked_autoencoder import StackedAutoencoder
from numpy.linalg import norm
from numpy import inf
from random import sample
import time
import sys

ENABLE_FINE_TUNING = True
fine_tune_learning_rate = 0.1
fine_tune_epochs = 1000
pretraining_epochs = 100

path_suffix = "master"

batch_size = 1
num_training_samples = 1000000
num_training_samples_per_file = 250
num_test_samples = 10000
num_test_samples_per_file = 100


history_length = 5


hidden_layer_sizes = [100, 100]
corruption_levels = [0.1 ** (i+1) for i in range(len(hidden_layer_sizes))]
learning_rates = [0.001, 0.1, 0.1, 0.1, 0.1]
tied_weights =              [False, True, True, True, True]
sigmoid_compressions =      [True, True, True, True, True]
sigmoid_reconstructions =   [False, True, True, True, True]


training_path = "/home/willist/Documents/dnn/data/training/"
testing_path = "/home/willist/Documents/dnn/data/testing/"

result_path = "/home/willist/Documents/dnn/autoencoder/"
autoencoder_weights_path = "/home/willist/Documents/dnn/autoencoder/"


training_set, training_labels, test_set, test_labels = load_sensor_files(training_path, testing_path,
                                                                         history_length=history_length,
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
label_dimension = training_labels.get_value(borrow=True).shape[1]
print '... sample dimension %d' % sample_dimension
print '... label dimension %d' % label_dimension

stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, n_ins=sample_dimension, n_outs=label_dimension,
                                         hidden_layers_sizes=hidden_layer_sizes, tied_weights=tied_weights,
                                         sigmoid_compressions=sigmoid_compressions,
                                         sigmoid_reconstructions=sigmoid_reconstructions)


print '... getting the pre-training functions'
pretraining_fns = stacked_autoencoder.pretraining_functions(train_set_x=training_set, batch_size=batch_size)


if ENABLE_FINE_TUNING:
    print '... getting the fine-tune function'
    finetune_fn, validate_model = stacked_autoencoder.finetune_functions(training_set=training_set,
                                                                         training_labels=training_labels,
                                                                         test_set=test_set,
                                                                         test_labels=test_labels,
                                                                         batch_size=batch_size,
                                                                         learning_rate=fine_tune_learning_rate)


print '... pre-training the model'
start_time = clock()
# Pre-train layer-wise
for i in xrange(stacked_autoencoder.n_layers):
    # go through pretraining epochs
    learning_rate = learning_rates[i]
    corruption_level = corruption_levels[i]
    for epoch in range(pretraining_epochs):
        # go through the training set
        c = []
        for batch_index in xrange(n_train_batches):
            cur_cost = pretraining_fns[i](index=batch_index, corruption=corruption_level, lr=learning_rate)
            c.append(cur_cost)

        print time.strftime("%c")
        print 'Pre-training layer %i, epoch %d, cost ' % (i+1, epoch),
        print mean(c)

end_time = clock()

print 'The pre-training code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' % ((end_time - start_time) / 60.)


if ENABLE_FINE_TUNING:
    print '... fine-tuning the model'
    patience = 10 * n_train_batches  # look as this many examples regardless
    patience_increase = 2.  # wait this much longer when a new best is
                            # found
    improvement_threshold = 0.995  # a relative improvement of this much is
                                   # considered significant
    validation_frequency = min(n_train_batches, patience / 2)
                                  # go through this many
                                  # minibatche before checking the network
                                  # on the validation set; in this case we
                                  # check every epoch

    best_validation_loss = inf
    start_time = time.clock()

    done_looping = False
    epoch = 0

    while (epoch < fine_tune_epochs) and (not done_looping):
        epoch += 1
        for minibatch_index in xrange(n_train_batches):
            minibatch_avg_cost = finetune_fn(minibatch_index)
            iter = (epoch - 1) * n_train_batches + minibatch_index

            if (iter + 1) % validation_frequency == 0:
                validation_losses = validate_model()
                this_validation_loss = mean(validation_losses)
                print 'epoch %i, minibatch %i/%i, validation error %f' % (epoch, minibatch_index + 1, n_train_batches,
                                                                          this_validation_loss)

                # if we got the best validation score until now
                if this_validation_loss < best_validation_loss:
                    # improve patience if loss improvement is good enough
                    if this_validation_loss < best_validation_loss * improvement_threshold:
                        patience = max(patience, iter * patience_increase)

                    # save best validation score and iteration number
                    best_validation_loss = this_validation_loss
                    best_iter = iter

            if patience <= iter:
                done_looping = True
                break

    end_time = time.clock()
    print 'Optimization complete with best validation score of %f on iteration %i' % (best_validation_loss,
                                                                                      best_iter + 1)
    print 'The training code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' %\
                                                                       ((end_time - start_time) / 60.)


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

test_samples = []
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


