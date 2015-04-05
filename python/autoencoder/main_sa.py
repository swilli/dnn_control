import time
import os
import theano
from sys import exit
from time import clock
from data_loader import load_sensor_files
from stacked_autoencoder import StackedAutoencoder
from numpy import random, mean, inf, asarray
from numpy.linalg import norm
from random import sample

data_set = "optical_flow"

user_path = os.path.expanduser("~")

training_path = user_path + "/Documents/dnn/data/" + data_set + "/training/"
testing_path = user_path + "/Documents/dnn/data/" + data_set + "/testing/"

result_path = user_path + "/Documents/dnn/autoencoder/"
autoencoder_weights_path = user_path + "/Documents/dnn/autoencoder/"

feature_indexes = [val for val in range(3, 12)]
label_indexes = [val for val in range(3)]
num_training_samples = 1000000
num_training_samples_per_file = 250
num_test_samples = 10000
num_test_samples_per_file = 50
history_length = 10

batch_size = 1

pretraining_epochs = 50

ENABLE_FINE_TUNING = True
fine_tune_supervised = True
fine_tune_learning_rate = 0.00001
fine_tune_epochs = 1000
supervised_sigmoid_activation = False

hidden_layer_sizes = [120]
corruption_levels = [0.01]
pretraining_learning_rates = [0.001]
tied_weights =              [True]
sigmoid_compressions =      [False]
sigmoid_reconstructions =   [False]


description = "Data Set: " + data_set + "\n" + \
              "Training Path: " + training_path + "\n" + \
              "Testing Path: " + testing_path + "\n" + \
              "Feature Indexes: " + str(feature_indexes) + "\n" + \
              "Label Indexes: " + str(label_indexes) + "\n" + \
              "Number of Training Samples: " + str(num_training_samples) + "\n" + \
              "Number of Training Samples per File: " + str(num_training_samples_per_file) + "\n" + \
              "Number of Testing Samples: " + str(num_test_samples) + "\n" + \
              "Number of Testing Samples per File: " + str(num_test_samples_per_file) + "\n" + \
              "History Length: " + str(history_length) + "\n" + \
              "Batch Size: " + str(batch_size) + "\n" + \
              "Pretraining Epochs: " + str(pretraining_epochs) + "\n" + \
              "Fine Tuning Enabled: " + str(ENABLE_FINE_TUNING) + "\n" + \
              "Fine Tune Learning Rate: " + str(fine_tune_learning_rate) + "\n" + \
              "Fine Tune Epochs: " + str(fine_tune_epochs) + "\n" + \
              "Fine Tune Supervised: " + str(fine_tune_supervised) + "\n" + \
              "Hidden Layer Sizes: " + str(hidden_layer_sizes) + "\n" + \
              "Corruption Levels: " + str(corruption_levels) + "\n" + \
              "Pretraining Learning Rates: " + str(pretraining_learning_rates) + "\n" + \
              "Tied Weights: " + str(tied_weights) + "\n" + \
              "Sigmoid Compressions: " + str(sigmoid_compressions) + "\n" + \
              "Sigmoid Reconstructions: " + str(sigmoid_reconstructions) + "\n" + \
              "Supervised Sigmoid Activation: " + str(supervised_sigmoid_activation) + "\n"

print(description)

training_set, training_labels, test_set, test_labels = load_sensor_files(training_path, testing_path,
                                                                         history_length=history_length,
                                                                         num_training_samples=num_training_samples,
                                                                         num_training_samples_per_file=num_training_samples_per_file,
                                                                         num_test_samples=num_test_samples,
                                                                         num_test_samples_per_file=num_test_samples_per_file,
                                                                         feature_indexes=feature_indexes,
                                                                         label_indexes=label_indexes)

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
                                         hidden_layer_sizes=hidden_layer_sizes, tied_weights=tied_weights,
                                         sigmoid_compressions=sigmoid_compressions,
                                         sigmoid_reconstructions=sigmoid_reconstructions,
                                         supervised_sigmoid_activation=supervised_sigmoid_activation)


print '... getting the pre-training functions'
pretraining_fns = stacked_autoencoder.pretraining_functions(training_set=training_set, batch_size=batch_size)


if ENABLE_FINE_TUNING:
    print '... getting the fine-tune function'
    if fine_tune_supervised:
        finetune_fn, validate_model = stacked_autoencoder.finetune_functions(training_set=training_set,
                                                                             training_labels=training_labels,
                                                                             test_set=test_set, test_labels=test_labels,
                                                                             batch_size=batch_size,
                                                                             learning_rate=fine_tune_learning_rate)
    else:
        finetune_fn, validate_model = stacked_autoencoder.finetune_functions_unsupervised(training_set=training_set,
                                                                                      test_set=test_set,
                                                                                      batch_size=batch_size,
                                                                                      learning_rate=fine_tune_learning_rate)


print '... pre-training the model'
start_time = clock()
# Pre-train layer-wise
for i in range(stacked_autoencoder.n_layers):
    # go through pretraining epochs
    learning_rate = pretraining_learning_rates[i]
    corruption_level = corruption_levels[i]
    for epoch in range(pretraining_epochs):
        # go through the training set
        c = []
        for batch_index in xrange(n_train_batches):
            cur_cost = pretraining_fns[i](index=batch_index, corruption=corruption_level, lr=learning_rate)
            c.append(cur_cost)

        print '... ' + time.strftime("%c") + ': pre-training layer %i, epoch %d, cost %.20f' % (i+1, epoch, mean(c))

end_time = clock()

print '... the pre-training code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' % ((end_time - start_time) / 60.)


if ENABLE_FINE_TUNING:
    print '... fine-tuning the model'
    patience = 10 * n_train_batches  # look as this many examples regardless
    patience_increase = 2.  # wait this much longer when a new best is
                            # found
    improvement_threshold = 0.99999  # a relative improvement of this much is
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
    best_iter = 0

    while epoch < fine_tune_epochs and not done_looping:
        epoch += 1
        for minibatch_index in xrange(n_train_batches):
            minibatch_avg_cost = finetune_fn(minibatch_index)
            iter = (epoch - 1) * n_train_batches + minibatch_index

            if (iter + 1) % validation_frequency == 0:
                validation_losses = validate_model()
                this_validation_loss = mean(validation_losses)
                print '... ' + time.strftime("%c") + ': epoch %i, minibatch %i/%i, validation error %.20f' % (epoch, minibatch_index + 1,
                                                                                                          n_train_batches,this_validation_loss)

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
    print '... optimization complete with best validation score of %.20f on iteration %i' % (best_validation_loss,
                                                                                      best_iter + 1)
    print '... the training code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' %\
                                                                       ((end_time - start_time) / 60.)


result_path += "conf_" + "_".join([str(value) for value in hidden_layer_sizes]) + "_" + data_set + "/"
if not os.path.exists(result_path):
    os.makedirs(result_path)

print '... saving stacked autencoder config to path ' + result_path
stacked_autoencoder.save(result_path)
reloaded_sa = StackedAutoencoder.from_config_path(numpy_rng, result_path)

num_tests = 10
samples = asarray(random.rand(num_tests, sample_dimension), dtype=theano.config.floatX)

errors = []
for sample in samples:
    result_a = stacked_autoencoder.predict(sample)
    result_reloaded = reloaded_sa.predict(sample)
    errors.extend([norm(result_a - result_reloaded)])
    print(mean(errors))






