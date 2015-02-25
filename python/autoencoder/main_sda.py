from numpy import random, mean
import os
from time import clock
from sensor_data_loader import load_sensor_files
from stacked_denoising_autoencoder import StackedDenoisingAutoencoder
from numpy.linalg import norm
from random import sample

pretraining_epochs = 5000
pretraining_learning_rate = 0.05
batch_size = 100
input_size = 18
hidden_layer_sizes = [6, 3]
corruption_levels = [0.05, 0.001]
data_path = "/home/willist/Documents/dnn/data/"

training_set, test_set = load_sensor_files(data_path)

# compute number of minibatches for training, validation and testing
n_train_batches = training_set.get_value(borrow=True).shape[0]
n_train_batches /= batch_size

# numpy random generator
#  start-snippet-3

numpy_rng = random.RandomState(89677)
print '... building the model'
# construct the stacked denoising autoencoder class

sda = StackedDenoisingAutoencoder(
        numpy_rng=numpy_rng,
        n_ins=input_size,
        hidden_layers_sizes=hidden_layer_sizes)


    # end-snippet-3 start-snippet-4
    #########################
    # PRETRAINING THE MODEL #
    #########################
print '... getting the pretraining functions'
pretraining_fns = sda.pretraining_functions(train_set_x=training_set,
                                                batch_size=batch_size)

print '... pre-training the model'
start_time = clock()
## Pre-train layer-wise
for i in xrange(sda.n_layers):
    # go through pretraining epochs
    for epoch in xrange(pretraining_epochs):
        # go through the training set
        c = []
        for batch_index in xrange(n_train_batches):
            c.append(pretraining_fns[i](index=batch_index,
                     corruption=corruption_levels[i],
                     lr=pretraining_learning_rate))
        print 'Pre-training layer %i, epoch %d, cost ' % (i, epoch),
        print mean(c)

end_time = clock()

print 'The pretraining code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' % ((end_time - start_time) / 60.)

samples = training_set.get_value(borrow=True)
num_test_samples = 1000
test_samples = sample(test_set.get_value(borrow=True), num_test_samples)
mean_error = 0.0
num_tests = 0
for sample in test_samples:
    s_corrupt = sda.corrupt(sample)
    s_compr = sda.compress(s_corrupt)
    s_decompr = sda.decompress(s_compr)
    mean_error += norm(sample - s_decompr)
    num_tests += 1
    print(mean_error / num_tests)

print mean_error / num_test_samples
