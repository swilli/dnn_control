from numpy import random, mean
import os
from time import clock
from sensor_data_loader import load_sensor_files
from stacked_autoencoder import StackedAutoencoder
from numpy.linalg import norm
from random import sample

learning_rate = 0.01
training_epochs = 50
batch_size = 1
hidden_layer_sizes = [200, 200, 100, 50, 25, 5]
corruption_level = 0.2
history_length = 10
data_path = "/home/willist/Documents/dnn/data/"
result_path = "/home/willist/Documents/dnn/autoencoder/"

training_set, test_set = load_sensor_files(data_path, history_length=history_length)

# compute number of minibatches for training, validation and testing
n_train_batches = training_set.get_value(borrow=True).shape[0]
n_train_batches /= batch_size

# numpy random generator
#  start-snippet-3

numpy_rng = random.RandomState(89677)
print '... building the model'
# construct the stacked denoising autoencoder class

stacked_autoencoder = StackedAutoencoder(numpy_rng=numpy_rng, n_ins=training_set.get_value(borrow=True).shape[1], hidden_layers_sizes=hidden_layer_sizes)

print '... getting the pretraining functions'
pretraining_fns = stacked_autoencoder.pretraining_functions(train_set_x=training_set, batch_size=batch_size)

print '... pre-training the model'
start_time = clock()
## Pre-train layer-wise
for i in xrange(stacked_autoencoder.n_layers):
    # go through pretraining epochs
    for epoch in xrange(training_epochs):
        # go through the training set
        c = []
        for batch_index in xrange(n_train_batches):
            c.append(pretraining_fns[i](index=batch_index, corruption=corruption_level, lr=learning_rate))
        print 'Pre-training layer %i, epoch %d, cost ' % (i, epoch),
        print mean(c)

end_time = clock()

print 'The pretraining code for file ' + os.path.split(__file__)[1] + ' ran for %.2fm' % ((end_time - start_time) / 60.)

num_test_samples = 100
test_samples = sample(test_set, num_test_samples)
mean_error = 0.0
num_tests = 0
for sample in test_samples:
    print("===")
    s_compr = stacked_autoencoder.compress(sample)
    s_decompr = stacked_autoencoder.decompress(s_compr)
    print("x: {0}".format(sample))
    print("y: {0}".format(s_compr))
    print("z: {0}".format(s_decompr))
    mean_error += norm(sample - s_decompr)
    num_tests += 1
    print("avg error: {0}".format(mean_error / num_tests))
    print("")

for i in xrange(stacked_autoencoder.n_layers):
    output_path = result_path + "layer_{0}_W.txt".format(i+1)
    output_file = open(output_path, 'w+')
    W = stacked_autoencoder.dA_layers[i].W.get_value(borrow=True).T.tolist()
    W_str = "\n".join(", ".join(map(str, value)) for value in W)
    output_file.write(W_str)
    output_file.close()
    output_path = result_path + "layer_{0}_b.txt".format(i+1)
    output_file = open(output_path, 'w+')
    b = stacked_autoencoder.dA_layers[i].b.get_value(borrow=True).T.tolist()
    b_str = ", ".join(str(value) for value in b)
    output_file.write(b_str)
    output_file.close()
