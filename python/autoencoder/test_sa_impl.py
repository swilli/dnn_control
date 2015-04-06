import os
from numpy import random, asarray
from stacked_autoencoder import StackedAutoencoder
import theano

user_path = os.path.expanduser("~")
autoencoder_weights_path = user_path + "/Documents/dnn/autoencoder/conf_120_optical_flow_linear/"

numpy_rng = random.RandomState(89677)

stacked_autoencoder = StackedAutoencoder.from_config_path(numpy_rng, autoencoder_weights_path)

samples = asarray(random.rand(5000, stacked_autoencoder.input_dimension()), dtype=theano.config.floatX)
predictions = stacked_autoencoder.predict(samples)

result = ""
for x, y in zip(samples.tolist(), predictions.tolist()):
    str_sample = ", ".join(str(val) for val in x)
    str_y = ", ".join(str(val) for val in y)
    result += str_sample + "\n" + str_y + "\n"

with open(user_path + "/Documents/dnn/results/implementation.txt", 'w+') as output_file:
    output_file.write(result)