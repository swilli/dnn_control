class DenoisingAutoencoder:
    '''
        Adapted class from http://deeplearning.net/tutorial/dA.html
        The autoencoder uses tied weights.
    '''

    def __init__(self, size_input, size_compressed, learning_rate=1.0, corruption_level=0.0, weights=None,
                 input=None):
        from numpy import asarray, random, zeros, sqrt
        from theano import config, shared
        from theano.tensor import fmatrix
        from theano.tensor.shared_randomstreams import RandomStreams

        self.size_input = size_input
        self.size_compressed = size_compressed
        self._learning_rate = learning_rate
        self._corruption_level = corruption_level

        self._random = RandomStreams(random.randint(2 ** 30))

        if weights is None:
            initial_weights = asarray(random.uniform(
                low=-4.0 * sqrt(6.0 / (self.size_input + self.size_compressed)),
                high=4.0 * sqrt(6.0 / (self.size_input + self.size_compressed)),
                size=(self.size_input, self.size_compressed)), dtype=config.floatX)
            weights = shared(value=initial_weights, name='W', borrow=True)

        bias_input = shared(value=zeros(self.size_input, dtype=config.floatX), borrow=True)

        bias_compressed = shared(value=zeros(self.size_compressed, dtype=config.floatX),
                                 name='b', borrow=True)

        if input is None:
            self._input = fmatrix('input')
        else:
            self._input = input

        self._weights = weights

        self._bias_input = bias_input

        self._bias_compressed = bias_compressed

        self._weights_transpose = self._weights.T

        self._parameters = [self._weights, self._bias_compressed, self._bias_input]

    def _corrupt_input(self):
        from theano import config

        return self._random.binomial(size=self._input.shape, n=1, p=1.0 - self._corruption_level, dtype=config.floatX) * self._input

    def _compress(self, data):
        from theano.tensor import nnet, dot

        return nnet.sigmoid(dot(data, self._weights) + self._bias_compressed)

    def _decompress(self, compressed_data):
        from theano.tensor import nnet, dot

        return nnet.sigmoid(dot(compressed_data, self._weights_transpose) + self._bias_input)

    def get_cost_updates(self):
        from theano.tensor import sum, mean, grad, log

        data_corrupted = self._corrupt_input()
        data_compressed = self._compress(data_corrupted)
        data_uncompressed = self._decompress(data_compressed)

        cost = mean(sum((self._input - data_uncompressed)**2, axis=1))
        #cost = mean(-sum(data * log(data_uncompressed) + (1 - data) * log(1 - data_uncompressed), axis=1))

        grad_parameters = grad(cost, self._parameters)
        updates = [
            (param, param - self._learning_rate * grad_param)
            for param, grad_param in zip(self._parameters, grad_parameters)
        ]

        return cost, updates

    def compress(self, data):
        from theano.tensor import nnet, dot
        from theano import function

        return function([], nnet.sigmoid(dot(data, self._weights) + self._bias_compressed))()

    def decompress(self, compressed_data):
        from theano.tensor import nnet, dot
        from theano import function

        return function([], nnet.sigmoid(dot(compressed_data, self._weights_transpose) + self._bias_input))()