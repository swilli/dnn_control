import numpy

import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams


class Autoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, input=None, n_visible=10, n_hidden=5, W=None, bhid=None, bvis=None):
        self.n_visible = n_visible
        self.n_hidden = n_hidden

        if not theano_rng:
            theano_rng = RandomStreams(numpy_rng.randint(2 ** 30))

        if not W:
            initial_W = numpy.asarray(numpy_rng.uniform(low=-4 * numpy.sqrt(6. / (n_hidden + n_visible)),
                                                        high=4 * numpy.sqrt(6. / (n_hidden + n_visible)),
                                                        size=(n_visible, n_hidden)), dtype=theano.config.floatX)
            W = theano.shared(value=initial_W, name='W', borrow=True)

        if not bvis:
            bvis = theano.shared(value=numpy.zeros(n_visible, dtype=theano.config.floatX), borrow=True)

        if not bhid:
            bhid = theano.shared(value=numpy.zeros(n_hidden, dtype=theano.config.floatX), name='b', borrow=True)

        self.W = W
        self.b = bhid
        self.b_prime = bvis
        self.W_prime = self.W.T
        self.theano_rng = theano_rng
        if input is None:
            self.x = T.dmatrix(name='input')
        else:
            self.x = input

        self.params = [self.W, self.b, self.b_prime]

    def _get_hidden_values(self, input):
        return T.nnet.sigmoid(T.dot(input, self.W) + self.b)

    def _get_reconstructed_input(self, hidden):
        return T.nnet.sigmoid(T.dot(hidden, self.W_prime) + self.b_prime)

    def get_corrupted_input(self, input, corruption_level):
        return self.theano_rng.binomial(size=input.shape, n=1, p=1.0 - corruption_level) * input

    def get_cost_updates(self, corruption_level, learning_rate):
        tilde_x = self.get_corrupted_input(self.x, corruption_level)
        y = self._get_hidden_values(tilde_x)
        z = self._get_reconstructed_input(y)
        cost = T.mean(T.sum((self.x - z)**2, axis=1))

        gparams = T.grad(cost, self.params)

        updates = [(param, param - learning_rate * gparam) for param, gparam in zip(self.params, gparams)]

        return cost, updates

    def compress(self, data):
        from theano.tensor import nnet, dot
        from theano import function

        return function([], nnet.sigmoid(dot(data, self.W) + self.b))()

    def decompress(self, compressed_data):
        from theano.tensor import nnet, dot
        from theano import function

        return function([], nnet.sigmoid(dot(compressed_data, self.W_prime) + self.b_prime))()