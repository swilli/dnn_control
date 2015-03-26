import numpy

import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams


class Autoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, input=None, n_visible=10, n_hidden=5, tied_weights=True,
                 sigmoid_compression=True, sigmoid_reconstruction=True, W=None, Whid=None, bhid=None, bvis=None):

        self.n_visible = n_visible
        self.n_hidden = n_hidden
        self.tied_weights = tied_weights
        self.sigmoid_compression = sigmoid_compression
        self.sigmoid_reconstruction = sigmoid_reconstruction

        if not theano_rng:
            theano_rng = RandomStreams(numpy_rng.randint(2 ** 30))

        self.theano_rng = theano_rng

        if not W:
            initial_W = numpy.asarray(numpy_rng.uniform(low=-numpy.sqrt(6. / (n_hidden + n_visible)),
                                                        high=numpy.sqrt(6. / (n_hidden + n_visible)),
                                                        size=(n_visible, n_hidden)), dtype=theano.config.floatX)
            if self.sigmoid_compression:
                initial_W *= 4

            W = theano.shared(value=initial_W, name='W', borrow=True)

        if not tied_weights and not Whid:
            initial_Whid = numpy.asarray(numpy_rng.uniform(low=-numpy.sqrt(6. / (n_hidden + n_visible)),
                                                           high=numpy.sqrt(6. / (n_hidden + n_visible)),
                                                           size=(n_hidden, n_visible)), dtype=theano.config.floatX)
            if self.sigmoid_reconstruction:
                initial_Whid *= 4

            Whid = theano.shared(value=initial_Whid, borrow=True)

        if not bvis:
            bvis = theano.shared(value=numpy.zeros(n_visible, dtype=theano.config.floatX), borrow=True)

        if not bhid:
            bhid = theano.shared(value=numpy.zeros(n_hidden, dtype=theano.config.floatX), name='b', borrow=True)

        self.W = W
        self.b = bhid
        self.b_prime = bvis
        if self.tied_weights:
            self.W_prime = self.W.T
        else:
            self.W_prime = Whid

        if input is None:
            self.x = T.dmatrix(name='input')
        else:
            self.x = input

        if self.tied_weights:
            self.params = [self.W, self.b, self.b_prime]
        else:
            self.params = [self.W, self.b, self.W_prime, self.b_prime]

    def get_hidden_values(self, input):
        if self.sigmoid_compression:
            return T.nnet.sigmoid(T.dot(input, self.W) + self.b)
        else:
            return T.dot(input, self.W) + self.b

    def get_reconstructed_input(self, hidden):
        if self.sigmoid_reconstruction:
            return T.nnet.sigmoid(T.dot(hidden, self.W_prime) + self.b_prime)
        else:
            return T.dot(hidden, self.W_prime) + self.b_prime

    def get_corrupted_input(self, input, corruption_level):
        return self.theano_rng.binomial(size=input.shape, n=1, p=1.0 - corruption_level) * input
        #return input + input * self.theano_rng.normal(size=input.shape, avg=0.0, std=corruption_level)

    def get_cost_updates(self, corruption_level, learning_rate):
        tilde_x = self.get_corrupted_input(self.x, corruption_level)
        y = self.get_hidden_values(tilde_x)
        z = self.get_reconstructed_input(y)
        cost = T.mean(T.sum((self.x - z)**2, axis=1))

        gparams = T.grad(cost, self.params)

        updates = [(param, param - learning_rate * gparam) for param, gparam in zip(self.params, gparams)]

        return cost, updates

    def compress(self, data):
        from theano.tensor import nnet, dot
        from theano import function

        if self.sigmoid_compression:
            return function([], nnet.sigmoid(dot(data, self.W) + self.b))()
        else:
            return function([], dot(data, self.W) + self.b)()

    def decompress(self, compressed_data):
        from theano.tensor import nnet, dot
        from theano import function

        if self.sigmoid_reconstruction:
            return function([], nnet.sigmoid(dot(compressed_data, self.W_prime) + self.b_prime))()
        else:
            return function([], dot(compressed_data, self.W_prime) + self.b_prime)()