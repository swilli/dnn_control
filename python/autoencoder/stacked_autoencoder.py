import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams

from mlp import HiddenLayer
from autoencoder import Autoencoder


# start-snippet-1
class StackedAutoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, n_ins=10, hidden_layers_sizes=[40, 20], tied_weights=[True,True],
                 linear_reconstructions=[False, False], autoencoder_weights=None):

        self.sigmoid_layers = []
        self.dA_layers = []
        self.params = []
        self.n_layers = 0
        self.x = T.matrix('x')
        self.y = T.ivector('y')
        self.n_layers = len(hidden_layers_sizes)

        if not theano_rng:
            theano_rng = RandomStreams(numpy_rng.randint(2 ** 30))

        if autoencoder_weights is None:
            for i in xrange(self.n_layers):

                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layers_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.sigmoid_layers[-1].output

                sigmoid_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid)

                self.sigmoid_layers.append(sigmoid_layer)

                self.params.extend(sigmoid_layer.params)

                dA_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=sigmoid_layer.W, bhid=sigmoid_layer.b, tied_weights=tied_weights[i],
                                       linear_reconstruction=linear_reconstructions[i])

                self.dA_layers.append(dA_layer)
        else:
            for i in xrange(len(autoencoder_weights)):
                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layers_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.sigmoid_layers[-1].output

                sigmoid_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid,
                                            W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])

                self.sigmoid_layers.append(sigmoid_layer)

                self.params.extend(sigmoid_layer.params)

                dA_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=sigmoid_layer.W, bhid=sigmoid_layer.b, Whid=autoencoder_weights[i][2],
                                       bvis=autoencoder_weights[i][3], tied_weights=tied_weights[i],
                                       linear_reconstruction=linear_reconstructions[i])

                self.dA_layers.append(dA_layer)

    def pretraining_functions(self, train_set_x, batch_size):
        index = T.lscalar('index')
        learning_rate = T.scalar('lr')
        corruption_level = T.scalar('corruption')
        batch_begin = index * batch_size
        batch_end = batch_begin + batch_size

        pretrain_fns = []
        for dA in self.dA_layers:
            cost, updates = dA.get_cost_updates(corruption_level, learning_rate)
            fn = theano.function(
                inputs=[
                    index,
                    theano.Param(corruption_level, default=0.0),
                    theano.Param(learning_rate, default=0.05)
                ],
                outputs=cost,
                updates=updates,
                givens={
                    self.x: train_set_x[batch_begin: batch_end]
                }
            )
            pretrain_fns.append(fn)

        return pretrain_fns

    def compress(self, data):
        from numpy import array

        output = array(data)
        for da in self.dA_layers:
            output = da.compress(output)

        return output

    def decompress(self, compressed_data):
        from numpy import array

        output = array(compressed_data)
        for da in self.dA_layers[::-1]:
            output = da.decompress(output)

        return output