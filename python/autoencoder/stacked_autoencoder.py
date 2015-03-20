import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams

from mlp import HiddenLayer
from autoencoder import Autoencoder


# start-snippet-1
class StackedAutoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, n_ins=10, hidden_layers_sizes=[40, 20], tied_weights=[True, True],
                 sigmoid_compressions=[True, True], sigmoid_reconstructions=[True, True],
                 autoencoder_weights=None):

        self.mlp_layers = []
        self.autoencoder_layers = []
        self.params = []
        self.n_layers = 0
        self.x = T.matrix('x')
        self.y = T.ivector('y')
        self.n_layers = len(hidden_layers_sizes)

        if not theano_rng:
            theano_rng = RandomStreams(numpy_rng.randint(2 ** 30))

        self.theano_rng = theano_rng

        if autoencoder_weights is None:
            for i in xrange(self.n_layers):

                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layers_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.mlp_layers[-1].output

                if sigmoid_compressions[i]:
                    mlp_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid)
                else:
                    mlp_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=None)

                self.mlp_layers.append(mlp_layer)


                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=mlp_layer.W, bhid=mlp_layer.b, tied_weights=tied_weights[i],
                                       sigmoid_compression=sigmoid_compressions[i],
                                       sigmoid_reconstruction=sigmoid_reconstructions[i])

                self.params.extend(autoencoder_layer.params)

                self.autoencoder_layers.append(autoencoder_layer)
        else:
            for i in xrange(len(autoencoder_weights)):
                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layers_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.mlp_layers[-1].output

                if sigmoid_compressions[i]:
                    mlp_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid,
                                            W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])
                else:
                    mlp_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=None,
                                            W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])

                self.mlp_layers.append(mlp_layer)

                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=mlp_layer.W, bhid=mlp_layer.b, Whid=autoencoder_weights[i][2],
                                       bvis=autoencoder_weights[i][3], tied_weights=tied_weights[i],
                                       sigmoid_compression=sigmoid_compressions[i],
                                       sigmoid_reconstruction=sigmoid_reconstructions[i])

                self.params.extend(autoencoder_layer.params)

                self.autoencoder_layers.append(autoencoder_layer)

    def get_corrupted_input(self, input, corruption_level):
        #return self.theano_rng.binomial(size=input.shape, n=1, p=1.0 - corruption_level) * input
        return input + input * self.theano_rng.normal(size=input.shape, avg=0.0, std=corruption_level)

    def pretraining_functions(self, train_set_x, batch_size):
        index = T.lscalar('index')
        learning_rate = T.scalar('lr')
        corruption_level = T.scalar('corruption')
        batch_begin = index * batch_size
        batch_end = batch_begin + batch_size

        pretrain_fns = []
        for dA in self.autoencoder_layers:
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

    def finetune_function(self, train_set_x, batch_size):
        index = T.lscalar('index')
        learning_rate = T.scalar('lr')
        corruption_level = T.scalar('corruption')
        batch_begin = index * batch_size
        batch_end = batch_begin + batch_size

        result = self.get_corrupted_input(self.x, corruption_level)

        for enc in self.autoencoder_layers:
            result = enc.get_hidden_values(result)

        for enc in self.autoencoder_layers[::-1]:
            result = enc.get_reconstructed_input(result)

        cost = T.mean(T.sum((self.x - result)**2, axis=1))

        gparams = T.grad(cost, self.params)

        updates = [(param, param - learning_rate * gparam) for param, gparam in zip(self.params, gparams)]

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

        return fn

    def compress(self, data):
        from numpy import array

        output = array(data)
        for da in self.autoencoder_layers:
            output = da.compress(output)

        return output

    def decompress(self, compressed_data):
        from numpy import array

        output = array(compressed_data)
        for da in self.autoencoder_layers[::-1]:
            output = da.decompress(output)

        return output