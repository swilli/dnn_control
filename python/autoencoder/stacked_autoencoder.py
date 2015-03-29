import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams

from mlp import HiddenLayer
from autoencoder import Autoencoder


# start-snippet-1
class StackedAutoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, n_ins=10, n_outs=5, hidden_layers_sizes=[40, 20], tied_weights=[True, True],
                 sigmoid_compressions=[True, True], sigmoid_reconstructions=[True, True],
                 autoencoder_weights=None):

        self.hidden_layers = []
        self.autoencoder_layers = []
        self.params = []
        self.n_layers = 0
        self.x = T.matrix('x')
        self.y = T.matrix('y')
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
                    layer_input = self.hidden_layers[-1].output

                if sigmoid_compressions[i]:
                    hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid)
                else:
                    hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=None)

                self.hidden_layers.append(hidden_layer)
                self.params.extend(hidden_layer.params)

                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=hidden_layer.W, bhid=hidden_layer.b, tied_weights=tied_weights[i],
                                       sigmoid_compression=sigmoid_compressions[i],
                                       sigmoid_reconstruction=sigmoid_reconstructions[i])


                self.autoencoder_layers.append(autoencoder_layer)

            # Add a supervised layer on top of it
            self.supervised_layer = HiddenLayer(rng=numpy_rng, input=self.hidden_layers[-1].output,
                                            n_in=hidden_layers_sizes[-1],
                                            n_out=n_outs, activation=None)

            self.params.extend(self.supervised_layer.params)

            self.finetune_cost = T.mean(T.sum((self.supervised_layer.output - self.y)**2, axis=1))

        else:
            for i in xrange(len(autoencoder_weights)):
                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layers_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.hidden_layers[-1].output

                if sigmoid_compressions[i]:
                    hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=T.nnet.sigmoid,
                                            W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])
                else:
                    hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                            n_out=hidden_layers_sizes[i], activation=None,
                                            W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])

                self.hidden_layers.append(hidden_layer)

                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                       n_visible=input_size, n_hidden=hidden_layers_sizes[i],
                                       W=hidden_layer.W, bhid=hidden_layer.b, Whid=autoencoder_weights[i][2],
                                       bvis=autoencoder_weights[i][3], tied_weights=tied_weights[i],
                                       sigmoid_compression=sigmoid_compressions[i],
                                       sigmoid_reconstruction=sigmoid_reconstructions[i])

                self.params.extend(autoencoder_layer.params)

                self.autoencoder_layers.append(autoencoder_layer)

            # Add a supervised layer on top of it
            #self.supervised_layer = HiddenLayer(rng=numpy_rng, input=self.hidden_layers[-1].output,
                                            #n_in=hidden_layers_sizes[-1],
                                            #n_out=n_outs, W=autoencoder_weights[-1][0], b=autoencoder_weights[-1][1],
                                            #activation=None)

            #self.params.extend(self.supervised_layer.params)

            #self.finetune_cost = T.mean(T.sum((self.supervised_layer.output - self.y)**2, axis=1))

    def pretraining_functions(self, training_set, batch_size):
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
                    self.x: training_set[batch_begin: batch_end]
                }
            )

            pretrain_fns.append(fn)

        return pretrain_fns

    def finetune_functions(self, training_set, training_labels, test_set, test_labels, batch_size, learning_rate):
        index = T.lscalar('index')  # index to a [mini]batch

        # compute number of minibatches for training, validation and testing
        n_valid_batches = test_set.get_value(borrow=True).shape[0]
        n_valid_batches /= batch_size

        # compute the gradients with respect to the model parameters
        gparams = T.grad(self.finetune_cost, self.params)

        # compute list of fine-tuning updates
        updates = [(param, param - gparam * learning_rate) for param, gparam in zip(self.params, gparams)]

        train_fn = theano.function(
            inputs=[index],
            outputs=self.finetune_cost,
            updates=updates,
            givens={
                self.x: training_set[
                    index * batch_size: (index + 1) * batch_size
                ],
                self.y: training_labels[
                    index * batch_size: (index + 1) * batch_size
                ]
            },
            name='train'
        )

        valid_score_i = theano.function(
            [index],
            self.finetune_cost,
            givens={
                self.x: test_set[
                    index * batch_size: (index + 1) * batch_size
                ],
                self.y: test_labels[
                    index * batch_size: (index + 1) * batch_size
                ]
            },
            name='valid'
        )

        # Create a function that scans the entire validation set
        def valid_score():
            return [valid_score_i(i) for i in xrange(n_valid_batches)]

        return train_fn, valid_score

    def finetune_functions_unsupervised(self, training_set, test_set, batch_size, learning_rate):
        index = T.lscalar('index')  # index to a [mini]batch

        # compute number of minibatches for training, validation and testing
        n_valid_batches = test_set.get_value(borrow=True).shape[0]
        n_valid_batches /= batch_size

        params = []

        z = self.autoencoder_layers[0].get_corrupted_input(self.x, 0.05)
        for dA in self.autoencoder_layers:
            params.extend(dA.params)
            z = dA.get_hidden_values(z)

        for dA in self.autoencoder_layers[::-1]:
            z = dA.get_reconstructed_input(z)

        cost = T.mean(T.sum((z - self.x)**2, axis=1))

        # compute the gradients with respect to the model parameters
        gparams = T.grad(cost, params)

        # compute list of fine-tuning updates
        updates = [(param, param - gparam * learning_rate) for param, gparam in zip(params, gparams)]

        train_fn = theano.function(
            inputs=[index],
            outputs=cost,
            updates=updates,
            givens={
                self.x: training_set[
                    index * batch_size: (index + 1) * batch_size
                ]
            },
            name='train'
        )

        valid_score_i = theano.function(
            inputs=[index],
            outputs=cost,
            givens={
                self.x: test_set[
                    index * batch_size: (index + 1) * batch_size
                ]
            },
            name='valid'
        )

        # Create a function that scans the entire validation set
        def valid_score():
            return [valid_score_i(i) for i in xrange(n_valid_batches)]

        return train_fn, valid_score

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