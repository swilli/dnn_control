import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams

from mlp import HiddenLayer
from autoencoder import Autoencoder


# start-snippet-1
class StackedAutoencoder(object):

    def __init__(self, numpy_rng, theano_rng=None, n_ins=10, n_outs=5, hidden_layer_sizes=[40, 20],
                 tied_weights=[True, True], sigmoid_compressions=[True, True], sigmoid_reconstructions=[True, True],
                 supervised_sigmoid_activation=False, autoencoder_weights=None, supervised_weights=None):

        self.hidden_layers = []
        self.autoencoder_layers = []
        self.params = []
        self.x = T.matrix('x')
        self.y = T.matrix('y')
        self.n_ins = n_ins
        self.n_outs = n_outs
        self.n_layers = len(hidden_layer_sizes)
        self.hidden_layer_sizes = hidden_layer_sizes
        self.sigmoid_compressions = sigmoid_compressions
        self.sigmoid_reconstructions = sigmoid_reconstructions
        self.tied_weights = tied_weights
        self.supervised_sigmoid_activation = supervised_sigmoid_activation

        if theano_rng is None:
            theano_rng = RandomStreams(numpy_rng.randint(2 ** 30))

        self.theano_rng = theano_rng

        if autoencoder_weights is None or supervised_weights is None:
            for i in range(self.n_layers):
                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layer_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.hidden_layers[-1].output

                activation_compression = None
                activation_reconstruction = None
                if sigmoid_compressions[i]:
                    activation_compression = T.nnet.sigmoid
                if sigmoid_reconstructions[i]:
                    activation_reconstruction = T.nnet.sigmoid

                hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                           n_out=hidden_layer_sizes[i], activation=activation_compression)

                self.hidden_layers.append(hidden_layer)
                self.params.extend(hidden_layer.params)

                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                                n_visible=input_size, n_hidden=hidden_layer_sizes[i],
                                                W=hidden_layer.W, bhid=hidden_layer.b, tied_weights=tied_weights[i],
                                                activation_compression=activation_compression,
                                                activation_reconstruction=activation_reconstruction)

                self.autoencoder_layers.append(autoencoder_layer)

            activation = None
            if supervised_sigmoid_activation:
                activation = T.nnet.sigmoid

            # Add a supervised layer on top of it
            self.supervised_layer = HiddenLayer(rng=numpy_rng, input=self.hidden_layers[-1].output,
                                                n_in=hidden_layer_sizes[-1],
                                                n_out=n_outs, activation=activation)

        else:
            for i in range(self.n_layers):
                if i == 0:
                    input_size = n_ins
                else:
                    input_size = hidden_layer_sizes[i - 1]

                if i == 0:
                    layer_input = self.x
                else:
                    layer_input = self.hidden_layers[-1].output

                activation_compression = None
                activation_reconstruction = None
                if sigmoid_compressions[i]:
                    activation_compression = T.nnet.sigmoid
                if sigmoid_reconstructions[i]:
                    activation_reconstruction = T.nnet.sigmoid

                hidden_layer = HiddenLayer(rng=numpy_rng, input=layer_input, n_in=input_size,
                                           n_out=hidden_layer_sizes[i], activation=activation_compression,
                                           W=autoencoder_weights[i][0], b=autoencoder_weights[i][1])

                self.hidden_layers.append(hidden_layer)
                self.params.extend(hidden_layer.params)

                autoencoder_layer = Autoencoder(numpy_rng=numpy_rng, theano_rng=theano_rng, input=layer_input,
                                                n_visible=input_size, n_hidden=hidden_layer_sizes[i],
                                                W=hidden_layer.W, bhid=hidden_layer.b, tied_weights=tied_weights[i],
                                                activation_compression=activation_compression,
                                                activation_reconstruction=activation_reconstruction,
                                                Whid=autoencoder_weights[i][2], bvis=autoencoder_weights[i][3])

                self.autoencoder_layers.append(autoencoder_layer)

            activation = None
            if supervised_sigmoid_activation:
                activation = T.nnet.sigmoid

            # Add a supervised layer on top of it
            self.supervised_layer = HiddenLayer(rng=numpy_rng, input=self.hidden_layers[-1].output,
                                                n_in=hidden_layer_sizes[-1],
                                                n_out=n_outs, activation=activation,
                                                W=supervised_weights[0], b=supervised_weights[1])

        self.params.extend(self.supervised_layer.params)
        self.finetune_cost = T.mean(T.sum((self.supervised_layer.output - self.y)**2, axis=1))

    @classmethod
    def from_config_path(cls, numpy_rng, config_path, theano_rng=None):
        from data_loader import load_config_data

        autoencoder_weights, supervised_weights, config_string = load_config_data(config_path)

        n_ins, n_outs, supervised_sigmoid_activation = [val.replace("\n", "").replace(" ", "") for val in config_string[0].split(",")]
        n_ins = int(n_ins)
        n_outs = int(n_outs)
        supervised_sigmoid_activation = (supervised_sigmoid_activation == 'True')

        hidden_layer_sizes = [int(val) for val in config_string[1].split(",")]
        tied_weights = [val.replace("\n", "").replace(" ", "") == "True" for val in config_string[2].split(",")]
        sigmoid_compressions = [val.replace("\n", "").replace(" ", "") == "True" for val in config_string[3].split(",")]
        sigmoid_reconstructions = [val.replace("\n", "").replace(" ", "") == "True" for val in config_string[4].split(",")]

        return cls(numpy_rng, theano_rng=theano_rng, n_ins=n_ins, n_outs=n_outs, hidden_layer_sizes=hidden_layer_sizes,
                   tied_weights=tied_weights, sigmoid_compressions=sigmoid_compressions,
                   sigmoid_reconstructions=sigmoid_reconstructions,
                   supervised_sigmoid_activation=supervised_sigmoid_activation,
                   autoencoder_weights=autoencoder_weights, supervised_weights=supervised_weights)

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
            output = da.get_hidden_values(output)

        return theano.function([], output)()

    def decompress(self, compressed_data):
        from numpy import array

        output = array(compressed_data)
        for da in self.autoencoder_layers[::-1]:
            output = da.get_reconstructed_input(output)

        return theano.function([], output)()

    def predict(self, data):
        from numpy import array

        tmp = array(data)
        if tmp.ndim == 1:
            tmp = array([data])

        data = tmp
        data = theano.shared(value=data, borrow=True)
        fn = theano.function(
            inputs=[],
            outputs=self.supervised_layer.output,
            givens={
                self.x: data
            },
            name='predict'
        )

        return fn()

    def save(self, save_path):
        for i in range(self.n_layers):
            output_path = save_path + "al{0}W.txt".format(i)
            with open(output_path, 'w+') as output_file:
                W = self.autoencoder_layers[i].W.get_value(borrow=True).T.tolist()
                W_str = "\n".join(", ".join(map(str, value)) for value in W)
                output_file.write(W_str)

            output_path = save_path + "al{0}b.txt".format(i)
            with open(output_path, 'w+') as output_file:
                b = self.autoencoder_layers[i].b.get_value(borrow=True).T.tolist()
                b_str = ", ".join(str(value) for value in b)
                output_file.write(b_str)

            output_path = save_path + "al{0}W_prime.txt".format(i)
            with open(output_path, 'w+') as output_file:
                if self.autoencoder_layers[i].tied_weights:
                    W_prime = self.autoencoder_layers[i].W.get_value(borrow=True).T.T.tolist()
                else:
                    W_prime = self.autoencoder_layers[i].W_prime.get_value(borrow=True).T.tolist()
                W_prime_str = "\n".join(", ".join(map(str, value)) for value in W_prime)
                output_file.write(W_prime_str)

            output_path = save_path + "al{0}b_prime.txt".format(i)
            with open(output_path, 'w+') as output_file:
                b_prime = self.autoencoder_layers[i].b_prime.get_value(borrow=True).T.tolist()
                b_prime_str = ", ".join(str(value) for value in b_prime)
                output_file.write(b_prime_str)

        output_path = save_path + "slW.txt".format(i)
        with open(output_path, 'w+') as output_file:
            W = self.supervised_layer.W.get_value(borrow=True).T.tolist()
            W_str = "\n".join(", ".join(map(str, value)) for value in W)
            output_file.write(W_str)

        output_path = save_path + "slb.txt".format(i)
        with open(output_path, 'w+') as output_file:
            b = self.supervised_layer.b.get_value(borrow=True).T.tolist()
            b_str = ", ".join(str(value) for value in b)
            output_file.write(b_str)

        output_path = save_path + 'conf.txt'
        with open(output_path, 'w+') as output_file:
            out_str = str(self.n_ins) + ", " + str(self.n_outs) + ", " + str(self.supervised_sigmoid_activation) + "\n"
            out_str += ", ".join(str(val) for val in self.hidden_layer_sizes) + "\n"
            out_str += ", ".join(str(val) for val in self.tied_weights) + "\n"
            out_str += ", ".join(str(val) for val in self.sigmoid_compressions) + "\n"
            out_str += ", ".join(str(val) for val in self.sigmoid_reconstructions) + "\n"
            output_file.write(out_str)
