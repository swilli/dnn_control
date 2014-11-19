def sigmoid(value):
    from math import exp

    return 1.0 / (1.0 + exp(-value))


def derivative_sigmoid(value):
    return sigmoid(value) * (1.0 - sigmoid(value))


class DenoisingAutoencoder:
    '''
        DenoisingAutoencoder for one dimensional input data.
    '''

    def __init__(self, size_input, size_compressed_dimension):
        from outputlayer import OutputLayer
        from hiddenlayer import HiddenLayer


        self._hidden_layers = [HiddenLayer(size_input, size_compressed_dimension, sigmoid, derivative_sigmoid, 1.0)]
        self._output_layer = OutputLayer(size_compressed_dimension, size_input, sigmoid, derivative_sigmoid, 1.0)


    def _back_propagate(self, error):
        delta_weights = self._output_layer.back_propagate(error)
        for layer in reversed(self._hidden_layers):
            delta_weights = layer.back_propagate(delta_weights)

    def train(self, data):
        output = self.compress_decompress(data)
        error = data - output
        self._back_propagate(error)
        return output

    def compress_decompress(self, data):
        for layer in self._hidden_layers:
            data = layer.propagate(data)
        return self._output_layer.propagate(data)

