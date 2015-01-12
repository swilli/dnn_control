def sigmoid(value):
    from math import exp

    return 1.0 / (1.0 + exp(-value))


def derivative_sigmoid(value):
    return sigmoid(value) * (1.0 - sigmoid(value))


class NeuralNetwork:
    def __init__(self):
        from test.outputlayer import OutputLayer
        from test.hiddenlayer import HiddenLayer

        from numpy import array

        self._hidden_layers = [HiddenLayer(2, 2, sigmoid, derivative_sigmoid, 1.0, array([[0.1, 0.4], [0.8, 0.6]]))]
        self._output_layer = OutputLayer(2, 1, sigmoid, derivative_sigmoid, 1.0, array([[0.3], [0.9]]))


    def _back_propagate(self, error):
        delta_weights = self._output_layer.back_propagate(error)
        for layer in reversed(self._hidden_layers):
            delta_weights = layer.back_propagate(delta_weights)


    def train(self, data, target):
        output = self.forward_pass(data)
        error = target - output
        self._back_propagate(error)
        return output


    def forward_pass(self, data):
        for layer in self._hidden_layers:
            data = layer.propagate(data)
        return self._output_layer.propagate(data)