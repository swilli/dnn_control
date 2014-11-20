class OutputLayer:

    def __init__(self, size_input, size_output, fun_activation, fun_ddt_activation, learning_rate, weights=None):
        from numpy import empty

        self.size_input = size_input
        self.size_output = size_output
        self._weights = empty([self.size_input, self.size_output])
        if weights is None:
            self._weights[:] = 1.0 / (self.size_output * self.size_input)
        else:
            self._weights = weights
        self._input_values = empty([self.size_input, 1])
        self._d_dt_activation_values = empty([self.size_output, 1])
        self._fun_activation = fun_activation
        self._fun_ddt_activation = fun_ddt_activation
        self._learning_rate = learning_rate

    def propagate(self, data):
        from numpy import array, empty

        self._input_values = data
        result = empty([self.size_output, 1])
        for i in range(self.size_output):
            result[i] = 0.0
            for j in range(self.size_input):
                result[i] += self._weights[j][i] * data[j]

        self._d_dt_activation_values = array([self._fun_ddt_activation(val) for val in result])
        return array([self._fun_activation(val) for val in result])

    def back_propagate(self, error):
        from numpy import empty

        deltas = empty([self.size_output, 1])
        delta_weights = empty([self.size_input, self.size_output])

        for i in range(self.size_output):
            deltas[i] = error[i] * self._d_dt_activation_values[i]

        for i in range(self.size_input):
            for j in range(self.size_output):
                self._weights[i][j] += self._learning_rate * deltas[j] * self._input_values[i]

        for i in range(self.size_input):
            for j in range(self.size_output):
                delta_weights[i][j] = deltas[j] * self._weights[i][j]

        return delta_weights