def sigmoid(value):
    from math import exp

    return 1.0 / (1.0 + exp(-value))


class Autoencoder2D:
    '''
        Autoencoder for two dimensional input data. It iteratively convolutes and down samples the data until the data
        is a nx1 vector.
    '''

    def __init__(self, size_input_x, size_input_y, size_filter, size_down_sampling):
        from numpy import empty
        from math import log

        self._size_filter = size_filter
        self._size_input_x = size_input_x
        self._size_input_y = size_input_y
        self._size_down_sampling = size_down_sampling


        result = log(size_input_x) / log(size_down_sampling)
        if result != int(result):
            self._num_layers = int(result) + 1
        else:
            self._num_layers = int(result)

        self._cached_y = []
        self._cached_x = []
        self._weights = []
        for i in range(self._num_layers):
            weights = empty([self._size_filter, self._size_filter])
            weights[:] = 1.0 / (size_filter * size_filter)
            self._weights.append(weights)


    def _get(self, layer, i, j):
        dim_x, dim_y = layer.shape
        if i < 0 or j < 0 or i >= dim_x or j >= dim_y:
            return 0
        else:
            return layer[i][j]

    def _convolution(self, previous_y, weights):
        from numpy import empty

        dim_x, dim_y = previous_y.shape
        x = empty([dim_x, dim_y])
        y = empty([dim_x, dim_y])
        for i in range(dim_x):
            for j in range(dim_y):
                activation_sum = 0.0
                for a in range(self._size_filter):
                    for b in range(self._size_filter):
                        activation_sum += weights[a][b] * self._get(previous_y, i + a, j + b)
                x[i][j] = activation_sum
                y[i][j] = sigmoid(activation_sum)

        return x, y

    def _down_sampling(self, previous_y):
        from numpy import empty
        from sys import float_info

        dim_x, dim_y = previous_y.shape
        dim_x /= self._size_down_sampling
        dim_y /= self._size_down_sampling
        y = empty([dim_x, dim_y])
        print("{0} {1}".format(dim_x, dim_y))
        for i in range(dim_x):
            for j in range(dim_y):
                maximum = float_info.min
                for a in range(self._size_down_sampling):
                    for b in range(self._size_down_sampling):
                        maximum = max(maximum, self._get(previous_y, i + a, j + b))
                y[i][j] = maximum
        return y

    def train(self, data):
        pass

    def compress(self, data):
        x, y = self._convolution(data, self._weights[0])
        self._cached_x.append(x)
        self._cached_y.append(y)
        for layer in range(self._num_layers):
            y = self._down_sampling(y)
            x, y = self._convolution(y, self._weights[layer])
            self._cached_x.append(x)
            self._cached_y.append(y)

        return y


    def decompress(self, compressed_data):
        pass

