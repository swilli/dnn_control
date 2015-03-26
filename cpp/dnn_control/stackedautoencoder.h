#ifndef STACKEDAUTOENCODER_H
#define STACKEDAUTOENCODER_H

#include "feedforwardneuralnetwork.h"

class StackedAutoencoder  {
    /*
    * This class represents the compressing part of a stacked autoencoder which is basically a feed forward neural network.
    */
public:
    StackedAutoencoder(const std::string &path_to_layer_configurations);

    // Returns the compressed version of the input by forward passing the input to the output layer
    std::vector<double> Compress(const std::vector<double> &input);

    unsigned int InputSize() const;

    unsigned int OutputSize() const;

    // StackedAutoencoder can throw the following exceptions
    class Exception {};
    class ConfigurationFolderDoesNotExist : public Exception {};

private:
    // The size of the first layer
    unsigned int input_size_;

    // The size of the last layer
    unsigned int output_size_;

    // The Autoencoder weights come from a theano learned stacked autoencoder. This method parses a weight matrix W of one layer
    std::vector<std::vector<double> > ParseWeightMatrix(const std::string &path_to_matrix);

    // The Autoencoder weights come from a theano learned stacked autoencoder. This method parses a bias vector b of one layer
    std::vector<double> ParseBiasVector(const std::string &path_to_vector);

    // The feed forward neural network representing the stacked autoencoder
    FeedForwardNeuralNetwork neural_network_;
};

#endif // STACKEDAUTOENCODER_H
