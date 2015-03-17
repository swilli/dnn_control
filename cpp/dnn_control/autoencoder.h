#ifndef AUTOENCODER_H
#define AUTOENCODER_H

#include "feedforwardneuralnetwork.h"

class Autoencoder  {
public:
    Autoencoder(const std::string &path_to_layer_configurations);

    std::vector<double> Compress(const std::vector<double> &input);

private:
    std::vector<std::vector<double> > ParseWeightMatrix(const std::string &path_to_matrix);
    std::vector<double> ParseBiasVector(const std::string &path_to_vector);

    FeedForwardNeuralNetwork neural_network_;
};

#endif // AUTOENCODER_H
