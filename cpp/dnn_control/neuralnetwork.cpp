#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork(const unsigned int &size)
    : size_(size) {

}

NeuralNetwork::~NeuralNetwork()
{

}

unsigned int NeuralNetwork::Size() const {
    return size_;
}

unsigned int NeuralNetwork::TotalSizeForNetworkConfiguration(const std::vector<std::pair<unsigned int, bool> > &network_configuration) {
    unsigned int size = 0;

    for (unsigned int i = 1; i < network_configuration.size(); ++i) {
        if (network_configuration.at(i-1).second) {
            size += (network_configuration.at(i-1).first + 1) * network_configuration.at(i).first;
        } else {
            size += network_configuration.at(i-1).first * network_configuration.at(i).first;
        }
    }

    return size;
}

